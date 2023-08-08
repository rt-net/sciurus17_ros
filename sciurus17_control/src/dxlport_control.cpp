#include    <ros/ros.h>
#include    <ros/package.h>
#include    <angles/angles.h>
#include    <sciurus17_control/dxlport_control.h>
#include    <math.h>
#include    <stdio.h>

/* MACRO */
#define     DXLPOS2RAD(pos)    (( ((pos)*(360.0/POSITION_STEP) )/360) *2*M_PI)
#define     RAD2DXLPOS(rad)    (( ((rad)/2.0/M_PI)*360.0 ) * (POSITION_STEP / 360.0))
#define     INIT_EFFCNST_UNIT(c) ((c)*0.001)

//#define     MASTER_WAIST_SLAVE_NECK

DXLPORT_CONTROL::DXLPORT_CONTROL( ros::NodeHandle handle, CONTROL_SETTING &setting )
{
    int jj;
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;
    int position_mode_joint_num = 0;
    int current_mode_joint_num = 0;
    int current_position_mode_joint_num = 0;

    init_stat = false;
    tx_err = rx_err = 0;
    tempCount = 0;
    tempTime = getTime();

    portHandler      = NULL;
    writeGoalGroup   = NULL;
    writePosGoalGroup= NULL;
    readTempGroup    = NULL;
    readIndirectGroup= NULL;

    joint_num = setting.getjointNum();
    std::vector<ST_SERVO_PARAM> list = setting.getServoParam();
    for( jj=0 ; jj<joint_num ; ++jj ){
        JOINT_CONTROL work( list[jj].name, list[jj].id, list[jj].center, list[jj].home, list[jj].eff_cnst, list[jj].mode );
        joints.push_back( work );
    }

    dev_mtx = new DEVICE_MUTEX( setting.getPortName().c_str() );

    /* DynamixelSDKとros_controlへの接続初期化 */
    packetHandler    = dynamixel::PacketHandler::getPacketHandler( PROTOCOL_VERSION );
    portHandler      = dynamixel::PortHandler::getPortHandler( setting.getPortName().c_str() );
    writeGoalGroup   = new dynamixel::GroupBulkWrite( portHandler, packetHandler );
    writePosGoalGroup= new dynamixel::GroupBulkWrite( portHandler, packetHandler );
    readTempGroup    = new dynamixel::GroupBulkRead( portHandler, packetHandler );
    readIndirectGroup= new dynamixel::GroupBulkRead( portHandler, packetHandler );
    
    for( jj=0 ; jj<joint_num ; ++jj ){
        uint8_t dxl_id = joints[jj].get_dxl_id();
        if( joints[jj].get_ope_mode() == OPERATING_MODE_CURRENT ){
            //CURRENT MODE
            if( !writeGoalGroup->addParam( dxl_id, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, joints[jj].get_dxl_goal_addr() ) ){// [TODO]
                error_queue.push("Bulk current write setting failed.");
                return;
            }
        }else if( joints[jj].get_ope_mode() == OPERATING_MODE_CURR_POS ){
            //CURRENT base POSITION MODE
            if( !writePosGoalGroup->addParam( dxl_id, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, joints[jj].get_dxl_goal_addr() ) ){// [TODO]
                error_queue.push("Bulk pos write setting failed.");
                return;
            }
        }else{
            //POSITION MODE
            if( !writeGoalGroup->addParam( dxl_id, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, joints[jj].get_dxl_goal_addr() ) ){// [TODO]
                error_queue.push("Bulk pos write setting failed.");
                return;
            }
        }
        if( !readTempGroup->addParam( dxl_id, ADDR_PRESENT_TEMP, LEN_PRESENT_TEMP ) ){
            error_queue.push("Bulk temp read setting failed.");
            return;
        }
        if( !readIndirectGroup->addParam( dxl_id, DATA_INDIRECT_TOP, LEN_INDIRECT_GROUP ) ){
            error_queue.push("Bulk group read setting failed.");
            return;
        }
    }

    // Open port
    lock_port();
    if( !portHandler->openPort() ){
        error_queue.push("Port open failed.");
        port_stat = false;
    }else{
        // Set port baudrate
        if( !portHandler->setBaudRate( setting.getBaudrate() ) ){
            error_queue.push("Setup baudrate failed.");
            port_stat = false;
        }else{
            port_stat = true; // 有効と仮定してread
            for( jj=0 ; jj<joint_num ; ++jj ){
                uint8_t dxl_id = joints[jj].get_dxl_id();
                setup_indirect( dxl_id );
            }
            if( !read( ros::Time::now(), ros::Duration(0) ) ){
                error_queue.push("Initialize communication failed.");
                port_stat = false;
            }
        }
    }
    read( ros::Time::now(), ros::Duration(0) );
    unlock_port();

    for( jj=0 ; jj<joint_num ; ++jj ){
        hardware_interface::JointStateHandle reg_state_handle( joints[jj].get_joint_name(), joints[jj].get_position_addr(), joints[jj].get_velocity_addr(), joints[jj].get_effort_addr() );
        joint_stat_if.registerHandle( reg_state_handle );
    }
    registerInterface( &joint_stat_if );

    for( jj=0 ; jj<joint_num ; ++jj ){
        hardware_interface::JointHandle reg_joint_handle( joint_stat_if.getHandle(joints[jj].get_joint_name()), joints[jj].get_command_addr() );
        if( joints[jj].get_ope_mode() == OPERATING_MODE_CURRENT ){
            joint_eff_if.registerHandle( reg_joint_handle );
            ++current_mode_joint_num;
        }else{//POS & Current base POS
            joint_pos_if.registerHandle( reg_joint_handle );
            ++position_mode_joint_num;
        }
        // Get limits
        if( joint_limits_interface::getJointLimits( joints[jj].get_joint_name(), handle, limits ) ){
            joints[jj].set_limits( limits );
            soft_limits.k_position = 1.0;
            soft_limits.k_velocity = 1.0;
            soft_limits.max_position = limits.max_position;
            soft_limits.min_position = limits.min_position;
            joint_limits_interface::PositionJointSoftLimitsHandle
                            reg_limits_handle( reg_joint_handle, limits, soft_limits );
            joint_limits_if.registerHandle( reg_limits_handle );
        }
    }
    if( position_mode_joint_num > 0 ){
        registerInterface( &joint_pos_if );
    }
    if( current_mode_joint_num > 0 ){
        registerInterface( &joint_eff_if );
    }

    init_stat = true;
}

DXLPORT_CONTROL::~DXLPORT_CONTROL()
{
    portHandler->closePort();
	delete( portHandler );
    delete( dev_mtx );
    if(readTempGroup!=NULL)    delete( readTempGroup );
    if(writeGoalGroup!=NULL)    delete( writeGoalGroup );
    if(writePosGoalGroup!=NULL)    delete( writePosGoalGroup );
    if(readIndirectGroup!=NULL)     delete( readIndirectGroup );
}

bool DXLPORT_CONTROL::read( ros::Time time, ros::Duration period )
{
    bool result = false;

    if( !port_stat ){
        return true;
    }

    int dxl_comm_result = readIndirectGroup->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++rx_err;
    }else{
        if( readId(time, period) && readCurrent( time, period ) && readVel( time, period ) && readPos( time, period ) && readTemp( time, period ) ){
            result = true;
        }
    }

    return result;
}

bool DXLPORT_CONTROL::readId( ros::Time time, ros::Duration period )
{
    bool result = false;

    for( int jj=0 ; jj<joint_num ; ++jj ){
        uint8_t get_id = 0;
        uint8_t dxl_id = joints[jj].get_dxl_id();
        bool dxl_getdata_result = readIndirectGroup->isAvailable( dxl_id, ADDR_INDIRECT_DXLID, LEN_INDIRECT_DXLID );
        if( !dxl_getdata_result ){
            ++rx_err;
            ROS_INFO("readId error [%d]",dxl_id);
            break;
        }else{
            get_id = readIndirectGroup->getData( dxl_id, ADDR_INDIRECT_DXLID, LEN_INDIRECT_DXLID );
            if( get_id == dxl_id ){
                result = true;
            }else{
                break;
            }
        }
    }
    return result;
}

bool DXLPORT_CONTROL::readPos( ros::Time time, ros::Duration period )
{
    bool result = false;

    for( int jj=0 ; jj<joint_num ; ++jj ){
        int32_t present_pos = 0;
        uint8_t dxl_id = joints[jj].get_dxl_id();
        bool dxl_getdata_result = readIndirectGroup->isAvailable( dxl_id, ADDR_INDIRECT_POSITION, LEN_PRESENT_POSITION );
        if( !dxl_getdata_result ){
            ++rx_err;
            break;
        }else{
            present_pos = readIndirectGroup->getData( dxl_id, ADDR_INDIRECT_POSITION, LEN_PRESENT_POSITION );
            joints[jj].set_dxl_pos( present_pos );
            present_pos = (present_pos - joints[jj].get_center());
            joints[jj].set_position( DXLPOS2RAD( present_pos ) );
            result = true;
        }
    }
    return result;
}

bool DXLPORT_CONTROL::readCurrent( ros::Time time, ros::Duration period )
{
    bool result = false;

    for( int jj=0 ; jj<joint_num ; ++jj ){
        int16_t present_current = 0;
        uint8_t dxl_id = joints[jj].get_dxl_id();
        bool dxl_getdata_result = readIndirectGroup->isAvailable( dxl_id, ADDR_INDIRECT_CURRENT, LEN_PRESENT_CURRENT );
        if( !dxl_getdata_result ){
            ++rx_err;
            break;
        }else{
            present_current = readIndirectGroup->getData( dxl_id, ADDR_INDIRECT_CURRENT, LEN_PRESENT_CURRENT );
            joints[jj].set_dxl_curr( present_current );
            joints[jj].set_current( (DXL_CURRENT_UNIT * present_current) );
            joints[jj].set_effort( DXL_CURRENT2EFFORT( present_current, joints[jj].get_eff_const() ) );
            result = true;
        }
    }
    return result;
}

bool DXLPORT_CONTROL::readTemp( ros::Time time, ros::Duration period )
{
    bool result = false;

    for( int jj=0 ; jj<joint_num ; ++jj ){
        uint8_t dxl_id = joints[jj].get_dxl_id();
        bool dxl_getdata_result = readIndirectGroup->isAvailable( dxl_id, ADDR_INDIRECT_TEMP, LEN_PRESENT_TEMP );
        if( !dxl_getdata_result ){
            ++rx_err;
            break;
        }else{
            uint8_t present_temp = readIndirectGroup->getData( dxl_id, ADDR_INDIRECT_TEMP, LEN_PRESENT_TEMP );
            joints[jj].set_dxl_temp( present_temp );
            joints[jj].set_temprature( present_temp );
            result = true;
        }
    }
    ++tempCount;
    return result;
}

bool DXLPORT_CONTROL::readVel( ros::Time time, ros::Duration period )
{
    bool result = false;

    for( int jj=0 ; jj<joint_num ; ++jj ){
        uint8_t dxl_id = joints[jj].get_dxl_id();
        bool dxl_getdata_result = readIndirectGroup->isAvailable( dxl_id, ADDR_INDIRECT_VELOCITY, LEN_PRESENT_VEL );
        if( !dxl_getdata_result ){
            ++rx_err;
            break;
        }else{
            int32_t present_velocity = readIndirectGroup->getData( dxl_id, ADDR_INDIRECT_VELOCITY, LEN_PRESENT_VEL );
            joints[jj].set_velocity( DXL_VELOCITY2RAD_S(present_velocity) );
            result = true;
        }
    }
    return result;
}

void DXLPORT_CONTROL::write( ros::Time time, ros::Duration period )
{
    double get_cmd;
    float* tau;

    if( !port_stat){
        for( int jj=0 ; jj<joint_num ; ++jj ){
            get_cmd = joints[jj].get_command();
            joints[jj].updt_d_command( get_cmd );
            joints[jj].set_position( get_cmd );
        }
        return;
    }

    for( int jj=0 ; jj<joint_num ; ++jj ){
#ifdef MASTER_WAIST_SLAVE_NECK
            double slave_pos_data;
            // Slave pos function
            if( joints[jj].get_dxl_id() == 18 ){
                slave_pos_data = joints[jj].get_command();
            }
            if( joints[jj].get_dxl_id() == 19 ){
                joints[jj].set_command( -slave_pos_data );
            }
#endif
        get_cmd = joints[jj].get_command();
        if( joints[jj].get_ope_mode() == OPERATING_MODE_CURRENT ){
            // Current control
            double work_cur = EFFORT2DXL_CURRENT( get_cmd, joints[jj].get_eff_const() );
            joints[jj].updt_d_command( 0.0 );

            uint16_t dxl_cur = (uint32_t)round( work_cur );
            uint8_t* goal_data = joints[jj].get_dxl_goal_addr();
            goal_data[0] = (uint8_t)(dxl_cur&0x000000FF);
            goal_data[1] = (uint8_t)((dxl_cur&0x0000FF00)>>8);

            writeGoalGroup->changeParam( joints[jj].get_dxl_id(), ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, goal_data );
        }else if(joints[jj].get_ope_mode() == OPERATING_MODE_CURR_POS ){
            // Current update
            uint16_t dxl_cur = (uint32_t)round( tau[jj] );
            uint8_t* goal_data = joints[jj].get_dxl_goal_addr();
            goal_data[0] = (uint8_t)(dxl_cur&0x000000FF);
            goal_data[1] = (uint8_t)((dxl_cur&0x0000FF00)>>8);
            writeGoalGroup->changeParam( joints[jj].get_dxl_id(), ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, goal_data );
        }else{
            // Position control
            double work_pos = RAD2DXLPOS( get_cmd );
            joints[jj].updt_d_command( get_cmd );
            work_pos += joints[jj].get_center();          // ROS(-180 <=> +180) => DXL(0 <=> 4095)
            if( work_pos < DXL_MIN_LIMIT ){
                work_pos = DXL_MIN_LIMIT;
            }
            if( work_pos > DXL_MAX_LIMIT ){
                work_pos = DXL_MAX_LIMIT;
            }

            uint32_t dxl_pos = (uint32_t)round( work_pos );
            uint8_t* goal_data = joints[jj].get_dxl_goal_addr();

            goal_data[0] = (uint8_t)(dxl_pos&0x000000FF);
            goal_data[1] = (uint8_t)((dxl_pos&0x0000FF00)>>8);
            goal_data[2] = (uint8_t)((dxl_pos&0x00FF0000)>>16);
            goal_data[3] = (uint8_t)((dxl_pos&0xFF000000)>>24);

            writeGoalGroup->changeParam( joints[jj].get_dxl_id(), ADDR_GOAL_POSITION, LEN_GOAL_POSITION, goal_data );
        }
    }
    int dxl_comm_result = writeGoalGroup->txPacket();
    if( dxl_comm_result != COMM_SUCCESS ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++tx_err;
    }
}

bool DXLPORT_CONTROL::is_change_positions( void )
{
    bool result = false;

    for( int jj=0 ; jj<joint_num ; ++jj ){
        if( fabs( joints[jj].get_d_command() ) > 0.0 ){
            result = true;
            break;
        }
    }
    return result;
}

void DXLPORT_CONTROL::set_gain_all( uint16_t gain )
{
    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num ; ++jj ){
        if( joints[jj].get_ope_mode() == OPERATING_MODE_CURRENT ){
            continue;
        }
        set_gain( joints[jj].get_dxl_id(), gain );
    }
}

void DXLPORT_CONTROL::set_gain( uint8_t dxl_id, uint16_t gain )
{
    uint8_t dxl_error = 0;                          // Dynamixel error

    lock_port();
    int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_POSITION_PGAIN, gain, &dxl_error );
    unlock_port();
    if( dxl_comm_result != COMM_SUCCESS ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++tx_err;
    }else if( dxl_error != 0 ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
        ++tx_err;
    }
}

void DXLPORT_CONTROL::set_goal_current_all( uint16_t current )
{
    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num ; ++jj ){
        if( joints[jj].get_ope_mode() == OPERATING_MODE_CURRENT ){
            set_goal_current( joints[jj].get_dxl_id(), current );
        }
    }
}


void DXLPORT_CONTROL::set_goal_current( uint8_t dxl_id, uint16_t current )
{
    uint8_t dxl_error = 0;                          // Dynamixel error

    lock_port();
    int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_GOAL_CURRENT, current, &dxl_error );
    unlock_port();
    if( dxl_comm_result != COMM_SUCCESS ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++tx_err;
    }else if( dxl_error != 0 ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
        ++tx_err;
    }
}

bool DXLPORT_CONTROL::set_torque( uint8_t dxl_id, bool torque )
{
    uint32_t set_param = torque ? TORQUE_ENABLE:TORQUE_DISABLE;
    bool result = false;

    if( !port_stat ){
        return true;
    }

    uint8_t dxl_error = 0; // Dynamixel error
    lock_port();
    int dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_TORQUE_ENABLE, set_param, &dxl_error );
    unlock_port();
    if( dxl_comm_result != COMM_SUCCESS ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++tx_err;
    }else if( dxl_error != 0 ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
        ++tx_err;
    }else{
        result = true;
    }
    return result;
}
void DXLPORT_CONTROL::set_torque_all( bool torque )
{
    for( uint8_t jj=0 ; jj<joint_num; ++jj ){
        if( set_torque( joints[jj].get_dxl_id(), torque ) ){
            joints[jj].set_torque( torque );
        }
    }
}

void DXLPORT_CONTROL::set_watchdog( uint8_t dxl_id, uint8_t value )
{
    if( !port_stat ){
        return;
    }

    uint8_t dxl_error = 0; // Dynamixel error
    lock_port();
    int dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_BUS_WATCHDOG, value, &dxl_error );
    unlock_port();
    if( dxl_comm_result != COMM_SUCCESS ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++tx_err;
    }else if( dxl_error != 0 ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
        ++tx_err;
    }
}

void DXLPORT_CONTROL::set_watchdog_all( uint8_t value )
{
    for( uint8_t jj=0 ; jj<joint_num; ++jj ){
        set_watchdog( joints[jj].get_dxl_id(), value );
    }
}

/* 起動時モーション */
void DXLPORT_CONTROL::startup_motion( void )
{
    ros::Rate rate( DXL_TORQUR_ON_STEP );
    int step_max = DXL_TORQUE_ON_STEP_MAX;
    ros::Time t = getTime();
    ros::Duration d = getDuration(t);
    std::vector<ST_HOME_MOTION_DATA> home_motion_data;

    /* 開始位置取り込みと差分計算 */

    lock_port();
    read( t, d );
    unlock_port();

    for( int jj=0 ; jj<joint_num ; ++jj ){
        ST_HOME_MOTION_DATA motion_work;
        motion_work.home      = joints[jj].get_home();
        motion_work.home_rad  = DXLPOS2RAD( motion_work.home ) - DXLPOS2RAD( joints[jj].get_center() );
        motion_work.start_rad = joints[jj].get_position();
        motion_work.start     = RAD2DXLPOS( motion_work.start_rad ) + joints[jj].get_center();
        motion_work.step_rad  = 
            (motion_work.home > motion_work.start) ? ((motion_work.home_rad - motion_work.start_rad)/step_max)
                                                   : -((motion_work.start_rad - motion_work.home_rad)/step_max);
        if( joints[jj].get_ope_mode() == OPERATING_MODE_CURRENT ){
            joints[jj].set_command( 0.0 );
        }else{
            joints[jj].set_command( joints[jj].get_position() );
        }
        home_motion_data.push_back( motion_work );
    }
    lock_port();
    write( t, d );
    unlock_port();

    set_torque_all( true );                         // 全関節トルクON
    set_gain_all( DXL_DEFAULT_PGAIN );

    /* ホームポジションへ移動する */
    for( int step=0 ; step<step_max ; ++step ){
        d = getDuration(t);
        t = getTime();
        if( !port_stat ){
            for( int jj=0 ; jj<joint_num ; ++jj ){
                if( joints[jj].get_ope_mode() == OPERATING_MODE_CURRENT ){
                    continue;
                }
                joints[jj].set_command( DXLPOS2RAD( joints[jj].get_home() ) - DXLPOS2RAD( joints[jj].get_center() ) );
            }
            continue;
        }
        for( int jj=0 ; jj<joint_num ; ++jj ){
            if( joints[jj].get_ope_mode() == OPERATING_MODE_CURRENT ){
                continue;
            }
            joints[jj].set_command( joints[jj].get_command() + home_motion_data[jj].step_rad );
        }
        lock_port();
        write( t, d );
        unlock_port();
        rate.sleep();
    }
    for( int jj=0 ; jj<joint_num ; ++jj ){
        if( joints[jj].get_ope_mode() == OPERATING_MODE_CURRENT ){
            continue;
        }
        joints[jj].set_command( home_motion_data[jj].home_rad );
    }
    lock_port();
    write( t, d );
    unlock_port();
    for( int jj=0 ; jj<joint_num ; ++jj ){
        joints[jj].updt_d_command( 0.0 );//差分の初期化
    }
}

/* セルフチェック */
bool DXLPORT_CONTROL::check_servo_param( uint8_t dxl_id, uint32_t test_addr, uint8_t equal, uint8_t& read_val )
{
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint8_t read_data;
    bool result = false;

    lock_port();
    int dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, dxl_id, test_addr, &read_data, &dxl_error);
    unlock_port();
    if( dxl_comm_result != COMM_SUCCESS ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++rx_err;
    }else if( dxl_error != 0 ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
        ++rx_err;
    }
    if( read_data == equal ){
        result = true;
    }
    read_val = read_data;
    return result;
}
bool DXLPORT_CONTROL::check_servo_param( uint8_t dxl_id, uint32_t test_addr, uint16_t equal, uint16_t& read_val )
{
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t read_data;
    bool result = false;

    lock_port();
    int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, test_addr, &read_data, &dxl_error);
    unlock_port();
    if( dxl_comm_result != COMM_SUCCESS ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++rx_err;
    }else if( dxl_error != 0 ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
        ++rx_err;
    }
    if( read_data == equal ){
        result = true;
    }
    read_val = read_data;
    return result;
}
bool DXLPORT_CONTROL::check_servo_param( uint8_t dxl_id, uint32_t test_addr, uint32_t equal, uint32_t& read_val )
{
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint32_t read_data;
    bool result = false;

    lock_port();
    int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, test_addr, &read_data, &dxl_error);
    unlock_port();
    if( dxl_comm_result != COMM_SUCCESS ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++rx_err;
    }else if( dxl_error != 0 ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
        ++rx_err;
    }
    if( read_data == equal ){
        result = true;
    }
    read_val = read_data;
    return result;
}

void DXLPORT_CONTROL::init_joint_params( ST_JOINT_PARAM &param, int table_id, int value )
{
    switch( table_id ){
    case enTableId_ReturnDelay:
        param.return_delay_time = (uint8_t)value;
        break;
    case enTableId_DriveMode:
        param.drive_mode = (uint8_t)value;
        break;
    case enTableId_OpeMode:
        param.operation_mode = (uint8_t)value;
        break;
    case enTableId_HomingOffset:
        param.homing_offset = (int32_t)value;
        break;
    case enTableId_MovingThreshold:
        param.moving_threshold = (uint16_t)value;
        break;
    case enTableId_TempLimit:
        param.temprature_limit = (uint8_t)value;
        break;
    case enTableId_MaxVolLimit:
        param.max_vol_limit = (uint8_t)value;
        break;
    case enTableId_MinVolLimit:
        param.min_vol_limit = (uint8_t)value;
        break;
    case enTableId_CurrentLimit:
        param.current_limit = (uint16_t)value;
        break;
    case enTableId_TorqueEnable:
        param.torque_enable = (uint8_t)value;
        break;
    case enTableId_VelocityIGain:
        param.velocity_i_gain = (uint16_t)value;
        break;
    case enTableId_VelocityPGain:
        param.velocity_p_gain = (uint16_t)value;
        break;
    case enTableId_PositionDGain:
        param.position_d_gain = (uint16_t)value;
        break;
    case enTableId_PositionIGain:
        param.position_i_gain = (uint16_t)value;
        break;
    case enTableId_PositionPGain:
        param.position_p_gain = (uint16_t)value;
        break;
    case enTableId_GoalCurrent:
    case enTableId_GoalVelocity:
    case enTableId_GoalPosition:
    case enTableId_PresentCurrent:
    case enTableId_PresentVelocity:
    case enTableId_PresentPosition:
    case enTableId_PresentTemp:
    case enTableId_Shutdown:
    default:
        break;
    }
}

std::string DXLPORT_CONTROL::self_check( void )
{
    std::string res_str = "[DYNAMIXEL PARAMETER SELF CHECK]\n";

    if( !port_stat ){
        res_str = "SKIP SELF CHECK...";
        return res_str;
    }
    for( int ii=0 ; ii<(sizeof(RegTable)/sizeof(ST_DYNAMIXEL_REG_TABLE)) ; ++ii ){
        bool check_result = true;
        bool read_result;
        uint8_t chk_8data, read_8data;
        uint16_t chk_16data, read_16data;
        uint32_t chk_32data, read_32data;
        if( RegTable[ii].selfcheck ){
            res_str += RegTable[ii].name + " test...\n";
        }
        switch( RegTable[ii].length ){
        case REG_LENGTH_BYTE:
            chk_8data = (uint8_t)RegTable[ii].init_value;
            if( RegTable[ii].name == "OPERATION_MODE" ){
                chk_8data = joints[ii].get_ope_mode();
            }
            for( int jj=0 ; jj<joint_num ; ++jj ){
                read_result = check_servo_param( joints[jj].get_dxl_id(), RegTable[ii].address, chk_8data, read_8data );
                if( RegTable[ii].selfcheck && !read_result ){
                    res_str += " ID: " + std::to_string(joints[jj].get_dxl_id()) + " check NG\n";
                    check_result = false;
                }
                ST_JOINT_PARAM work = joints[jj].get_joint_param();
                init_joint_params( work, ii, read_8data );
                joints[jj].set_joint_param( work );
            }
            break;
        case REG_LENGTH_WORD:
            chk_16data = (uint16_t)RegTable[ii].init_value;
            for( int jj=0 ; jj<joint_num ; ++jj ){
                read_result = check_servo_param( joints[jj].get_dxl_id(), RegTable[ii].address, chk_16data, read_16data );
                if( RegTable[ii].selfcheck && !read_result ){
                    res_str += " ID: " + std::to_string(joints[jj].get_dxl_id()) + " check NG\n";
                    check_result = false;
                }
                ST_JOINT_PARAM work = joints[jj].get_joint_param();
                init_joint_params( work, ii, read_16data );
                joints[jj].set_joint_param( work );
            }
            break;
        case REG_LENGTH_DWORD:
            chk_32data = (uint32_t)RegTable[ii].init_value;
            for( int jj=0 ; jj<joint_num ; ++jj ){
                read_result = check_servo_param( joints[jj].get_dxl_id(), RegTable[ii].address, chk_32data, read_32data );
                if( RegTable[ii].selfcheck && !read_result ){
                    res_str += " ID: " + std::to_string(joints[jj].get_dxl_id()) + " check NG\n";
                    check_result = false;
                }
                ST_JOINT_PARAM work = joints[jj].get_joint_param();
                init_joint_params( work, ii, read_32data );
                joints[jj].set_joint_param( work );
            }
            break;
        }
        if( RegTable[ii].selfcheck ){
            if( check_result ){
                res_str += " CHECK OK\n";
            }else{
                res_str += " CHECK NG!\n";
            }
        }
    }
    return res_str;
}

void DXLPORT_CONTROL::set_param_delay_time( uint8_t dxl_id, int val )
{
    uint8_t set_param = (uint8_t)val;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( new_param.return_delay_time != set_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_RETURN_DELAY, set_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.return_delay_time = set_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_drive_mode( uint8_t dxl_id, int val )
{
    uint8_t set_param = (uint8_t)val;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( new_param.drive_mode != set_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_DRIVE_MODE, set_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.drive_mode = set_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_ope_mode( uint8_t dxl_id, int val )
{
    uint8_t set_param = (uint8_t)val;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( new_param.operation_mode != set_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_OPE_MODE, set_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.operation_mode = set_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_home_offset( uint8_t dxl_id, int val )
{
    uint32_t set_param = (uint32_t)val;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( new_param.homing_offset != set_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write4ByteTxRx( portHandler, dxl_id, ADDR_HOMING_OFFSET, set_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.homing_offset = (int32_t)set_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_moving_threshold( uint8_t dxl_id, int val )
{
    uint32_t set_param = (uint32_t)val;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( new_param.moving_threshold != set_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write4ByteTxRx( portHandler, dxl_id, ADDR_MOVING_THRESHOLD, set_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.moving_threshold = set_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_temp_limit( uint8_t dxl_id, int val )
{
    uint8_t set_param = (uint8_t)val;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( new_param.temprature_limit != set_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_TEMPRATURE_LIMIT, set_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.temprature_limit = set_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_vol_limit( uint8_t dxl_id, int max, int min )
{
    uint16_t set_max_param = (uint32_t)max;
    uint16_t set_min_param = (uint32_t)min;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( new_param.max_vol_limit != set_max_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_MAX_VOL_LIMIT, set_max_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            if( new_param.min_vol_limit != set_min_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_MIN_VOL_LIMIT, set_min_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.max_vol_limit = set_max_param;
            new_param.min_vol_limit = set_min_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_current_limit( uint8_t dxl_id, int val )
{
    uint16_t set_param = (uint16_t)val;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( new_param.current_limit != set_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_CURRENT_LIMIT, set_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.current_limit = set_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}
void DXLPORT_CONTROL::set_param_vel_gain( uint8_t dxl_id, int p, int i )
{
    uint16_t set_p_param = (uint16_t)p;
    uint16_t set_i_param = (uint16_t)i;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( (new_param.velocity_p_gain != set_p_param) ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_VELOCITY_PGAIN, set_p_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            if( (new_param.velocity_i_gain != set_i_param) ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_VELOCITY_IGAIN, set_i_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.velocity_p_gain = set_p_param;
            new_param.velocity_i_gain = set_i_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}

void DXLPORT_CONTROL::set_param_pos_gain_all( int p, int i, int d )
{
    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num ; ++jj ){
        set_param_pos_gain( joints[jj].get_dxl_id(), p, i, d);
    }
}

void DXLPORT_CONTROL::set_param_pos_gain( uint8_t dxl_id, int p, int i, int d )
{
    uint16_t set_p_param = (uint16_t)p;
    uint16_t set_i_param = (uint16_t)i;
    uint16_t set_d_param = (uint16_t)d;

    if( !port_stat ){
        return;
    }
    for( int jj=0 ; jj<joint_num; ++jj ){
        if( dxl_id == joints[jj].get_dxl_id() ){
            ST_JOINT_PARAM new_param = joints[jj].get_joint_param();
            if( new_param.position_p_gain != set_p_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_POSITION_PGAIN, set_p_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            if( new_param.position_i_gain != set_i_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_POSITION_IGAIN, set_i_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            if( new_param.position_d_gain != set_d_param ){
                uint8_t dxl_error = 0; // Dynamixel error
                lock_port();
                int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, ADDR_POSITION_DGAIN, set_d_param, &dxl_error );
                unlock_port();
                if( dxl_comm_result != COMM_SUCCESS ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
                    ++tx_err;
                }else if( dxl_error != 0 ){
                    error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
                    ++tx_err;
                }
            }
            new_param.position_p_gain = set_p_param;
            new_param.position_i_gain = set_i_param;
            new_param.position_d_gain = set_d_param;
            joints[jj].set_joint_param( new_param );
            break;
        }
    }
}

std::string::size_type DXLPORT_CONTROL::get_error( std::string& errorlog )
{
    std::string::size_type result = error_queue.size();
    if( result > 0 ){
        errorlog = error_queue.front();
        error_queue.pop();
    }
    return result;
}

bool DXLPORT_CONTROL::setup_indirect( uint8_t dxl_id )
{
    uint8_t dxl_error = 0; // Dynamixel error
    uint16_t setup_data[] = { 224, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 146 };

    // Write DXL ID data
    int dxl_comm_result = packetHandler->write1ByteTxRx( portHandler, dxl_id, ADDR_INDIRECT_DXLID, dxl_id, &dxl_error );
    if( dxl_comm_result != COMM_SUCCESS ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
        ++tx_err;
    }else if( dxl_error != 0 ){
        error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
        ++tx_err;
    }

    for( int idx=0 ; idx<sizeof(setup_data)/2 ; ++idx ){
        uint32_t set_addr = ADDR_INDIRECT_TOP + (idx*2);
        int dxl_comm_result = packetHandler->write2ByteTxRx( portHandler, dxl_id, set_addr, setup_data[idx], &dxl_error );
        if( dxl_comm_result != COMM_SUCCESS ){
            error_queue.push( (std::string(__func__) + " ") + packetHandler->getTxRxResult( dxl_comm_result ) );
            ++tx_err;
        }else if( dxl_error != 0 ){
            error_queue.push( (std::string(__func__) + " ") + packetHandler->getRxPacketError( dxl_error ) );
            ++tx_err;
        }
    }

    return true;
}
