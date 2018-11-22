#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <string>
#include <unistd.h>


/* メイン処理 */
int main(int argc, char **argv)
{
    rs2::context ctx;

    for( auto&& dev : ctx.query_devices() ){
        // 取得したすべてのデバイスにリセットを発行する
        dev.hardware_reset();
        sleep( 1 );// 再接続待ち
    }
}
