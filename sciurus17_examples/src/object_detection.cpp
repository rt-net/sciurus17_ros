// Copyright 2019 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

static ros::Publisher _pub_point_cloud;
static ros::Publisher _pub_marker_array;


void convert_to_marker(visualization_msgs::Marker *marker, const int marker_id,
        const std::string &frame_id,
        const pcl::PointXYZRGB &min_pt, const pcl::PointXYZRGB &max_pt)
{
    // pcl::Pointの最大最小値をBox型のマーカに変換する

    marker->header.frame_id = frame_id;
    marker->header.stamp = ros::Time();
    marker->ns = "/sciurus17/example";
    marker->id = marker_id;
    marker->type = visualization_msgs::Marker::CUBE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->lifetime = ros::Duration(0.5);

    marker->pose.position.x = (max_pt.x + min_pt.x) * 0.5;
    marker->pose.position.y = (max_pt.y + min_pt.y) * 0.5;
    marker->pose.position.z = (max_pt.z + min_pt.z) * 0.5;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;
    marker->scale.x = (max_pt.x - min_pt.x);
    marker->scale.y = (max_pt.y - min_pt.y);
    marker->scale.z = (max_pt.z - min_pt.z);
    marker->color.a = 1.0;
    marker->color.r = 1.0;
    marker->color.g = 1.0;
    marker->color.b = 1.0;
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    const static std::string FRAME_ID = "base_link";
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
    enum COLOR_RGB{
        RED=0,
        GREEN,
        BLUE,
        COLOR_MAX
    };
    const int CLUSTER_MAX = 10;
    const int CLUSTER_COLOR[CLUSTER_MAX][COLOR_MAX] = {
        {230, 0, 18},{243, 152, 18}, {255, 251, 0},
        {143, 195, 31},{0, 153, 68}, {0, 158, 150},
        {0, 160, 233},{0, 104, 183}, {29, 32, 136},
        {146, 7, 131}
    };

    sensor_msgs::PointCloud2 cloud_transformed;

    // point cloudの座標を変換
    // base_linkとカメラ間のTFを取得する
    while(true){
        try{
            listener.lookupTransform(FRAME_ID, cloud_msg->header.frame_id, ros::Time(0), transform);
            break;
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    pcl_ros::transformPointCloud(FRAME_ID, transform, *cloud_msg, cloud_transformed);

    // sensor_msgs/PointCloud2からpcl/PointCloudに変換
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::fromROSMsg(cloud_transformed, *cloud);

    // Z方向の範囲でフィルタリング
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.02, 1.0);
    pass.filter (*cloud_passthrough);

    // voxelgridでダウンサンプリング
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxelgrid (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid;
    voxelgrid.setInputCloud (cloud_passthrough);
    voxelgrid.setLeafSize (0.01, 0.01, 0.01);
    voxelgrid.filter (*cloud_voxelgrid);

    // pointがなければ処理を抜ける
    if(cloud_voxelgrid->size() <= 0){
        return;
    }

    // クラスタ抽出のためKdTreeオブジェクトを作成
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud (cloud_voxelgrid);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02);
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (250);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_voxelgrid);
    ec.extract (cluster_indices);

    int cluster_i=0;
    visualization_msgs::MarkerArray markers;

    // クラスタごとにPointのRGB値を変更する
    // クラスタをもとにMarkerを生成する
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
            it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>());
        // クラスタ内のPointRGB値を変更
        for (std::vector<int>::const_iterator pit = it->indices.begin (); 
                pit != it->indices.end (); ++pit){
            cloud_voxelgrid->points[*pit].r = CLUSTER_COLOR[cluster_i][RED];
            cloud_voxelgrid->points[*pit].g = CLUSTER_COLOR[cluster_i][GREEN];
            cloud_voxelgrid->points[*pit].b = CLUSTER_COLOR[cluster_i][BLUE];
            cloud_cluster->points.push_back (cloud_voxelgrid->points[*pit]); //*
        }
        // Unorganized datasetsとしてwidth, heightを入力
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        // 無効なpointがないのでis_denseはtrue
        cloud_cluster->is_dense = true;
        *cloud_output += *cloud_cluster;

        // Markerの作成
        visualization_msgs::Marker marker;
        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        convert_to_marker(&marker, cluster_i, FRAME_ID, min_pt, max_pt);
        markers.markers.push_back(marker);

        cluster_i++;
        if(cluster_i >= CLUSTER_MAX){
            break;
        }
    }

    // pcl/PointCloudからsensor_msgs/PointCloud2に変換
    sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*cloud, output);
    // pcl::toROSMsg(*cloud_passthrough, output);
    // pcl::toROSMsg(*cloud_voxelgrid, output);
    pcl::toROSMsg(*cloud_output, output);
    output.header.frame_id = FRAME_ID;

    _pub_point_cloud.publish(output);
    _pub_marker_array.publish(markers);
}


int main (int argc, char** argv)
{
    ros::init (argc, argv, "object_detection");
    ros::NodeHandle nh("~");

    _pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/sciurus17/example/points", 1);
    _pub_marker_array = nh.advertise<visualization_msgs::MarkerArray>("/sciurus17/example/markers", 1);

    ros::Subscriber sub = nh.subscribe("/sciurus17/camera/depth_registered/points", 1, cloud_cb);

    ros::spin ();
}

