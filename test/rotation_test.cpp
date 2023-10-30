#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
typedef pcl::PointXYZI  PointType;
string src_path = "/home/sdu/pcd_data/scen2/origin.pcd";
string dst_path = "/home/sdu/pcd_data/translated_cloud.pcd";
ros::Publisher *point_pub_;

bool src_handler(std_srvs::Empty::Request & req,
                    std_srvs::Empty::Response &res){
    pcl::PointCloud<PointType>::Ptr SourceCloud(new pcl::PointCloud<PointType>); // 源点云
    pcl::PointCloud<PointType>::Ptr SCloud(new pcl::PointCloud<PointType>); // 源点云
    pcl::io::loadPCDFile(src_path, *SourceCloud);
    sensor_msgs::PointCloud2 pt_cloud;
    pcl::toROSMsg(*SourceCloud, pt_cloud);
    std::vector<int> temp;
    pcl::removeNaNFromPointCloud(*SourceCloud, temp);
    pt_cloud.header.stamp = ros::Time::now();
    pt_cloud.header.frame_id = "velodyne";
    point_pub_->publish(pt_cloud);
}

bool dst_handler(std_srvs::Empty::Request & req,
                    std_srvs::Empty::Response &res){
    pcl::PointCloud<PointType>::Ptr TargetCloud(new pcl::PointCloud<PointType>); // 目标点云   
    pcl::PointCloud<PointType>::Ptr TCloud(new pcl::PointCloud<PointType>); // 目标点云
    pcl::io::loadPCDFile(dst_path, *TargetCloud);
    sensor_msgs::PointCloud2 pt_cloud;
    pcl::toROSMsg(*TargetCloud, pt_cloud);
    std::vector<int> temp;
    pcl::removeNaNFromPointCloud(*TargetCloud, temp);
    pt_cloud.header.stamp = ros::Time::now();
    pt_cloud.header.frame_id = "velodyne";
    point_pub_->publish(pt_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rolo_sam");
    ros::NodeHandle nh;
    ros::ServiceServer src_server = nh.advertiseService("pub_src", src_handler);
    ros::ServiceServer dst_server = nh.advertiseService("pub_dst", dst_handler);
    
    ros::Publisher point_pub = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
    point_pub_ = &point_pub;
    ROS_INFO("Service established!");
    
    ros::spin();
    
    return 0;
}