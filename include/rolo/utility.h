#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE 

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

// #include <opencv/cv.h>


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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <flann/flann.hpp>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include "rolo/CloudInfoStamp.h"
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
using namespace std;

typedef pcl::PointXYZI PointType;

enum class lidarType { VELODYNE, OUSTER };

class ParamLoader
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic; // 输入的激光
    string odomTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    lidarType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;
    float lidarNoiseBound;
    bool deskewEnabled;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Scan Registration
    float CT_lambda;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag; // 回环检测使能位
    float loopClosureFrequency; // 回环检测频率
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;
    // 载入param参数
    ParamLoader()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("rolo/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("rolo/odomTopic", odomTopic, "odometry/imu");

        nh.param<std::string>("rolo/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("rolo/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("rolo/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("rolo/mapFrame", mapFrame, "map");

        nh.param<bool>("rolo/savePCD", savePCD, false);
        nh.param<std::string>("rolo/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        std::string sensorStr;
        nh.param<std::string>("rolo/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = lidarType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = lidarType::OUSTER;
        }
        // else if (sensorStr == "livox")
        // {
        //     sensor = lidarType::LIVOX;
        // }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("rolo/N_SCAN", N_SCAN, 16);
        nh.param<int>("rolo/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("rolo/downsampleRate", downsampleRate, 1);
        nh.param<float>("rolo/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("rolo/lidarMaxRange", lidarMaxRange, 1000.0);
        nh.param<float>("rolo/lidarNoiseBound", lidarNoiseBound, 0.05);
        nh.param<bool>("rolo/deskewEnabled", deskewEnabled, true);

        nh.param<float>("rolo/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("rolo/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("rolo/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("rolo/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("rolo/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("rolo/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("rolo/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("rolo/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("rolo/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("rolo/numberOfCores", numberOfCores, 2);
        nh.param<double>("rolo/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("rolo/continuousTrajectoryWeight", CT_lambda, 1.0);
        
        nh.param<float>("rolo/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("rolo/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("rolo/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("rolo/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("rolo/loopClosureEnableFlag", loopClosureEnableFlag, true);
        nh.param<float>("rolo/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("rolo/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("rolo/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("rolo/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("rolo/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("rolo/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("rolo/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("rolo/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("rolo/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }
};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

template<typename T>
double GET_ROS_TIMESTAMP(T msg)
{
    return msg->header.stamp.toSec();
}

double radTodeg(double radians)
{
  return radians * 180.0 / M_PI;
}

double degTorad(double degrees)
{
  return degrees * M_PI / 180.0;
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

/**
 * Remove one row from matrix.
 * Credit to: https://stackoverflow.com/questions/13290395
 * @param matrix an Eigen::Matrix.
 * @param rowToRemove index of row to remove. If >= matrix.rows(), no operation will be taken
 */
template <class T, int R, int C>
void removeRow(Eigen::Matrix<T, R, C>& matrix, unsigned int rowToRemove) {
  if (rowToRemove >= matrix.rows()) {
    return;
  }
  unsigned int numRows = matrix.rows() - 1;
  unsigned int numCols = matrix.cols();

  if (rowToRemove < numRows) {
    matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) =
        matrix.bottomRows(numRows - rowToRemove);
  }

  matrix.conservativeResize(numRows, numCols);
}

/**
 * Remove one column from matrix.
 * Credit to: https://stackoverflow.com/questions/13290395
 * @param matrix
 * @param colToRemove index of col to remove. If >= matrix.cols(), no operation will be taken
 */
template <class T, int R, int C>
void removeColumn(Eigen::Matrix<T, R, C>& matrix, unsigned int colToRemove) {
  if (colToRemove >= matrix.cols()) {
    return;
  }
  unsigned int numRows = matrix.rows();
  unsigned int numCols = matrix.cols() - 1;

  if (colToRemove < numCols) {
    matrix.block(0, colToRemove, numRows, numCols - colToRemove) =
        matrix.rightCols(numCols - colToRemove);
  }

  matrix.conservativeResize(numRows, numCols);
}

#endif
