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

#include <opencv/cv.h>
#include <eigen3/Eigen/Dense>

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
#include "rolo_sam/CloudInfoStamp.h"

using namespace std;

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFHFeature;
typedef std::vector<Eigen::VectorXf> Feature;
typedef flann::Index<flann::L2<float>> FLANNKDTree;

enum class lidarType { VELODYNE, OUSTER };

class ParamLoader
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic; // 输入的激光
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    lidarType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int Ground_Scan_Ring;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;
    float lidarNoiseBound;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;
    float groundDegreeThre;
    float nearestSearchRadius;

    // Rotation Optimization
    float inlierDiffBound;
    float costFactor;
    float costDiffThre;
    int maxOptIteration;
    int minOptConstrains;
    int maxOptContrains;

    // Translation Optimization
    vector<double> KScalar;
    Eigen::Matrix3d KScalarMat;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

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

        nh.param<std::string>("rolo_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("rolo_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("rolo_sam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("rolo_sam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("rolo_sam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("rolo_sam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("rolo_sam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("rolo_sam/mapFrame", mapFrame, "map");

        nh.param<bool>("rolo_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("rolo_sam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("rolo_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("rolo_sam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("rolo_sam/savePCD", savePCD, false);
        nh.param<std::string>("rolo_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        std::string sensorStr;
        nh.param<std::string>("rolo_sam/sensor", sensorStr, "");
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

        nh.param<int>("rolo_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("rolo_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("rolo_sam/Ground_Scan_Ring", Ground_Scan_Ring, 7);
        nh.param<int>("rolo_sam/downsampleRate", downsampleRate, 1);
        nh.param<float>("rolo_sam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("rolo_sam/lidarMaxRange", lidarMaxRange, 1000.0);
        nh.param<float>("rolo_sam/lidarNoiseBound", lidarNoiseBound, 0.05);

        nh.param<float>("rolo_sam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("rolo_sam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("rolo_sam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("rolo_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("rolo_sam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("rolo_sam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("rolo_sam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("rolo_sam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("rolo_sam/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        nh.param<float>("rolo_sam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("rolo_sam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("rolo_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("rolo_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);
        nh.param<float>("rolo_sam/groundDegreeThre", groundDegreeThre, 10);
        groundDegreeThre = groundDegreeThre * M_PI/180.0;   // rad form
        nh.param<float>("rolo_sam/nearestSearchRadius", nearestSearchRadius, 1.0);

        nh.param<float>("rolo_sam/inlierDiffBound", inlierDiffBound, 0.1);
        nh.param<float>("rolo_sam/costFactor", costFactor, 1.2);
        if (costFactor <= 1.0){
            ROS_ERROR_STREAM(
                "Invalid costFactor! costFactor must greater than 1");
            ros::shutdown();
        }
        nh.param<float>("rolo_sam/costDiffThre", costDiffThre, 0.001);
        nh.param<int>("rolo_sam/maxOptIteration", maxOptIteration, 100);
        nh.param<int>("rolo_sam/minOptConstrains", minOptConstrains, 50);
        nh.param<int>("rolo_sam/maxOptContrains", maxOptContrains, 100);

        nh.param<vector<double>>("rolo_sam/KScalar", KScalar, vector<double>());
        KScalarMat = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(KScalar.data(), 3, 3);

        nh.param<float>("rolo_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("rolo_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("rolo_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("rolo_sam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("rolo_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("rolo_sam/numberOfCores", numberOfCores, 2);
        nh.param<double>("rolo_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("rolo_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("rolo_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("rolo_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("rolo_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("rolo_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("rolo_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("rolo_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("rolo_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("rolo_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("rolo_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("rolo_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("rolo_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("rolo_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("rolo_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

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
