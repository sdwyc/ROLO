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
#include <jsk_recognition_msgs/BoundingBoxArray.h>
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
#include <cctype>
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

struct EIGEN_ALIGN16 GroundPatchType
{
    PCL_ADD_POINT4D;
    float intensity;
    float timestamp;
    uint8_t label;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    inline GroundPatchType()
    {
      x = y = z = timestamp = 0.0f;
      intensity = 0.0f;
      label = 0;
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(GroundPatchType,
	(float,x,x)
	(float,y,y)
	(float,z,z)
	(float,intensity,intensity)
	(float,timestamp,timestamp)
	(uint8_t,label,label)
)

inline Eigen::Vector3d LoadVector3Param(ros::NodeHandle &nh, const std::string &name,
                                        const Eigen::Vector3d &fallback)
{
    std::vector<double> raw;
    nh.param<std::vector<double>>(name, raw, std::vector<double>());
    if (raw.size() != 3)
        return fallback;
    return Eigen::Map<const Eigen::Matrix<double, 3, 1>>(raw.data());
}

inline Eigen::Matrix3d LoadMatrix3Param(ros::NodeHandle &nh, const std::string &name,
                                        const Eigen::Matrix3d &fallback)
{
    std::vector<double> raw;
    nh.param<std::vector<double>>(name, raw, std::vector<double>());
    if (raw.size() != 9)
        return fallback;
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(raw.data());
}

inline std::vector<Eigen::Vector2d> LoadVector2ArrayParam(ros::NodeHandle &nh, const std::string &name)
{
    std::vector<double> raw;
    nh.param<std::vector<double>>(name, raw, std::vector<double>());
    std::vector<Eigen::Vector2d> values;
    if (raw.empty() || raw.size() % 2 != 0)
        return values;

    values.reserve(raw.size() / 2);
    for (size_t i = 0; i < raw.size(); i += 2)
        values.emplace_back(raw[i], raw[i + 1]);
    return values;
}

inline std::vector<Eigen::Vector3d> LoadVector3ArrayParam(ros::NodeHandle &nh, const std::string &name)
{
    std::vector<double> raw;
    nh.param<std::vector<double>>(name, raw, std::vector<double>());
    std::vector<Eigen::Vector3d> values;
    if (raw.empty() || raw.size() % 3 != 0)
        return values;

    values.reserve(raw.size() / 3);
    for (size_t i = 0; i < raw.size(); i += 3)
        values.emplace_back(raw[i], raw[i + 1], raw[i + 2]);
    return values;
}

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
    std::string loopCloseType;
    std::string scInputType;
    float loopClosureFrequency; // 回环检测频率
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;
    bool  priorFactorEnableFlag;
    float priorFactorFrequency;
    float groundPatchSize;
    float nearPriorRadius;
    float priorFitnessScore;
    float priorTimeValidation;
    float priorRangeValidation;
    float priorRotDiffTolerance;
    float priorTransDiffTolerance;
    float priorFactorWeight;
    float priorSyncedInterval;

    // Prior pose node
    std::string priorPoseNodePcdTopic;
    std::string priorPoseNodePoseTopic;
    std::string priorPoseNodePoseCovTopic;
    std::string priorPoseNodeFrameId;
    std::string priorPoseNodeChildFrameId;
    std::string priorPoseNodeMeshResource;
    double priorPoseNodeMarkerScale;
    double priorVehicleSizeXY;
    double priorVehicleComZ;
    double priorKSpring;
    double priorGravity;
    int priorMaxIters;
    double priorLmLambda;
    double priorTolCost;
    double priorTolStep;
    double priorGroundAvgRadius;
    int priorGroundMinNeighbors;
    double priorToleranceZMin;
    double priorToleranceZMax;
    double priorToleranceRoll;
    double priorTolerancePitch;
    double priorToleranceWheelDistance;
    bool priorPublishTF;
    bool priorPublishModelMarker;
    double priorModelMarkerWidth;
    bool priorVerbose;
    Eigen::Vector3d priorLidarOffsetTrans;
    Eigen::Matrix3d priorLidarOffsetRot;
    Eigen::Vector3d priorMeshOffset;
    Eigen::Vector3d priorMeshRPY;
    std::vector<Eigen::Vector2d> priorWheelXY;
    std::vector<Eigen::Vector3d> priorMeshWheelPoints;
    double priorVehicleSizeX;
    double priorVehicleSizeY;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;
    // 载入param参数
    ParamLoader(bool requireSensorConfig = true)
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
        nh.param<std::string>("rolo/sensor", sensorStr, requireSensorConfig ? "" : "velodyne");
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
            if (requireSensorConfig)
            {
                ROS_ERROR_STREAM(
                    "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
                ros::shutdown();
            }
            else
            {
                sensor = lidarType::VELODYNE;
            }
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
        nh.param<std::string>("rolo/loopCloseType", loopCloseType, "sc");
        std::transform(loopCloseType.begin(), loopCloseType.end(), loopCloseType.begin(), ::tolower);
        if (loopCloseType != "sc" && loopCloseType != "rs")
        {
            ROS_WARN_STREAM("Invalid rolo/loopCloseType '" << loopCloseType << "', fallback to 'sc'.");
            loopCloseType = "sc";
        }
        nh.param<std::string>("rolo/scInputType", scInputType, "scan_raw");
        std::transform(scInputType.begin(), scInputType.end(), scInputType.begin(), ::tolower);
        if (scInputType == "raw")
            scInputType = "scan_raw";
        else if (scInputType == "feat")
            scInputType = "scan_feat";
        if (scInputType != "scan_raw" && scInputType != "scan_feat")
        {
            ROS_WARN_STREAM("Invalid rolo/scInputType '" << scInputType << "', fallback to 'scan_raw'.");
            scInputType = "scan_raw";
        }
        nh.param<float>("rolo/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("rolo/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("rolo/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("rolo/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("rolo/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("rolo/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);
        nh.param<bool>("prior_factor/priorFactorEnableFlag", priorFactorEnableFlag, true);
        nh.param<float>("prior_factor/priorFactorFrequency", priorFactorFrequency, 1.0);
        nh.param<float>("prior_factor/groundPatchSize", groundPatchSize, 2.0);
        nh.param<float>("prior_factor/nearPriorRadius", nearPriorRadius, 1.0);
        nh.param<float>("prior_factor/priorFitnessScore", priorFitnessScore, 0.01);
        nh.param<float>("prior_factor/priorTimeValidation", priorTimeValidation, 1.0);
        nh.param<float>("prior_factor/priorRangeValidation", priorRangeValidation, 10.0);
        nh.param<float>("prior_factor/priorRotDiffTolerance", priorRotDiffTolerance, 5.0f);
        priorRotDiffTolerance = priorRotDiffTolerance * M_PI / 180.0f;
        nh.param<float>("prior_factor/priorTransDiffTolerance", priorTransDiffTolerance, 1.0);
        nh.param<float>("prior_factor/priorFactorWeight", priorFactorWeight, 100.0);
        nh.param<float>("prior_factor/priorSyncedInterval", priorSyncedInterval, 0.0f);

        nh.param<std::string>("prior_pose_node/pcd_topic", priorPoseNodePcdTopic, "/voxel_map");
        nh.param<std::string>("prior_pose_node/pose_topic", priorPoseNodePoseTopic, "/predicted_pose");
        nh.param<std::string>("prior_pose_node/pose_cov_topic", priorPoseNodePoseCovTopic, "/initialpose");
        nh.param<std::string>("prior_pose_node/frame_id", priorPoseNodeFrameId, "map");
        nh.param<std::string>("prior_pose_node/child_frame_id", priorPoseNodeChildFrameId, "vehicle");
        nh.param<std::string>("prior_pose_node/mesh_resource", priorPoseNodeMeshResource, "package://rolo/resource/meshes/vehicle.dae");
        nh.param<double>("prior_pose_node/marker_scale", priorPoseNodeMarkerScale, 1.0);
        nh.param<double>("prior_pose_node/vehicle_size_xy", priorVehicleSizeXY, 2.0);
        nh.param<double>("prior_pose_node/vehicle_com_z", priorVehicleComZ, 1.0);
        nh.param<double>("prior_pose_node/k_spring", priorKSpring, 20.0);
        nh.param<double>("prior_pose_node/g", priorGravity, 1.0);
        nh.param<int>("prior_pose_node/max_iters", priorMaxIters, 60);
        nh.param<double>("prior_pose_node/lm_lambda", priorLmLambda, 1e-2);
        nh.param<double>("prior_pose_node/tol_cost", priorTolCost, 1e-12);
        nh.param<double>("prior_pose_node/tol_step", priorTolStep, 1e-10);
        nh.param<double>("prior_pose_node/ground_avg_radius", priorGroundAvgRadius, 0.3);
        nh.param<int>("prior_pose_node/ground_min_neighbors", priorGroundMinNeighbors, 5);
        nh.param<double>("prior_pose_node/tolerance_z_min", priorToleranceZMin, -10.0);
        nh.param<double>("prior_pose_node/tolerance_z_max", priorToleranceZMax, 10.0);
        nh.param<double>("prior_pose_node/tolerance_roll", priorToleranceRoll, 1.0);
        nh.param<double>("prior_pose_node/tolerance_pitch", priorTolerancePitch, 1.0);
        nh.param<double>("prior_pose_node/tolerance_wheel_distance", priorToleranceWheelDistance, 1.0);
        nh.param<bool>("prior_pose_node/publish_tf", priorPublishTF, true);
        nh.param<bool>("prior_pose_node/publish_model_marker", priorPublishModelMarker, true);
        nh.param<double>("prior_pose_node/model_marker_width", priorModelMarkerWidth, 0.05);
        nh.param<bool>("prior_pose_node/verbose", priorVerbose, false);

        priorWheelXY = LoadVector2ArrayParam(nh, "prior_pose_node/wheel_xy");
        priorMeshWheelPoints = LoadVector3ArrayParam(nh, "prior_pose_node/mesh_wheel_points");
        priorLidarOffsetTrans = LoadVector3Param(nh, "prior_pose_node/lidarOffsetTrans", Eigen::Vector3d::Zero());
        priorLidarOffsetRot = LoadMatrix3Param(nh, "prior_pose_node/lidarOffsetRot", Eigen::Matrix3d::Identity());
        priorMeshOffset = LoadVector3Param(nh, "prior_pose_node/mesh_offset", Eigen::Vector3d::Zero());
        priorMeshRPY = LoadVector3Param(nh, "prior_pose_node/mesh_rpy", Eigen::Vector3d::Zero());

        priorVehicleSizeX = std::max(priorVehicleSizeXY, 0.1);
        priorVehicleSizeY = std::max(priorVehicleSizeXY, 0.1);
        if (!priorWheelXY.empty())
        {
            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();
            for (const auto &xy : priorWheelXY)
            {
                min_x = std::min(min_x, xy.x());
                max_x = std::max(max_x, xy.x());
                min_y = std::min(min_y, xy.y());
                max_y = std::max(max_y, xy.y());
            }
            priorVehicleSizeX = std::max(max_x - min_x, 0.1);
            priorVehicleSizeY = std::max(max_y - min_y, 0.1);
        }

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

inline double radTodeg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double degTorad(double degrees)
{
  return degrees * M_PI / 180.0;
}

inline float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


inline float pointDistance(PointType p1, PointType p2)
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
