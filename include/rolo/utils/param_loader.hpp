#pragma once
#ifndef ROLO_UTILS_PARAM_LOADER_HPP_
#define ROLO_UTILS_PARAM_LOADER_HPP_

#include "rolo/utils/point_type.hpp"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <unistd.h>

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

    // Topics
    std::string pointCloudTopic;
    std::string odomTopic;

    // Frames
    std::string lidarFrame;
    std::string baselinkFrame;
    std::string odometryFrame;
    std::string mapFrame;
    std::vector<double> initPose;

    // Save pcd
    bool savePCD;
    std::string savePCDDirectory;

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

    // Voxel filter params
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize;

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
    bool loopClosureEnableFlag;
    std::string loopCloseType;
    std::string scInputType;
    float loopClosureFrequency;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;
    bool priorFactorEnableFlag;
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

    // Global map visualization
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamLoader(bool requireSensorConfig = true)
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("rolo/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("rolo/odomTopic", odomTopic, "odometry/imu");

        nh.param<std::string>("rolo/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("rolo/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("rolo/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("rolo/mapFrame", mapFrame, "map");
        nh.param<std::vector<double>>("rolo/initPose", initPose, std::vector<double>());
        for (std::size_t i = 3; i < initPose.size(); ++i)
            initPose[i] = initPose[i] * (M_PI / 180.0);

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
        else
        {
            if (requireSensorConfig)
            {
                ROS_ERROR_STREAM("Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
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
        nh.param<bool>("rolo/deskewEnabled", deskewEnabled, false);

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

#endif  // ROLO_UTILS_PARAM_LOADER_HPP_
