#include "utility.hpp"
#include "rolo/eskf/eskf.hpp"
#include "autoware_rviz_msgs/Path.h"
#include "autoware_rviz_msgs/PathPoint.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "rolo/registration.hpp"

using namespace Eigen;

class TransformFusion : public ParamLoader
{
public:
    std::mutex mtx;

    ros::Subscriber subLidarOdometry;
    ros::Subscriber subMappingOdometry;

    ros::Publisher pubLidarOdometry;
    ros::Publisher pubLidarPath;
    ros::Publisher pubLidarSpeed;
    ros::Publisher pubFuturePath;
    ros::Publisher pubFuturePoseLidar;
    ros::Timer fusionTimer;
    ros::Timer predictTimer;

    Eigen::Affine3f mappingOdomAffine;
    Eigen::Affine3f lidarOdomAffineFront;
    Eigen::Affine3f lidarOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;
    tf::TransformBroadcaster tfMap2Odom;
    tf::TransformBroadcaster tfOdom2BaseLink;

    double mappingOdomTime = -1;
    double lastProcessedLidarTime = -1;
    double lastPathTime = -1;
    bool hasLidarOdomAffineFront = false;
    nav_msgs::Odometry latestLidarOdomTemplate;
    nav_msgs::Path lidarPath;
    deque<nav_msgs::Odometry> lidarOdomQueue;
    rolo::eskf::PoseESEKF pose_regulator;
    //! Read base-lidar TF and set I/O
    TransformFusion()
    {
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (const tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }
        // Subscribe to backend and preintegrated odometry
        subMappingOdometry = nh.subscribe<nav_msgs::Odometry>("rolo/mapping/odometry", 5, &TransformFusion::mappingOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLidarOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::lidarOdometryHandler,   this, ros::TransportHints().tcpNoDelay());
        // Publish fused odometry
        pubLidarOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubLidarPath   = nh.advertise<nav_msgs::Path>("rolo/lidar_odometry/path", 1);
        pubLidarSpeed      = nh.advertise<std_msgs::Float32>(odomTopic + "/speed", 2000);
        pubFuturePath      = nh.advertise<autoware_rviz_msgs::Path>("future_path", 1);
        pubFuturePoseLidar = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("future_pose_lidar", 1);
        fusionTimer        = nh.createTimer(ros::Duration(1.0 / 20.0), &TransformFusion::fusionTimerHandler, this);
        predictTimer       = nh.createTimer(ros::Duration(1.0 / 30.0), &TransformFusion::predictTimerHandler, this);
    }

    //! Store lidar odometry transform
    void mappingOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        mappingOdomAffine = odom2affine(*odomMsg);

        mappingOdomTime = odomMsg->header.stamp.toSec();
        hasLidarOdomAffineFront = false;
    }
    //! Fuse front-end odometry with backend odometry
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomQueue.push_back(*odomMsg);
        latestLidarOdomTemplate = *odomMsg;
    }

    void fusionTimerHandler(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(mtx);

        ros::Time stamp = ros::Time::now();
        tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, stamp, mapFrame, odometryFrame));

        if(mappingOdomTime == -1)
            return;

        while(!lidarOdomQueue.empty() && lidarOdomQueue.front().header.stamp.toSec() <= mappingOdomTime)
            lidarOdomQueue.pop_front();

        // if(!hasLidarOdomAffineFront)
        // {
        if(lidarOdomQueue.empty())
            return;
        lidarOdomAffineFront = odom2affine(lidarOdomQueue.front());
        // hasLidarOdomAffineFront = true;
        // }

        bool has_new_lidar_odom = false;
        if(!lidarOdomQueue.empty() && lidarOdomQueue.back().header.stamp.toSec() > lastProcessedLidarTime)
            has_new_lidar_odom = true;

        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

        if(has_new_lidar_odom)
        {
            nav_msgs::Odometry lidar_odom = lidarOdomQueue.back();
            Eigen::Affine3f lidar_odom_affine = odom2affine(lidar_odom);
            Eigen::Vector3d measurement_position;
            Eigen::Quaterniond measurement_orientation;
            affineToPose(lidar_odom_affine, measurement_position, measurement_orientation);
            double lidar_time = lidar_odom.header.stamp.toSec();
            if(pose_regulator.processMeasurement(lidar_time, measurement_position, measurement_orientation))
            {
                lastProcessedLidarTime = lidar_time;
                // lidarOdomQueue.clear();
            }
        }

        if(!pose_regulator.initialized())
            return;

        rolo::eskf::PoseESEKF pose_preview = pose_regulator;
        // if(!has_new_lidar_odom)
        pose_preview.statePredict(stamp.toSec());

        position = pose_preview.position();
        orientation = pose_preview.orientation();
        velocity = pose_preview.velocity();

        Eigen::Affine3f lidarOdomAffineBack = Eigen::Affine3f::Identity();
        lidarOdomAffineBack.translation() = position.cast<float>();
        lidarOdomAffineBack.linear() = orientation.toRotationMatrix().cast<float>();
        Eigen::Affine3f lidarOdomAffineIncre = lidarOdomAffineFront.inverse() * lidarOdomAffineBack;
        Eigen::Affine3f lidarOdomAffineLast = mappingOdomAffine * lidarOdomAffineIncre;
        affineToPose(lidarOdomAffineLast, position, orientation);
        // velocity = mappingOdomAffine.rotation().cast<double>() * velocity;

        nav_msgs::Odometry laserOdometry = latestLidarOdomTemplate;
        laserOdometry.header.stamp = stamp;
        laserOdometry.header.frame_id = odometryFrame;
        laserOdometry.child_frame_id = baselinkFrame;
        laserOdometry.pose.pose.position.x = position.x();
        laserOdometry.pose.pose.position.y = position.y();
        laserOdometry.pose.pose.position.z = position.z();
        laserOdometry.pose.pose.orientation.x = orientation.x();
        laserOdometry.pose.pose.orientation.y = orientation.y();
        laserOdometry.pose.pose.orientation.z = orientation.z();
        laserOdometry.pose.pose.orientation.w = orientation.w();
        laserOdometry.twist.twist.linear.x = velocity.x();
        laserOdometry.twist.twist.linear.y = velocity.y();
        laserOdometry.twist.twist.linear.z = velocity.z();
        pubLidarOdometry.publish(laserOdometry);

        // publish tf
        // Publish odometry and TF
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish Lidar odometry path
        double lidarTime = stamp.toSec();
        if (lidarTime - lastPathTime > 0.05)
        {
            lastPathTime = lidarTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            lidarPath.poses.push_back(pose_stamped);
            // Keep only 1s of front-end path
            while(!lidarPath.poses.empty() && lidarPath.poses.front().header.stamp.toSec() < lidarTime - 1.0)
                lidarPath.poses.erase(lidarPath.poses.begin());
            lidarPath.header.stamp = stamp;
            lidarPath.header.frame_id = odometryFrame;
            pubLidarPath.publish(lidarPath);
        }

        std_msgs::Float32 speed_msg;
        speed_msg.data = velocity.norm();
        pubLidarSpeed.publish(speed_msg);
    }

    void predictTimerHandler(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if(!pose_regulator.initialized())
            return;

        rolo::eskf::PoseESEKF::PoseVectorList futurePoseList = pose_regulator.planarPropagate(0.2, 6.0);
        if(futurePoseList.empty())
            return;

        ros::Time stamp = ros::Time::now();

        Eigen::Vector3d current_position = pose_regulator.position();
        Eigen::Quaterniond current_orientation = pose_regulator.orientation();
        Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
        current_pose.translation() = current_position;
        current_pose.linear() = current_orientation.toRotationMatrix();

        Eigen::Vector3d local_velocity = current_pose.linear().transpose() * pose_regulator.velocity();
        double heading_rate = pose_regulator.angularVelocity().z();

        autoware_rviz_msgs::Path future_path;
        future_path.header.stamp = stamp;
        future_path.header.frame_id = lidarFrame;
        future_path.points.reserve(futurePoseList.size());

        geometry_msgs::PoseWithCovarianceStamped future_pose_lidar;
        future_pose_lidar.header.stamp = stamp;
        future_pose_lidar.header.frame_id = lidarFrame;

        for(size_t i = 0; i < futurePoseList.size(); ++i)
        {
            Eigen::Affine3d future_pose = Eigen::Affine3d::Identity();
            Eigen::Quaterniond future_orientation;
            future_orientation.x() = futurePoseList[i](3);
            future_orientation.y() = futurePoseList[i](4);
            future_orientation.z() = futurePoseList[i](5);
            future_orientation.w() = futurePoseList[i](6);
            future_orientation.normalize();
            future_pose.translation() = futurePoseList[i].head<3>();
            future_pose.linear() = future_orientation.toRotationMatrix();

            Eigen::Affine3d local_pose = current_pose.inverse() * future_pose;
            Eigen::Quaterniond local_orientation(local_pose.linear());
            local_orientation.normalize();

            autoware_rviz_msgs::PathPoint path_point;
            path_point.pose.position.x = local_pose.translation().x();
            path_point.pose.position.y = local_pose.translation().y();
            path_point.pose.position.z = 0.0; // local_pose.translation().z();
            path_point.pose.orientation.x = local_orientation.x();
            path_point.pose.orientation.y = local_orientation.y();
            path_point.pose.orientation.z = local_orientation.z();
            path_point.pose.orientation.w = local_orientation.w();
            path_point.longitudinal_velocity_mps = local_velocity.x();
            path_point.lateral_velocity_mps = local_velocity.y();
            path_point.heading_rate_rps = heading_rate;
            path_point.is_final = (i + 1 == futurePoseList.size());
            future_path.points.push_back(path_point);

            if(i + 1 == futurePoseList.size())
            {
                future_pose_lidar.pose.pose = path_point.pose;
                future_pose_lidar.pose.covariance.fill(0.0);
            }
        }

        pubFuturePath.publish(future_path);
        pubFuturePoseLidar.publish(future_pose_lidar);
    }
};

class LidarOdometry : public ParamLoader
{
private:
    mutex mtx;
    bool doneFirstOpt;
    bool isFirstFrame;
    bool failureFrameFlag;
    ros::Time cloudTimeStamp;
    double cloudTimeCur;
    double cloudTimeLast;
    double lastOdomTime;
    double lastMappingInterval; // Last backend optimization interval
    bool doneBackOpt;

    Affine3f lastOdomAffine; // Previous odometry affine
    Affine3f lidarMappingAffine; // Transform between adjacent odometry frames
    Affine3f transformation_interpolated;
    Affine3d imuToLidarAffine;
    Affine3d lidarToImuAffine;

    std::chrono::_V2::system_clock::time_point start_time;

    // ROS wrraper
    ros::Subscriber subCloudInfo;
    ros::Subscriber subImu;
    // TODO: receive backend optimized pose
    ros::Subscriber subOdometryMapped;
    ros::Publisher pubFrontCloudInfo;
    ros::Publisher pubLidarOdometry;
    ros::Publisher pubLaserPath;
    ros::Publisher pubLidarPose;
    ros::Publisher pubRegScan;
    ros::Publisher pubPlotData;
    
    nav_msgs::Path laser_odom_path;
    nav_msgs::Odometry laser_odom_incremental;
    geometry_msgs::PoseStamped laser_pose;
    pcl::PointCloud<PointType>::Ptr RegCloud;
    
    // Current frame data
    rolo::CloudInfoStamp laserCloudInfoLast;
    pcl::PointCloud<PointType>::Ptr FullCloudLast;
    pcl::PointCloud<PointType>::Ptr CloudCornerLast;
    pcl::PointCloud<PointType>::Ptr CloudSurfLast;
    pcl::PointCloud<PointType>::Ptr CloudGroundLast;
    pcl::PointCloud<PointType>::Ptr ground_and_cornerLast;
    pcl::PointCloud<PointType>::Ptr featureLast;

    // Previous frame data
    rolo::CloudInfoStamp laserCloudInfoOld;
    pcl::PointCloud<PointType>::Ptr FullCloudOld;
    pcl::PointCloud<PointType>::Ptr CloudCornerOld;
    pcl::PointCloud<PointType>::Ptr CloudSurfOld;
    pcl::PointCloud<PointType>::Ptr CloudGroundOld;
    pcl::PointCloud<PointType>::Ptr ground_and_cornerOld;
    pcl::PointCloud<PointType>::Ptr featureOld;

    std::queue<rolo::CloudInfoStamp> laserCloudInfoBuf;

    std::mutex imuMtx;
    bool imuIntegrationStarted = false;
    bool imuDeltaAvailable = false;
    double imuWindowStartTime = -1.0;
    double imuLastTime = -1.0;
    double lastRotationSolveMs = 0.0;
    double lastTranslationSolveMs = 0.0;
    Quaterniond imuDeltaRot = Quaterniond::Identity(); // current IMU frame w.r.t. scan start
    Vector3d imuDeltaVel = Vector3d::Zero();
    Vector3d imuDeltaPos = Vector3d::Zero();
    
    Matrix3d Rotation;
    Vector3d Translation;
    Vector3d TranslationOld;
    float LaserOdomPose[6] = {
        static_cast<float>(initPose[0]), static_cast<float>(initPose[1]),
        static_cast<float>(initPose[2]), static_cast<float>(initPose[3]),
        static_cast<float>(initPose[4]), static_cast<float>(initPose[5])}; // [x, y, z, roll, pitch, yaw]


public:  
    LidarOdometry():
    doneFirstOpt(true),
    isFirstFrame(true),
    failureFrameFlag(false),
    doneBackOpt(false)
    {

        // Subscribe to backend optimized odometry
        subOdometryMapped = nh.subscribe<nav_msgs::Odometry>("rolo/mapping/odometry", 10, &LidarOdometry::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        // Subscribe to feature cloud info
        subCloudInfo = nh.subscribe<rolo::CloudInfoStamp>("rolo/feature/cloud_info", 10, &LidarOdometry::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        if (imuEnable)
            subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &LidarOdometry::imuHandler, this, ros::TransportHints().tcpNoDelay());
        // Publish predicted front-end odometry
        pubFrontCloudInfo = nh.advertise<rolo::CloudInfoStamp>(odomTopic+"/cloud_info", 2000);
        pubLidarOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);
        if (debugMode)
            pubLidarPose = nh.advertise<geometry_msgs::PoseStamped> (odomTopic+"_incremental/pose", 2000);
        if (debugMode)
            pubLaserPath = nh.advertise<nav_msgs::Path> (odomTopic+"_incremental/path", 2000);
        pubRegScan = nh.advertise<sensor_msgs::PointCloud2> (odomTopic+"/registration_scan", 10);
        pubPlotData = nh.advertise<std_msgs::Float64MultiArray> ("rolo/data_test", 10);
        Init();
    }
    ~LidarOdometry(){}

    void Init(){
        Rotation = Matrix3d::Identity();
        Translation = Vector3d::Zero();
        TranslationOld = Vector3d::Zero();
        lidarMappingAffine = Affine3f::Identity();
        lastOdomAffine = Affine3f::Identity();
        transformation_interpolated = Affine3f::Identity();
        imuToLidarAffine = Affine3d::Identity();
        imuToLidarAffine.linear() = imuToLidarRot;
        imuToLidarAffine.translation() = imuToLidarTrans;
        lidarToImuAffine = imuToLidarAffine.inverse();
        lastOdomTime = -1;
        RegCloud.reset(new pcl::PointCloud<PointType>());
        lastMappingInterval = 9999.0;
        // Current frame data
        FullCloudLast.reset(new pcl::PointCloud<PointType>());
        CloudCornerLast.reset(new pcl::PointCloud<PointType>());
        CloudSurfLast.reset(new pcl::PointCloud<PointType>());
        CloudGroundLast.reset(new pcl::PointCloud<PointType>());
        ground_and_cornerLast.reset(new pcl::PointCloud<PointType>());
        featureLast.reset(new pcl::PointCloud<PointType>());

        // Previous frame data
        FullCloudOld.reset(new pcl::PointCloud<PointType>());
        CloudCornerOld.reset(new pcl::PointCloud<PointType>());
        CloudSurfOld.reset(new pcl::PointCloud<PointType>());
        CloudGroundOld.reset(new pcl::PointCloud<PointType>());
        ground_and_cornerOld.reset(new pcl::PointCloud<PointType>());
        featureOld.reset(new pcl::PointCloud<PointType>());
        start_time = std::chrono::system_clock::now();

    }

    //! Fuse backend odometry into the front-end stream
    void odometryHandler(const nav_msgs::OdometryConstPtr &mappedOdom){
        // Current odometry time
        double currentCorrectionTime = mappedOdom->header.stamp.toSec();
        nav_msgs::Odometry mappedOdom_ = *mappedOdom;
        lastOdomTime = currentCorrectionTime;
        doneBackOpt = true;
    }

    void resetImuIntegration(double scanStartTime)
    {
        std::lock_guard<std::mutex> lock(imuMtx);
        imuIntegrationStarted = true;
        imuDeltaAvailable = false;
        imuWindowStartTime = scanStartTime;
        imuLastTime = scanStartTime;
        imuDeltaRot = Quaterniond::Identity();
        imuDeltaVel.setZero();
        imuDeltaPos.setZero();
    }

    Vector3d gravityCompensatedAcc(const sensor_msgs::Imu& imuMsg)
    {
        Vector3d acc(imuMsg.linear_acceleration.x,
                     imuMsg.linear_acceleration.y,
                     imuMsg.linear_acceleration.z);
        Quaterniond qWorldImu(imuMsg.orientation.w,
                              imuMsg.orientation.x,
                              imuMsg.orientation.y,
                              imuMsg.orientation.z);
        if (imuMsg.orientation_covariance[0] == -1 || qWorldImu.norm() < 0.1)
            return Vector3d::Zero();

        qWorldImu.normalize();
        const Vector3d gravityWorld(0.0, 0.0, 9.81);
        return acc - qWorldImu.inverse() * gravityWorld;
    }

    void imuHandler(const sensor_msgs::ImuConstPtr& imuMsg)
    {
        std::lock_guard<std::mutex> lock(imuMtx);
        if (!imuIntegrationStarted)
            return;

        double imuTime = imuMsg->header.stamp.toSec();
        if (imuTime <= imuWindowStartTime || imuTime <= imuLastTime)
            return;

        double dt = imuTime - imuLastTime;
        if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0)
        {
            imuLastTime = imuTime;
            return;
        }

        Vector3d gyr(imuMsg->angular_velocity.x,
                     imuMsg->angular_velocity.y,
                     imuMsg->angular_velocity.z);
        Vector3d deltaAngle = gyr * dt;
        double angle = deltaAngle.norm();
        if (angle > 1e-12)
            imuDeltaRot = (imuDeltaRot * Quaterniond(AngleAxisd(angle, deltaAngle / angle))).normalized();

        Vector3d accStart = imuDeltaRot * gravityCompensatedAcc(*imuMsg);
        imuDeltaPos += imuDeltaVel * dt + 0.5 * accStart * dt * dt;
        imuDeltaVel += accStart * dt;
        imuLastTime = imuTime;
        imuDeltaAvailable = true;
    }

    bool getImuInitialGuess(double scanEndTime, Affine3f& initialGuess)
    {
        std::lock_guard<std::mutex> lock(imuMtx);
        if (!imuEnable || !imuIntegrationStarted || !imuDeltaAvailable || imuLastTime < scanEndTime - 0.02)
            return false;

        Affine3d imuStartToCur = Affine3d::Identity();
        imuStartToCur.linear() = imuDeltaRot.toRotationMatrix();
        /*  IMU translation integration is not accurate */
        // imuStartToCur.translation() = imuDeltaPos;
        Affine3d lidarCurToStart = imuToLidarAffine * imuStartToCur.inverse() * lidarToImuAffine;
        // Affine3d lidarCurToStart = imuToLidarAffine * imuStartToCur * lidarToImuAffine;
        initialGuess = lidarCurToStart.cast<float>();
        return true;
    }

    void scanRegeistration(){
        auto start = std::chrono::steady_clock::now();
        // std::chrono::duration<double> elapsed_seconds = end - start;
        // printf("Solver Duration: %f ms.\n" ,elapsed_seconds.count() * 1000);

        pcl::PointCloud<PointType>::Ptr feature_propagated(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr feature_rotated(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>);
        feature_propagated->clear();
        feature_rotated->clear();
        // Translate-interpolate for center alignment
        pcl::transformPointCloud(*featureOld, *feature_propagated, transformation_interpolated);
        rolo::SVGICP<PointType, PointType> svgicp;
        // svgicp.setResolution(1.0);
        svgicp.setPolarResolution(0.175, 0.175, 2.0);
        svgicp.setNumThreads(omp_get_max_threads());
        svgicp.clearTarget();
        svgicp.clearSource();
        svgicp.setInputTarget(featureLast);
        svgicp.setInputSource(feature_propagated);
        svgicp.align(*aligned);
        Eigen::Matrix4f trans = svgicp.getFinalTransformation(); // Rotation estimate
        // Rotation = trans.block<3, 3>(0, 0).cast<float>() * Rotation.eval();
        Eigen::Affine3f transformStep;
        transformStep.matrix() = trans.cast<float>();
        transformation_interpolated = transformation_interpolated * transformStep;
        Rotation = transformation_interpolated.rotation().cast<double>();
        Translation = transformation_interpolated.translation().cast<double>();
        auto r_end = std::chrono::steady_clock::now();
        lastRotationSolveMs = elapsedMillis(start, r_end);

        // Translation registration
        // Apply rotation first
        aligned->clear();
        pcl::transformPointCloud(*featureOld, *feature_rotated, transformation_interpolated);
        Eigen::Vector3d Reg_translation = Eigen::Vector3d::Zero();
        svgicp.computeTranslation(*aligned, Reg_translation, Translation, TranslationOld, 0.1, 0.1, CT_lambda);

        auto t_end = std::chrono::steady_clock::now();
        lastTranslationSolveMs = elapsedMillis(r_end, t_end);

        Translation += Reg_translation;
    }

    void cloudHandler(const rolo::CloudInfoStampConstPtr &cloudIn){
        // Push timestamped cloud info into buffer
        cloudTimeStamp = cloudIn->header.stamp;
        cloudTimeCur = cloudIn->header.stamp.toSec();
        laserCloudInfoBuf.push(*cloudIn);
        // Match cloud timestamp
        ros::Time TimeCur = ros::Time::now();
        const std::size_t queueSize = laserCloudInfoBuf.size();
        for (std::size_t i = 0; i < queueSize; ++i) {
            laserCloudInfoLast = laserCloudInfoBuf.front();
            laserCloudInfoBuf.pop();
            cloudTimeStamp = laserCloudInfoLast.header.stamp;
            cloudTimeCur = laserCloudInfoLast.header.stamp.toSec();
            if(std::fabs((TimeCur-cloudTimeStamp).toSec()) < 0.1){
                break;
            }
        }

        // Extracted cloud
        pcl::fromROSMsg(laserCloudInfoLast.extracted_corner,  *CloudCornerLast);
        pcl::fromROSMsg(laserCloudInfoLast.extracted_surface, *CloudSurfLast);
        pcl::fromROSMsg(laserCloudInfoLast.cloud_projected, *FullCloudLast);
        *featureLast = *CloudCornerLast + *CloudSurfLast;

        if(isFirstFrame){
            isFirstFrame = false;
            cloudTimeLast = cloudTimeCur;
            *FullCloudOld = *FullCloudLast;
            *CloudCornerOld = *CloudCornerLast;
            *CloudSurfOld = *CloudSurfLast;
            *featureOld = *featureLast;
            if (imuEnable)
                resetImuIntegration(cloudTimeCur);
            return;
        }

        Affine3f imuInitialGuess = Affine3f::Identity();
        bool hasImuInitialGuess = getImuInitialGuess(cloudTimeCur, imuInitialGuess);
        if (imuEnable)
            resetImuIntegration(cloudTimeCur);

        // Check first global optimization
        // if (doneFirstOpt == false)
        if (lastOdomTime == -1.0){
            updateTransform();
            pubMessage();
            return;
        }

        // Forward-propagate state.
        if(lastOdomTime != -1.0){   // Only after initialization
            double latestInterval = cloudTimeCur - cloudTimeLast;
            if (hasImuInitialGuess){
                stateLinearPropagation(lidarMappingAffine, lastMappingInterval, latestInterval, transformation_interpolated);
                transformation_interpolated.linear() = imuInitialGuess.rotation();
            }
            else
                stateLinearPropagation(lidarMappingAffine, lastMappingInterval, latestInterval, transformation_interpolated);
            Rotation = transformation_interpolated.rotation().cast<double>();
            Translation = transformation_interpolated.translation().cast<double>();
            doneBackOpt = false;
            cloudTimeLast = cloudTimeCur; // Interpolate adjacent frames only
            lastMappingInterval = latestInterval;
        }

        scanRegeistration();
        const Eigen::Vector3d solvedEuler = Rotation.transpose().eulerAngles(0, 1, 2) * 180.0 / M_PI;
        LOG(INFO) << ROLO_COLOR_FRONTEND
                    << "\n========== Frontend Lidar Odometry =========="
                    << "\nimu_enable: " << (imuEnable ? "true" : "false")
                    << "\nSolving rotation : " << lastRotationSolveMs << " (ms)"
                    << "\nSolving translation : " << lastTranslationSolveMs << " (ms)"
                    << "\nSolved Rotation (deg): " << std::fixed << std::setprecision(6)
                    << "[" << solvedEuler.x() << ", " << solvedEuler.y() << ", " << solvedEuler.z() << "]"
                    << "\nSolved Translation (m): " << std::fixed << std::setprecision(6)
                    << "[" << -Translation.x() << ", " << -Translation.y() << ", " << -Translation.z() << "]" 
                    << "\n============================================="
                    << ROLO_COLOR_RESET;

        updateTransform();
        if(!failureFrameFlag){
            // Publish odometry and TF
            pubMessage();
            pubTranform();
        }
        else{
            LOG(WARNING) << ROLO_COLOR_FRONTEND << "[lidarOdometry] failure transformation, resetting." << ROLO_COLOR_RESET;
            failureFrameFlag = false;
        }
    }

    void updateTransform(){
        Matrix4d trans = Matrix4d::Identity();
        trans << Rotation;
        trans.col(3) << Translation(0,0), Translation(1,0), Translation(2,0), 1.0;
        size_t cloudSize = FullCloudLast->points.size();

        RegCloud->clear();
        RegCloud->resize(cloudSize);
        RegCloud->points = FullCloudLast->points;
        pcl::transformPointCloud(*FullCloudLast, *RegCloud, trans);
        Affine3f transform_affine = pcl::getTransformation(LaserOdomPose[0], 
                                                           LaserOdomPose[1], 
                                                           LaserOdomPose[2], 
                                                           LaserOdomPose[3], 
                                                           LaserOdomPose[4], 
                                                           LaserOdomPose[5]);
        // Affine3d transform_affine_double = transform_affine.cast<double>;
        Affine3f transformStep;
        transformStep.matrix() = trans.cast<float>();
        Affine3f transformed_pose = transform_affine * transformStep.inverse(); // Affine updates are right-multiplied
        lidarMappingAffine = transformStep;
        
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        // if(!failureDetection(transform_affine, transformed_pose, elapsed_seconds.count()*1.0e6)){
        //     failureFrameFlag = true;
        //     return;
        // }

        start_time = end_time;
        // Vector3f rotation_euler;
        // float x, y, z;
        // pcl::getTranslationAndEulerAngles<float>(transformStep, 
        //                                          x, y, z,
        //                                          rotation_euler[0],
        //                                          rotation_euler[1], 
        //                                          rotation_euler[2]); 
        // std::cout << "rotation angles: " << std::endl << rotation_euler*180/M_PI << std::endl;

        pcl::getTranslationAndEulerAngles<float>(transformed_pose, 
                                          LaserOdomPose[0], 
                                          LaserOdomPose[1], 
                                          LaserOdomPose[2], 
                                          LaserOdomPose[3], 
                                          LaserOdomPose[4], 
                                          LaserOdomPose[5]);                            

        
        // Swap current and previous data
        *FullCloudOld = *FullCloudLast;
        *CloudCornerOld = *CloudCornerLast;
        *CloudSurfOld = *CloudSurfLast;
        *featureOld = *featureLast;
        TranslationOld = Translation;
    }

    //! Reject front-end odometry jumps
    bool failureDetection(Affine3f pose_affine, Affine3f pose_affine_transformed, double delt_Time){
        float x, y, z, roll, pitch, yaw;
        float t_x, t_y, t_z, t_roll, t_pitch, t_yaw;
        auto t_sq = pow(delt_Time, 2);
        pcl::getTranslationAndEulerAngles<float>(pose_affine, 
                                          x, y, z, roll, pitch, yaw);
        pcl::getTranslationAndEulerAngles<float>(pose_affine_transformed, 
                                          t_x, t_y, t_z, t_roll, t_pitch, t_yaw); 
        float delt_t = (t_x-x)*(t_x-x) + (t_y-y)*(t_y-y) + (t_z-z)*(t_z-z);
        float delt_r = (t_roll-roll)*(t_roll-roll) + (t_pitch-pitch)*(t_pitch-pitch) + (t_yaw-yaw)*(t_yaw-yaw);
        if(delt_t/t_sq >= 5.0 || delt_r/t_sq >= pow(0.2, 2)){
            return false;
        }
        return true;
    }
    
    void pubTranform(){
        // Publish odometry and TF
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(LaserOdomPose[3], LaserOdomPose[4], LaserOdomPose[5]),
                                                      tf::Vector3(LaserOdomPose[0], LaserOdomPose[1], LaserOdomPose[2]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, cloudTimeStamp, odometryFrame, "lidar");
        br.sendTransform(trans_odom_to_lidar);
    }

    void pubMessage(){
        publishCloud(pubRegScan, RegCloud, cloudTimeStamp, baselinkFrame);
        
        // Publish pose
        laser_pose.header.frame_id = odometryFrame;
        laser_pose.header.stamp = cloudTimeStamp;
        laser_pose.pose.position.x = LaserOdomPose[0];
        laser_pose.pose.position.y = LaserOdomPose[1];
        laser_pose.pose.position.z = LaserOdomPose[2];
        tf::Quaternion q = tf::createQuaternionFromRPY(LaserOdomPose[3], LaserOdomPose[4], LaserOdomPose[5]);
        laser_pose.pose.orientation.x = q.x();
        laser_pose.pose.orientation.y = q.y();
        laser_pose.pose.orientation.z = q.z();
        laser_pose.pose.orientation.w = q.w();        
        if (debugMode)
            pubLidarPose.publish(laser_pose);

        laser_odom_path.header.frame_id = odometryFrame;
        laser_odom_path.header.stamp = cloudTimeStamp;
        laser_odom_path.poses.push_back(laser_pose);
        nav_msgs::Path laser_odom_path2 = laser_odom_path;
        std::reverse(laser_odom_path2.poses.begin(), laser_odom_path2.poses.end());
        if (debugMode)
            pubLaserPath.publish(laser_odom_path2);

        // Publish incremental odometry
        laser_odom_incremental.header.frame_id = odometryFrame;
        laser_odom_incremental.header.stamp = cloudTimeStamp; //ros::Time::now();
        laser_odom_incremental.child_frame_id = "lidar_odometry";
        laser_odom_incremental.pose.pose = laser_pose.pose;
        pubLidarOdometry.publish(laser_odom_incremental);

        // Publish initial pose estimate
        rolo::CloudInfoStamp odometry_cloud;
        odometry_cloud = laserCloudInfoLast;
        odometry_cloud.initialGuessX = LaserOdomPose[0];
        odometry_cloud.initialGuessY = LaserOdomPose[1];
        odometry_cloud.initialGuessZ = LaserOdomPose[2];
        odometry_cloud.initialGuessRoll = LaserOdomPose[3];
        odometry_cloud.initialGuessPitch = LaserOdomPose[4];
        odometry_cloud.initialGuessYaw = LaserOdomPose[5];
        odometry_cloud.odomAvailable = true;
        pubFrontCloudInfo.publish(odometry_cloud);
    }

    //! Interpolate from previous backend transform
    void stateLinearPropagation(const Eigen::Affine3f& last_trans, const double& last_interval, const double& curr_interval,
                                Eigen::Affine3f &curr_trans){
        double propagation_ratio = curr_interval / last_interval;
        Eigen::Matrix<float, 6, 1> trans_vec;
        pcl::getTranslationAndEulerAngles(last_trans,
                                          trans_vec(0), trans_vec(1), trans_vec(2),
                                          trans_vec(3), trans_vec(4), trans_vec(5));
        trans_vec.tail(3) = Eigen::Matrix<float, 3, 1>::Zero();
        // std::cout << "transformation: \n" << trans_vec.transpose() << std::endl;
        trans_vec *= propagation_ratio;
        curr_trans = pcl::getTransformation(trans_vec(0), trans_vec(1), trans_vec(2),
                                            trans_vec(3), trans_vec(4), trans_vec(5));
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rolo");
    initLogging(argv[0]);
    
    LOG(INFO) << ROLO_COLOR_FRONTEND << "----> Laser Odometry Started." << ROLO_COLOR_RESET;
    LidarOdometry LO;
    TransformFusion TF;
    

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    // ros::spin();
    
    return 0;
}
