#include "rolo/utility.h"
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

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <rot_gicp/gicp/rot_vgicp.hpp>

using namespace Eigen;

//! 获取给定odom消息所代表的变换矩阵
Eigen::Affine3f odom2affine(nav_msgs::Odometry odom) // Eigen::Affine3f为仿射变换矩阵，旋转矩阵和平移矩阵的结合
{
    double x, y, z, roll, pitch, yaw;
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    return pcl::getTransformation(x, y, z, roll, pitch, yaw);
}

class TransformFusion : public ParamLoader
{
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubLidarOdometry;
    ros::Publisher pubLidarPath;

    Eigen::Affine3f mappingOdomAffine;
    Eigen::Affine3f lidarOdomAffineFront;
    Eigen::Affine3f lidarOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double mappingOdomTime = -1;
    deque<nav_msgs::Odometry> lidarOdomQueue;
    //! 读取base-lidar的TF，声明输入输出
    TransformFusion()
    {
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }
        // 接受后端优化里程和预积分传过来的历程
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("rolo/mapping/odometry", 5, &TransformFusion::mappingOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::lidarOdometryHandler,   this, ros::TransportHints().tcpNoDelay());
        // 发布IMU里程计
        pubLidarOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubLidarPath       = nh.advertise<nav_msgs::Path>("rolo/lidar_odometry/path", 1);
    }
    // //! 获取给定odom消息所代表的变换矩阵
    // Eigen::Affine3f odom2affine(nav_msgs::Odometry odom) // Eigen::Affine3f为仿射变换矩阵，旋转矩阵和平移矩阵的结合
    // {
    //     double x, y, z, roll, pitch, yaw;
    //     x = odom.pose.pose.position.x;
    //     y = odom.pose.pose.position.y;
    //     z = odom.pose.pose.position.z;
    //     tf::Quaternion orientation;
    //     tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
    //     tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    //     return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    // }
    //! 存储lidar_odom消息的变换关系
    void mappingOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        mappingOdomAffine = odom2affine(*odomMsg);

        mappingOdomTime = odomMsg->header.stamp.toSec();
    }
    //! imu预积分里程回调函数，根据后端优化后的激光里程消息，结合imu的位姿估计，得到当前时刻的位姿，并发布TF和odom消息，imu path
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // static tf
        // 设置map和odom坐标系重合，发布静态TF
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (mappingOdomTime == -1)
            return;
        while (!lidarOdomQueue.empty())
        {
            // 前端里程和后端里程进行时间标定
            if (lidarOdomQueue.front().header.stamp.toSec() <= mappingOdomTime)
                lidarOdomQueue.pop_front();
            else
                break;
        }

        Eigen::Affine3f lidarOdomAffineFront = odom2affine(lidarOdomQueue.front());
        Eigen::Affine3f lidarOdomAffineBack = odom2affine(lidarOdomQueue.back());
        // 求前后两帧lidar里程计位姿的变换关系
        Eigen::Affine3f lidarOdomAffineIncre = lidarOdomAffineFront.inverse() * lidarOdomAffineBack;
        Eigen::Matrix<float, 6, 1> lidar_pose_front;
        Eigen::Matrix<float, 6, 1> lidar_pose_back;
        Eigen::Matrix<float, 6, 1> lidar_pose_incre;
        // pcl::getTranslationAndEulerAngles(lidarOdomAffineFront, lidar_pose_front(0),
        //                                                         lidar_pose_front(1),
        //                                                         lidar_pose_front(2),
        //                                                         lidar_pose_front(3),
        //                                                         lidar_pose_front(4),
        //                                                         lidar_pose_front(5));
        // pcl::getTranslationAndEulerAngles(lidarOdomAffineBack, lidar_pose_back(0),
        //                                                         lidar_pose_back(1),
        //                                                         lidar_pose_back(2),
        //                                                         lidar_pose_back(3),
        //                                                         lidar_pose_back(4),
        //                                                         lidar_pose_back(5));
        // pcl::getTranslationAndEulerAngles(lidarOdomAffineIncre, lidar_pose_incre(0),
        //                                                         lidar_pose_incre(1),
        //                                                         lidar_pose_incre(2),
        //                                                         lidar_pose_incre(3),
        //                                                         lidar_pose_incre(4),
        //                                                         lidar_pose_incre(5));

        // std::cout << "lidar_pose_front: " << lidar_pose_front.transpose() << std::endl;
        // std::cout << "lidar_pose_back: " << lidar_pose_back.transpose() << std::endl;
        // std::cout << "lidar_pose_incre: " << lidar_pose_incre.transpose() << std::endl;
        
        // 对最新的激光里程位姿进行相应变换，得到当前帧的激光里程估计
        Eigen::Affine3f lidarOdomAffineLast = mappingOdomAffine * lidarOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(lidarOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        // publish latest odometry
        nav_msgs::Odometry laserOdometry = lidarOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubLidarOdometry.publish(laserOdometry);

        // publish tf
        // 发布odom->base_link的TF
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish Lidar odometry path
        static nav_msgs::Path lidarPath;
        static double last_path_time = -1;
        double lidarTime = lidarOdomQueue.back().header.stamp.toSec();
        if (lidarTime - last_path_time > 0.05)
        {
            last_path_time = lidarTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = lidarOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            lidarPath.poses.push_back(pose_stamped);
            // 只保留1s的imu里程计轨迹
            while(!lidarPath.poses.empty() && lidarPath.poses.front().header.stamp.toSec() < mappingOdomTime - 1.0)
                lidarPath.poses.erase(lidarPath.poses.begin());
            if (pubLidarPath.getNumSubscribers() != 0)
            {
                lidarPath.header.stamp = lidarOdomQueue.back().header.stamp;
                lidarPath.header.frame_id = odometryFrame;
                pubLidarPath.publish(lidarPath);
            }
        }
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
    double lastMappingInterval; // 上一次后端优化的时间间隔（以收到odom消息为准）
    bool doneBackOpt;

    Affine3f lastOdomAffine; // 上一时刻的odom对应的映射
    Affine3f lidarMappingAffine; // 相邻两帧odom之间的变换
    Affine3f transformation_interpolated;

    std::chrono::_V2::system_clock::time_point start_time;

    // ROS wrraper
    ros::Subscriber subCloudInfo;
    //TODO 接受后端的优化位姿
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
    
    // 当前帧数据
    rolo::CloudInfoStamp laserCloudInfoLast;
    pcl::PointCloud<PointType>::Ptr FullCloudLast;
    pcl::PointCloud<PointType>::Ptr CloudCornerLast;
    pcl::PointCloud<PointType>::Ptr CloudSurfLast;
    pcl::PointCloud<PointType>::Ptr CloudGroundLast;
    pcl::PointCloud<PointType>::Ptr ground_and_cornerLast;
    pcl::PointCloud<PointType>::Ptr featureLast;

    // 上一帧数据
    rolo::CloudInfoStamp laserCloudInfoOld;
    pcl::PointCloud<PointType>::Ptr FullCloudOld;
    pcl::PointCloud<PointType>::Ptr CloudCornerOld;
    pcl::PointCloud<PointType>::Ptr CloudSurfOld;
    pcl::PointCloud<PointType>::Ptr CloudGroundOld;
    pcl::PointCloud<PointType>::Ptr ground_and_cornerOld;
    pcl::PointCloud<PointType>::Ptr featureOld;

    std::queue<rolo::CloudInfoStamp> laserCloudInfoBuf;
    
    Matrix3d Rotation;
    Vector3d Translation;
    Vector3d TranslationOld;
    float LaserOdomPose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // [x, y, z, roll, pitch, yaw]


public:  
    LidarOdometry():
    doneFirstOpt(true),
    doneBackOpt(false),
    isFirstFrame(true),
    failureFrameFlag(false)
    {

        // mapOptimization传来的里程计数据
        subOdometryMapped = nh.subscribe<nav_msgs::Odometry>("rolo/mapping/odometry", 10, &LidarOdometry::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        // 接受imu原始数据
        subCloudInfo = nh.subscribe<rolo::CloudInfoStamp>("rolo/feature/cloud_info", 10, &LidarOdometry::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        // 发布imu预测里程计
        pubFrontCloudInfo = nh.advertise<rolo::CloudInfoStamp>(odomTopic+"/cloud_info", 2000);
        pubLidarOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);
        pubLidarPose = nh.advertise<geometry_msgs::PoseStamped> (odomTopic+"_incremental/pose", 2000);
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
        lastOdomTime = -1;
        RegCloud.reset(new pcl::PointCloud<PointType>());
        lastMappingInterval = 9999.0;
        // 当前帧数据
        FullCloudLast.reset(new pcl::PointCloud<PointType>());
        CloudCornerLast.reset(new pcl::PointCloud<PointType>());
        CloudSurfLast.reset(new pcl::PointCloud<PointType>());
        CloudGroundLast.reset(new pcl::PointCloud<PointType>());
        ground_and_cornerLast.reset(new pcl::PointCloud<PointType>());
        featureLast.reset(new pcl::PointCloud<PointType>());

        // 上一帧数据
        FullCloudOld.reset(new pcl::PointCloud<PointType>());
        CloudCornerOld.reset(new pcl::PointCloud<PointType>());
        CloudSurfOld.reset(new pcl::PointCloud<PointType>());
        CloudGroundOld.reset(new pcl::PointCloud<PointType>());
        ground_and_cornerOld.reset(new pcl::PointCloud<PointType>());
        featureOld.reset(new pcl::PointCloud<PointType>());
        start_time = std::chrono::system_clock::now();

    }

    //! 接受后端的里程计消息，并与前端里程计融合
    void odometryHandler(const nav_msgs::OdometryConstPtr &mappedOdom){
        // 当前时刻odom时间
        double currentCorrectionTime = mappedOdom->header.stamp.toSec();
        nav_msgs::Odometry mappedOdom_ = *mappedOdom;
        lastOdomTime = currentCorrectionTime;
        doneBackOpt = true;
    }

    void scanRegeistration(){
        // auto end = std::chrono::system_clock::now();
        // std::chrono::duration<double> elapsed_seconds = end - start;
        // printf("Solver Duration: %f ms.\n" ,elapsed_seconds.count() * 1000);

        pcl::PointCloud<PointType>::Ptr feature_propagated(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr feature_rotated(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>);
        feature_propagated->clear();
        feature_rotated->clear();
        // 先平移插值，使中心对齐
        pcl::transformPointCloud(*featureOld, *feature_propagated, transformation_interpolated);
        fast_gicp::RotVGICP<PointType, PointType> rot_vgicp;
        rot_vgicp.setResolution(1.0);
        rot_vgicp.setNumThreads(omp_get_max_threads());
        rot_vgicp.clearTarget();
        rot_vgicp.clearSource();
        rot_vgicp.setInputTarget(featureLast);
        rot_vgicp.setInputSource(feature_propagated);
        rot_vgicp.align(*aligned);
        Eigen::Matrix4f trans = rot_vgicp.getFinalTransformation();
        // Rotation = trans.block<3, 3>(0, 0).cast<float>() * Rotation.eval();
        Eigen::Affine3f transformStep;
        transformStep.matrix() = trans.cast<float>();
        transformation_interpolated = transformation_interpolated * transformStep;
        Rotation = transformation_interpolated.rotation().cast<double>();
        Translation = transformation_interpolated.translation().cast<double>();

        // Eigen::Vector3f rotation_euler;
        // float x, y, z;
        // pcl::getTranslationAndEulerAngles<float>(transformStep, 
        //                                             x, y, z,
        //                                             rotation_euler[0],
        //                                             rotation_euler[1], 
        //                                             rotation_euler[2]); 
        // std::cout << "rotation angles: " << std::endl << rotation_euler*180/M_PI << std::endl;

        //* 平移配准
        // 首先进行旋转
        aligned->clear();
        pcl::transformPointCloud(*featureOld, *feature_rotated, transformation_interpolated);
        Eigen::Vector3d Reg_translation = Eigen::Vector3d::Zero();
        rot_vgicp.computeTranslation(*aligned, Reg_translation, Translation, TranslationOld, 0.1, 0.1);
        // std::cout << "Reg_translation: " << Reg_translation.transpose() << std::endl;
        Translation += Reg_translation;
    }

    void cloudHandler(const rolo::CloudInfoStampConstPtr &cloudIn){
        // 取时间戳,入buffer
        cloudTimeStamp = cloudIn->header.stamp;
        cloudTimeCur = cloudIn->header.stamp.toSec();
        laserCloudInfoBuf.push(*cloudIn);
        // 进行时间匹配
        ros::Time TimeCur = ros::Time::now();
        for(int i=0; i<laserCloudInfoBuf.size(); i++){
            laserCloudInfoLast = laserCloudInfoBuf.front();
            laserCloudInfoBuf.pop();
            cloudTimeStamp = laserCloudInfoLast.header.stamp;
            cloudTimeCur = laserCloudInfoLast.header.stamp.toSec();
            if(std::fabs((TimeCur-cloudTimeStamp).toSec()) < 0.1){
                break;
            }
        }

        // 提取当前帧特征点云
        pcl::fromROSMsg(laserCloudInfoLast.extracted_corner,  *CloudCornerLast);
        pcl::fromROSMsg(laserCloudInfoLast.extracted_surface, *CloudSurfLast);
        pcl::fromROSMsg(laserCloudInfoLast.cloud_projected, *FullCloudLast);
        *featureLast = *CloudCornerLast + *CloudSurfLast;

        if(isFirstFrame){
            isFirstFrame = false;
            *FullCloudOld = *FullCloudLast;
            *CloudCornerOld = *CloudCornerLast;
            *CloudSurfOld = *CloudSurfLast;
            *featureOld = *featureLast;
            return;
        }

        // 是否完成第一次全图优化
        // if (doneFirstOpt == false)
        if (lastOdomTime == -1.0){
            updateTransform();
            pubMessage();
            return;
        }

        // 状态前向插值
        if(lastOdomTime != -1.0){   // 未初始化则不进行插值
            double latestInterval = cloudTimeCur - cloudTimeLast;
            
            stateLinearPropagation(lidarMappingAffine, lastMappingInterval, latestInterval, transformation_interpolated);
            Rotation = transformation_interpolated.rotation().cast<double>();
            Translation = transformation_interpolated.translation().cast<double>();
            if(Translation.array().maxCoeff() > 5.0){
                std::cout << "Translation: \n" << Translation << std::endl;
            }
            doneBackOpt = false;
            cloudTimeLast = cloudTimeCur; // 仅对相邻两帧之间进行插值
            lastMappingInterval = latestInterval;
        }

        scanRegeistration();

        updateTransform();
        if(!failureFrameFlag){
            // 发布ROS消息和TF
            pubMessage();
            pubTranform();
        }
        else{
            printf(" Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n Failure Transformation! Resetting! \n "); 
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
        Affine3f transformed_pose = transform_affine * transformStep.inverse(); // 仿射变换遵循右乘原则
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

        
        // 新旧信息交换
        *FullCloudOld = *FullCloudLast;
        *CloudCornerOld = *CloudCornerLast;
        *CloudSurfOld = *CloudSurfLast;
        *featureOld = *featureLast;
        TranslationOld = Translation;
    }

    //! 检测前端里程计估计是否发生跳变，如果发生跳变，忽略当前帧的估计结果
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
        // 发布TF
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(LaserOdomPose[3], LaserOdomPose[4], LaserOdomPose[5]),
                                                      tf::Vector3(LaserOdomPose[0], LaserOdomPose[1], LaserOdomPose[2]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, cloudTimeStamp, odometryFrame, "lidar");
        br.sendTransform(trans_odom_to_lidar);
    }

    void pubMessage(){
        publishCloud(pubRegScan, RegCloud, cloudTimeStamp, baselinkFrame);
        
        // 发布位姿
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
        pubLidarPose.publish(laser_pose);

        // 发布Path
        laser_odom_path.header.frame_id = odometryFrame;
        laser_odom_path.header.stamp = cloudTimeStamp;
        laser_odom_path.poses.push_back(laser_pose);
        nav_msgs::Path laser_odom_path2 = laser_odom_path;
        std::reverse(laser_odom_path2.poses.begin(), laser_odom_path2.poses.end());
        pubLaserPath.publish(laser_odom_path2);

        // 发布里程计
        laser_odom_incremental.header.frame_id = odometryFrame;
        laser_odom_incremental.header.stamp = cloudTimeStamp; //ros::Time::now();
        laser_odom_incremental.child_frame_id = "lidar_odometry";
        laser_odom_incremental.pose.pose = laser_pose.pose;
        pubLidarOdometry.publish(laser_odom_incremental);

        // 发布初始位姿估计
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

    //! 针对上一时刻的后端变换进行线性插值
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
    
    LidarOdometry LO;
    TransformFusion TF;
    
    ROS_INFO("\033[1;32m----> Laser Odometry Started.\033[0m");

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    // ros::spin();
    
    return 0;
}