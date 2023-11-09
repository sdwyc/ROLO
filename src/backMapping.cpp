#include "rolo/utility.h"
// #include "rolo/save_map.h"

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

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose


/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

class backMapping : public ParamLoader
{
public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;    // NonlinearFactorGraph相当于一个容器，用于向里面添加新的因子
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;    // ISAM其实是一种求解非线性最小二乘问题的方法
    Values isamCurrentEstimate; //当前时刻的最优估计位姿
    Eigen::MatrixXd poseCovariance; //当前时刻的状态估计协方差矩阵

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;

    ros::Publisher pubSLAMInfo;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;

    ros::ServiceServer srvSaveMap;

    std::deque<nav_msgs::Odometry> gpsQueue;
    rolo::CloudInfoStamp cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;   // 所有关键帧的角点集合（降采样）
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames; // 所有关键帧的平面点集合（降采样）
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;    // 历史关键帧状态的坐标位置，intensity为索引位置
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;// 历史关键帧状态的6D位姿，intensity为索引位置
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // 当前帧的角点集合 // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // 当前帧的平面点集合 // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudNormalLast;     // 当前帧的普通点集合
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // 降采样后的当前帧角点集合 // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;   // 降采样后的当前帧平面点集合 // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;  // 当前帧，雷达坐标系下，有效的特征点集合
    pcl::PointCloud<PointType>::Ptr coeffSel;       // 当前帧，雷达坐标系下，有效的特征点的点线+点面参数集合

    std::vector<PointType> laserCloudOriCornerVec; // 雷达坐标系下，能够作点线匹配的点集// corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;   // 能够作点线匹配的点集的点线参数，线的方向单位向量和距离
    std::vector<bool> laserCloudOriCornerFlag;  // 筛选出有效点线关系的点集Mask，初始值均为false
    std::vector<PointType> laserCloudOriSurfVec; // 雷达坐标系下，能够作点面匹配的点集 // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;     // 能够作点面匹配的点集的点面参数，点到平面的距离和平面法向量
    std::vector<bool> laserCloudOriSurfFlag;    // 筛选出有效点面关系的点集Mask，初始值均为false

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer; // map中的所有关键帧，key为索引值，value：first为角点，second为平面点
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;    // 全局坐标系下，周围关键帧的角点集合
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;      // 全局坐标系下，周围关键帧的平面点集合
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;  // 降采样后的周围关键帧特征点云
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];   // 全局雷达里程计位姿，初始值均为0，[roll, pitch, yaw, x, y, z]

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false; // 回环因子添加标志位
    map<int, int> loopIndexContainer; // 所有的建立的回环对集合 // from new to old
    vector<pair<int, int>> loopIndexQueue;  // 匹配上的回环对，first为历史时刻的关键帧索引，second为当前的关键帧索引
    vector<gtsam::Pose3> loopPoseQueue; // 匹配上的回环对所对应的位姿变换阵
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue; // 匹配上的回环对所对应的噪声模型
    deque<std_msgs::Float64MultiArray> loopInfoVec; // 外部给定的回环对，每个元素是一个数组，每个数组两个元素，0：当前帧，1：历史帧

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront; // 上一时刻的全局里程计位姿
    Eigen::Affine3f incrementalOdometryAffineBack;

    backMapping(){
        // isam初始化参数
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters); // 实例化ISAM

        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/trajectory", 1);  // 全局路径
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/map_global", 1);  // 全局地图
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("rolo/mapping/odometry", 1);         // 里程计
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("rolo/mapping/odometry_incremental", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("rolo/mapping/path", 1);  // 全局路径
        // Feature extration传过来的cloud_info
        subCloud = nh.subscribe<rolo::CloudInfoStamp>(odomTopic+"/cloud_info", 1, &backMapping::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        // GPS数据
        // subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &backMapping::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        // 回环数据
        // subLoop  = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &backMapping::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // srvSaveMap  = nh.advertiseService("rolo/save_map", &backMapping::saveMapService, this);

        // 历史关键帧点云
        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/icp_loop_closure_history_cloud", 1);
        // 关联关键帧点云
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/rolo/mapping/loop_closure_constraints", 1);
        //  局部地图
        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/map_local", 1);
        // 当前关键帧
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/cloud_registered_raw", 1);

        pubSLAMInfo           = nh.advertise<rolo::CloudInfoStamp>("rolo/mapping/slam_info", 1);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization
        // 为变量分配内存空间，赋初值
        allocateMemory();
    }

    //! 对输入值进行限幅输出
    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    //! 对输入点进行坐标变换，变换到全局map坐标系下
    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }
    //! 对给定点云中的空间点进行给定的坐标变换
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);
        // 得到变换矩阵
        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        // #pragma omp parallel for 会将下一行的 for 循环自动分块并行处理
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }
    //! 将pcl点云转换为gtsam的pose3格式
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }
    //! 将位姿数组转换为gtsam的pose3格式
    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }
    //! 将给定点的6D坐标，转换为eigen的变换矩阵
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }
    //! 将位姿变换阵转换为eigen库下的矩阵形式
    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }
    //! 格式转换
    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    void allocateMemory(){
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudNormalLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization        
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    //! 激光回调函数，
    void laserCloudInfoHandler(const rolo::CloudInfoStampConstPtr& msgIn){
        // extract time stamp 提取时间戳
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        // extract info and feature cloud 提取当前帧的特征点云（角点+平面点）
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->extracted_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->extracted_surface, *laserCloudSurfLast);
        pcl::fromROSMsg(msgIn->extracted_normal, *laserCloudNormalLast);
        *laserCloudSurfLast += *laserCloudNormalLast;

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        // 当时间间隔大于阈值时，才进行后端优化
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            // 更新时间
            timeLastProcessing = timeLaserInfoCur;
            // 根据前端匹配结果，得到当前时刻的先验位姿估计
            updateInitialGuess();
            // 提取周围的关键帧，并提取其角点和平面点
            extractSurroundingKeyFrames();
            // 对当前帧的平面点和角点进行降采样
            downsampleCurrentScan();
            // 对周围关键帧寻找有效的点线约束和点面约束，构建非线性问题，优化位姿，并于imu数据融合
            scan2MapOptimization();
            // 添加因子，全局优化，保存最优状态估计及其对应的特征点
            saveKeyFramesAndFactor();
            // 发生回环后，对历史上所有状态进行重新赋值
            correctPoses();
            // 发布全局位姿odom和变换关系的odom(incremental),同时发布全局TF
            publishOdometry();
            // 发布相关的点云话题
            publishFrames();
        }
    }

    //! 根据IMU预积分里程计或者后端odom+IMU融合里程计的方式得到当前帧的初始姿态估计
    void updateInitialGuess(){
        // save current transformation before any processing
        // 格式转换，将上一时刻的全局位姿转为PCL矩阵形式，并保存
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        // static Eigen::Affine3f lastOdomTransformation; // 上一时刻根据IMU等数据推断的位姿，static变量使得变量在函数结束时不销毁，下次可继续访问
        // initialization
        // 初始化过程
        if (cloudKeyPoses3D->points.empty())
        {
            // 来自前端IMU数据的估计姿态
            transformTobeMapped[0] = 0.0;
            transformTobeMapped[1] = 0.0;
            transformTobeMapped[2] = 0.0;

            if (!useImuHeadingInitialization)   // 如果使用GPS数据，则优先使用GPS的yaw
                transformTobeMapped[2] = 0;
            // 存储初始的IMU数据，即0，0，0坐标点，RPY姿态
            // lastOdomTransformation = pcl::getTransformation(0, 0, 0, 0, 0, 0); // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false; // 直接使用IMU预估计的标志位
        static Eigen::Affine3f lastImuPreTransformation; // 上一时刻的IMU预积分里程计
        if (cloudInfo.odomAvailable == true)    // 如果IMU预积分得到了初始里程计估计
        {   // 将IMU预积分里程计转为eigen矩阵格式
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else { // 如果不直接用imu预积分的里程计，则根据imu里程计的逆变换得到变换矩阵，再对上一帧进行变换,得到当前的位姿估计
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack; // 求出相邻两帧的变换关系
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                // lastOdomTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                return;
            }
        }

        // // use imu incremental estimation for pose guess (only rotation)
        // // 使用IMU的数据更新初始位姿
        // if (cloudInfo.imuAvailable == true)
        // {
        //     Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            // Eigen::Affine3f transIncre = lastOdomTransformation.inverse() * transBack;

        //     Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        //     Eigen::Affine3f transFinal = transTobe * transIncre;
        //     pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
        //                                                   transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            // lastOdomTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        //     return;
        // }
    }

    //! 提取周围的关键帧，同时提取其角点和平面点
    void extractSurroundingKeyFrames()
    {
        // 如果之前没有提取的关键帧，就返回
        if (cloudKeyPoses3D->points.empty() == true)
            return;
        
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        extractNearby();  // 提取周围的关键帧，提取其角点和平面点
    }

    //! 提取一定范围内的关键帧，然后对关键帧进行降采样，提取其中的角点和平面点
    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());  // 所有近邻关键帧的点云集合
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());// 降采样后的近邻关键帧的点云集合
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        // 对最后一个点搜索最近邻的关键帧，并存储其3D位置坐标
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]); // 所有关键帧的点云放到同一个集合中
        }
        // 对近邻的所有关键帧点云进行体素滤波
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        // 设置点云的intensity，与降采样前的最近点的intensity相同
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            // 这里的intensity指的是点云的索引值
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        // 把10s内的关键帧也加入到surroundingKeyPosesDS中，以防止机器人原地旋转
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }
        // 提取关键帧中的角点和平面点
        extractCloud(surroundingKeyPosesDS);
    }

    //! 提取出输入的周围关键帧的中的角点和平面点
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        // 遍历最近的关键帧的每一个点
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            // 距离滤波
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity; // 取索引
            // 在全局所有关键帧中寻找这个关键帧的某个点
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // 找到了这个关键帧
                // transformed cloud available
                // 取周围关键帧的角点和平面点
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // 没找到就加入到这个全局关键帧中
                // transformed cloud not available
                // 坐标变换到全局坐标系下
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
            
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    //! 对当前帧的平面点和角点进行体素滤波（降采样）
    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }    

    //! 对周围的关键帧进行搜索，寻找能够有效匹配的点线约束和点面约束，并构建非线性问题，用高斯牛顿法进行求解全局位姿，最后与imu数据加权融合
    void scan2MapOptimization()
    {
        // 如果没有存储的关键帧就返回
        if (cloudKeyPoses3D->points.empty())
            return;
        // 如果当前帧的特征点足够，才可以开始后端优化
        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
            // scan-to-map迭代30次
            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();
                // 寻找能够有效匹配的点线关系，并保存
                cornerOptimization();
                // 寻找能够有效匹配的点面关系，并保存
                surfOptimization();
                // 提取当前帧能够有效匹配的特征点
                combineOptimizationCoeffs();
                // 使用高斯牛顿进行求解，直至算法收敛
                if (LMOptimization(iterCount) == true)
                    break;              
            }
            // 加权融合imu数据
            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    //! 更新位姿变换矩阵
    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    //! 从所有当前帧角点中，寻找有效的点线关系，并加入到点集中
    void cornerOptimization()
    {
        updatePointAssociateToMap();    // 更新变换矩阵

        #pragma omp parallel for num_threads(numberOfCores) // 多线程并发
        for (int i = 0; i < laserCloudCornerLastDSNum; i++) // 遍历所有降采样当前帧角点
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            // 坐标变换到全局map坐标系下
            pointAssociateToMap(&pointOri, &pointSel);
            // PCA算法判断线特征
            // 搜索最近的5个点
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0)); // 5个点与中心点的距离协方差矩阵
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0)); // cov矩阵的特征值
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0)); // cov矩阵的特征向量
                    
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5; // 5个点的中心点
                // 构造协方差矩阵
                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1); // 计算特征值和特征向量，此方法得到的特征值为降序排列

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) { // 说明符合一个线特征，开始点线ICP
                // 最大特征值所属的特征向量代表这条线的方向向量
                    // 构建三个点，待匹配的点A+线上的两点B
                    //        A
                    //   B        C
                    // A点坐标
                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    // B点坐标
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    // C点坐标
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);
                    // 求(AB X AC)的模，代表ABC三点构成的平行四边形面积
                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
                    // BC的模（BC边长）
                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
                    // (la,lb,lc）= (BC X (AB X AC))/a012/l12 代表A到BC的垂线方向的单位向量
                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12; // A到BC的垂直距离
                    // 计算权重因子，点到直线的距离越大，s越小，则越不能将它放入点云集合laserCloudOri以及coeffSel中
                    float s = 1 - 0.9 * fabs(ld2);
                    // 存储点线的相关参数
                    // coeff用于保存距离的方向向量
                    // intensity本质上构成了一个核函数，ld2越接近于1，增长越慢
                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    // s>0.1 也就是要求点到直线的距离ld2要小于1m
                    // s越大说明ld2越小(离边缘线越近)，这样就说明点pointOri在直线上
                    if (s > 0.1) {
                        // 加入到点集中
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    //! 从所有当前帧平面点中，寻找有效的点面关系，并加入到点集中
    void surfOptimization()
    {
        // 更新为位姿变换矩阵
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores) // 并发处理
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            // 坐标变换到全局坐标系下
            pointAssociateToMap(&pointOri, &pointSel); 
            // 取5个点进行平面拟合
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }
                // 求用QR分解，平面法向量matX0
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                // 单位化
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                // 检查平面是否合格，如果5个点中有点到平面的距离超过0.2m，那么认为这些点太分散了，不构成平面
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    // 待匹配的平面点到拟合平面的距离
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));
                    // coeff存储点到平面的法向量（带惩罚因子），intensity为点到平面的距离
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;
                    //如果s>0.1,代表伪距离<1，即点距离平面越近，越有效
                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    //! 提取出当前帧能够有效用于匹配的特征点
    void combineOptimizationCoeffs()
    {
        // 提取有效匹配角点
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // 提取有效匹配平面点
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        // Mask重置
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }
    //! 使用高斯牛顿法，对构建的非线性问题进行迭代求解
    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);
        // 如果能匹配的特征点过少，则返回，不进行优化
        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }
        // 矩阵A为误差对平移和旋转的雅可比矩阵
        // 矩阵B为残差项
        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0)); // 矩阵A
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));// A的转置
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));   // A的转置 X A
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0)); // 矩阵B
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));   // A的转置 X B
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0)); // 矩阵X

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            // 计算雅可比矩阵的每一个元素值
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity; // 负号是为了后面的GN算法
        }

        // 利用高斯牛顿法进行求解，
        // 高斯牛顿法的原型是J^(T)*J * delta(x) = -J*f(x)
        // J是雅克比矩阵，这里是A，J^(T)*J为近似的海森矩阵，f(x)是优化目标，这里是-B(符号在给B赋值时候就放进去了)
        // 通过QR分解的方式，求解matAtA*matX=matAtB，得到解matX
        cv::transpose(matA, matAt); // 得到A的转置
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {   // 如果是第一次优化，需要初始化，并判断退化，即约束中较小的偏移会导致解所在的局部区域发生较大的变化
            // 海森矩阵，在这里为matAtA
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0)); // 海森矩阵的特征值
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0)); // 海森矩阵的特征向量
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);  // 求近似海森矩阵的特征值和特征向量
            matV.copyTo(matV2);
            //TODO 这一部分待深究
            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }
        //TODO
        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }
        // 更新当前位姿 x = x + delta_x
        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);
        // 计算平移和旋转变化量
        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));
        // 如果变化量足够小，说明算法收敛，则终止优化过程
        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    //! 对后端优化位姿与imu预积分推断位姿的roll，pitch和z值进行加权融合，并限幅
    void transformUpdate()
    {
        // if (cloudInfo.imuAvailable == true)
        // {
        //     if (std::abs(cloudInfo.imuPitchInit) < 1.4) // imu预积分得到的pitch角不能太大，否则认为数据无效
        //     {
        //         // 对IMU预积分和后端优化后的roll，pitch角进行加权融合
        //         double imuWeight = imuRPYWeight;
        //         tf::Quaternion imuQuaternion;
        //         tf::Quaternion transformQuaternion;
        //         double rollMid, pitchMid, yawMid;

        //         // slerp roll 横滚角插值
        //         transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
        //         imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
        //         tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
        //         transformTobeMapped[0] = rollMid;

        //         // slerp pitch  俯仰角插值
        //         transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
        //         imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
        //         tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
        //         transformTobeMapped[1] = pitchMid;
        //     }
        // }
        // 对roll，pitch角和z坐标进行限幅
        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    //! 判断当前帧是否可以被保存为关键帧
    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty()) // 无关键帧则跳过
            return true;

        // if (sensor == SensorType::LIVOX)
        // {
        //     if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
        //         return true;
        // }

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());   // 上一个关键帧的位姿
        // 当前帧的优化位姿
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;   // 计算变换矩阵
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
        // 如果位姿变换过大，则不可以保存
        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    //! 添加因子（里程计，GPS，回环）并进行全局优化，然后将当前关键帧的状态最优估计保存起来，并保存其对应的特征点
    void saveKeyFramesAndFactor()
    {
        // 判断关键帧是否可以保存
        if (saveFrame() == false)
            return;

        // odom factor
        // 如果是初始则添加第一个先验因子，否则，添加k-1到k帧的里程计因子
        addOdomFactor();

        // // gps factor
        // addGPSFactor();

        // loop factor
        // // 添加回环因子
        // addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);  // 向ISAM中添加现有的因子（残差项），和状态值
        isam->update(); // 执行一次优化

        if (aLoopIsClosed == true) // 如果有回环帧
        {
            // 执行优化
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }
        // 清空因子容器
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();    // 得到当前时刻的最优估计位姿
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");
        // 保存当前时刻的状态最优估计到cloudKeyPoses3D
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);
        // 保存当前时刻的状态最优估计到cloudKeyPoses6D
        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        // 得到当前时刻的协方差矩阵
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        // 保存到全局位姿
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        // 保存当前帧中所对应的角点和平面点
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        // 更新path，可视化
        updatePath(thisPose6D);
    }

    //! 如果是初始则添加第一个先验因子，否则，添加k-1到k帧的里程计因子
    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty()) // 无关键帧，说明是系统初始，首先要添加一个当前位置的先验因子
        {
            // 定义噪声模型为Diagonal，对角线元素符合高斯白噪声
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            // 加入第一个位置先验因子
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            // 加入Value保存
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            // 添加一个从k-1帧到k帧的里程计因子
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            // 将当前位姿存入Value
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    //! 若发生回环，则更新历史所有关键帧状态位姿列表，若为回环，则无操作
    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true) // 发生回环后，才会校正历史位姿
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            // 清除当前轨迹
            globalPath.poses.clear();
            // update key poses
            // 遍历优化后的所有存储位姿，重新进行关键帧状态保存
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
                // 添加到Path中
                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false; // 重置回环标志位
        }
    }

    //! 添加当前的状态估计到Path中用于可视化
    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    // 发布全局位姿odom和变换关系的odom(incremental)
    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)   // 带incremental的是相邻两帧之间的变换关系的累乘
        static bool lastIncreOdomPubFlag = false;  // 初始化标志位
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            // 得到上一帧到当前帧的位姿变换关系
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);

            //发布位姿变换消息
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    //! 发布相关的点云话题
    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        // 发布所有的关键帧的状态点云
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        // 发布周围关键帧的平面点云
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            // 将角点和平面点都变换到全局坐标系下
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            // 发布当前帧的特征点云
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_projected, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            // 发布去畸变的点云
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        // 发布全局Path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        // static int lastSLAMInfoPubSize = -1;
        // if (pubSLAMInfo.getNumSubscribers() != 0)
        // {
        //     if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
        //     {
        //         lio_sam::cloud_info slamInfo;
        //         slamInfo.header.stamp = timeLaserInfoStamp;
        //         pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        //         *cloudOut += *laserCloudCornerLastDS;
        //         *cloudOut += *laserCloudSurfLastDS;
        //         slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
        //         slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
        //         pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
        //         *localMapOut += *laserCloudCornerFromMapDS;
        //         *localMapOut += *laserCloudSurfFromMapDS;
        //         slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
        //         pubSLAMInfo.publish(slamInfo);
        //         lastSLAMInfoPubSize = cloudKeyPoses6D->size();
        //     }
        // }
    }

    //! 全局点云地图可视化线程
    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            // 可视化全局地图
            publishGlobalMap();
        }

        if (savePCD == false)
            return;

        // lio_sam::save_mapRequest  req;
        // lio_sam::save_mapResponse res;

        // if(!saveMapService(req, res)){
        //     cout << "Fail to save map" << endl;
        // }
    }

    //! 对关键帧状态降采样，并融合特征点云组成全局地图，最后发布ROS消息
    void publishGlobalMap()
    {
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        // 只显示固定范围内的Global Pose
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        // 对关键帧状态进行降采样
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        // 找到关键帧状态对应的特征点云
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            // 变换到全局坐标系下
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points 降采样
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }

    //! 回环检测线程，寻找当前帧周围的潜在回环，构建回环对，并求回环对之间的位姿变换矩阵
    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            // 寻找周围历史帧，建立回环对，利用ICP算法计算当前帧与历史帧的变换矩阵，并保存回环边
            performLoopClosure();
            // 可视化
            visualizeLoopClosure();
        }
    }

    //! 寻找周围历史帧，建立回环对，利用ICP算法计算当前帧与历史帧的变换矩阵，并保存回环边
    void performLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        // 保存一个副本
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        //* 此处未用到外部传入的回环对
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false) // 验证外部传进来的回环对是否有效
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)   // 找到周围时间间隔最长的历史帧，建立回环对
                return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);    // 代表取当前帧的特征点集
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum); // 寻找周围历史帧，并保存特征点集
            // 如果特征点集数量不够，则返回
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        // ICP算法参数设置
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0); // 不是用ransac算法进行离群点剔除

        // Align clouds 对齐点云，最终得到的变换矩阵是: source -> target
        icp.setInputSource(cureKeyframeCloud); // source点云为当前帧特征点
        icp.setInputTarget(prevKeyframeCloud); // target点云为历史帧特征点
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return; // 说明对齐效果不佳，不考虑这个回环

        // publish corrected cloud
        // 发布对齐后的点云
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            // 获得与历史帧对齐后的点云
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        // 得到source -> target 的变换矩阵
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        // 当前帧的位姿
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        // 经过变换矩阵，校正后的位姿
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        // 将当前帧变换校正后的位姿作为From，原来的历史帧作为To，实质这两个应该是表示的同一个场景，但因为现实中有误差，所以可以作为一个残差
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();// 适应度函数为各点到目标点的平均距离，以此作为噪声项
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        // 保存当前的匹配的回环对，用于可视化
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }
    //! 根据当前帧的位置，查询周围的历史帧，并保存时间间隔最长的历史帧，建立回环对
    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        // 找到周围一定范围内的历史帧
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) // 优先和时间间隔较长的历史帧匹配
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;
        // 输出
        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }
    //! 利用外部传入的回环对，查询当前帧和回环帧的时间，判断此回环对是否有效
    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();
        // 历史帧要比当前帧慢30s
        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)  // 历史关键帧太少，不考虑回环
            return false;

        // latest key
        // 从关键帧中，根据时间戳，找到对应的关键帧索引
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur); // 在回环对容器中查询
        if (it != loopIndexContainer.end()) // 如果当前帧已经建立回环对，则不再建立新的回环对
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }
    //! 根据给定的帧序列号和最大筛选帧数，找到满足数量的历史帧，并保存到nearKeyframes中，然后进行降采样
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )   // 边界检测
                continue;
            // 将周围的历史帧所对应的特征点变换到全局坐标系下，并加入到同一个集合中
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty()) // 周围没有历史帧
            return;

        // downsample near keyframes
        // 对周围历史帧的特征点降采样
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }
    //! 可视化回环边
    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;
        
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rolo");
    // 实例化后端优化类
    backMapping BM;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    std::thread loopthread(&backMapping::loopClosureThread, &BM);
    std::thread visualizeMapThread(&backMapping::visualizeGlobalMapThread, &BM);

    ros::spin();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}