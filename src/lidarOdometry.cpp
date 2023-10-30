#include "rolo_sam/utility.h"
#include "rolo_sam/undirected_graph.h"
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
#include <teaser/matcher.h>
#include <teaser/registration.h>

using namespace Eigen;

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
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("rolo_sam/mapping/odometry", 5, &TransformFusion::mappingOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::lidarOdometryHandler,   this, ros::TransportHints().tcpNoDelay());
        // 发布IMU里程计
        pubLidarOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubLidarPath       = nh.advertise<nav_msgs::Path>("rolo_sam/lidar_odometry/path", 1);
    }
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
            // IMU里程和激光里程进行时间标定
            if (lidarOdomQueue.front().header.stamp.toSec() <= mappingOdomTime)
                lidarOdomQueue.pop_front();
            else
                break;
        }
        Eigen::Affine3f lidarOdomAffineFront = odom2affine(lidarOdomQueue.front());
        Eigen::Affine3f lidarOdomAffineBack = odom2affine(lidarOdomQueue.back());
        // 求前后两帧lidar里程计位姿的变换关系
        Eigen::Affine3f lidarOdomAffineIncre = lidarOdomAffineFront.inverse() * lidarOdomAffineBack;
        // 对最新的激光里程位姿进行相应变换，结合imu积分推断得到当前帧的激光里程估计
        Eigen::Affine3f imuOdomAffineLast = mappingOdomAffine * lidarOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
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
    double  cloudTimeCur;
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
    Matrix<double, 3, Dynamic> MatCloudInfoLast;
    Matrix<int, 2, Dynamic> TI_map_last;
    rolo_sam::CloudInfoStamp laserCloudInfoLast;
    pcl::PointCloud<PointType>::Ptr FullCloudLast;
    pcl::PointCloud<PointType>::Ptr CloudCornerLast;
    pcl::PointCloud<PointType>::Ptr CloudSurfLast;
    pcl::PointCloud<PointType>::Ptr CloudGroundLast;
    pcl::PointCloud<PointType>::Ptr ground_and_cornerLast;
    pcl::PointCloud<PointType>::Ptr featureLast;
    FPFHFeature::Ptr fpfhFeatureLast;

    // 上一帧数据
    Matrix<double, 3, Dynamic> MatCloudInfoOld;
    Matrix<int, 2, Dynamic> TI_map_old;
    rolo_sam::CloudInfoStamp laserCloudInfoOld;
    pcl::PointCloud<PointType>::Ptr FullCloudOld;
    pcl::PointCloud<PointType>::Ptr CloudCornerOld;
    pcl::PointCloud<PointType>::Ptr CloudSurfOld;
    pcl::PointCloud<PointType>::Ptr CloudGroundOld;
    pcl::PointCloud<PointType>::Ptr ground_and_cornerOld;
    pcl::PointCloud<PointType>::Ptr featureOld;
    FPFHFeature::Ptr fpfhFeatureOld;
    FLANNKDTree *fpfhTree;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeOldCloud;
    std::queue<rolo_sam::CloudInfoStamp> laserCloudInfoBuf;

    Matrix<bool, 1, Dynamic> inliers1, inliers2;
    Matrix<double, 3, Dynamic> InliersLast, InliersOld;
    
    Matrix3d Rotation;
    Vector3d Translation;
    float LaserOdomPose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // [x, y, z, roll, pitch, yaw]


public:  
    LidarOdometry():
    doneFirstOpt(true),
    isFirstFrame(true),
    failureFrameFlag(false)
    {
        // 接受imu原始数据
        subCloudInfo = nh.subscribe<rolo_sam::CloudInfoStamp>("rolo_sam/feature/cloud_info", 10, &LidarOdometry::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        // mapOptimization传来的里程计数据
        // subOdometryMapped = nh.subscribe<nav_msgs::Odometry>("rolo_sam/mapping/odometry_incremental", 5, &LidarOdometry::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        // 发布imu预测里程计
        pubFrontCloudInfo = nh.advertise<rolo_sam::CloudInfoStamp>(odomTopic+"/cloud_info", 2000);
        pubLidarOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);
        pubLidarPose = nh.advertise<geometry_msgs::PoseStamped> (odomTopic+"_incremental/pose", 2000);
        pubLaserPath = nh.advertise<nav_msgs::Path> (odomTopic+"_incremental/path", 2000);
        pubRegScan = nh.advertise<sensor_msgs::PointCloud2> (odomTopic+"/registration_scan", 10);
        pubPlotData = nh.advertise<std_msgs::Float64MultiArray> ("rolo_sam/data_test", 10);
        Init();
    }
    ~LidarOdometry(){}

    void Init(){
        Rotation = Matrix3d::Identity();
        Translation = Vector3d::Zero();
        RegCloud.reset(new pcl::PointCloud<PointType>());
        kdtreeOldCloud.reset(new pcl::KdTreeFLANN<PointType>());
        // 当前帧数据
        FullCloudLast.reset(new pcl::PointCloud<PointType>());
        CloudCornerLast.reset(new pcl::PointCloud<PointType>());
        CloudSurfLast.reset(new pcl::PointCloud<PointType>());
        CloudGroundLast.reset(new pcl::PointCloud<PointType>());
        ground_and_cornerLast.reset(new pcl::PointCloud<PointType>());
        featureLast.reset(new pcl::PointCloud<PointType>());
        // fpfhFeatureLast.reset(new FPFHFeature());
        fpfhFeatureLast = NULL;

        // 上一帧数据
        FullCloudOld.reset(new pcl::PointCloud<PointType>());
        CloudCornerOld.reset(new pcl::PointCloud<PointType>());
        CloudSurfOld.reset(new pcl::PointCloud<PointType>());
        CloudGroundOld.reset(new pcl::PointCloud<PointType>());
        ground_and_cornerOld.reset(new pcl::PointCloud<PointType>());
        featureOld.reset(new pcl::PointCloud<PointType>());
        // fpfhFeatureOld.reset(new FPFHFeature());
        fpfhFeatureOld = NULL;
        fpfhTree = new FLANNKDTree(flann::KDTreeSingleIndexParams(15));
        // LaserOdomPose = {0,0,0,0,0,0};
        start_time = std::chrono::system_clock::now();

    }

    //! 接受后端的里程计消息，并与前端里程计融合
    void odometryHandler(const nav_msgs::OdometryConstPtr &mappedOdom){
        std::cout << "FUCK FUCK!!!!!!" << std::endl;
        std::lock_guard<std::mutex> lock(mtx);
        // 当前时刻odom时间
        double currentCorrectionTime = mappedOdom->header.stamp.toSec();
        nav_msgs::Odometry mappedOdom_ = *mappedOdom;
        // 时间标定
        if(currentCorrectionTime - cloudTimeCur < 0.2){
            // 将后端位姿传给前端
            mtx.lock();
            LaserOdomPose[0] = mappedOdom_.pose.pose.position.x;
            LaserOdomPose[1] = mappedOdom_.pose.pose.position.y;
            LaserOdomPose[2] = mappedOdom_.pose.pose.position.z;
            double roll, pitch, yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(mappedOdom_.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            LaserOdomPose[3] = roll;
            LaserOdomPose[4] = pitch;
            LaserOdomPose[5] = yaw;
            mtx.unlock();
        }

    }

    void scanRegeistration(){
        // 点云预处理
        // cloudPreprocessing<double>(featureLast, featureOld, MatCloudInfoLast, MatCloudInfoOld);
        
        // Convert to teaser point cloud
        teaser::PointCloud src_cloud;
        // voxel_filter(featureOld, featureOld, 0.8);
        pcd_to_teaser(featureOld, src_cloud);

        teaser::PointCloud tgt_cloud;
        // voxel_filter(featureLast, featureLast, 0.8);
        pcd_to_teaser(featureLast, tgt_cloud);
        // 计算FPFH特征
        fpfhFeatureLast = computeFPFH(featureLast);
        if(fpfhFeatureOld == NULL){
            fpfhFeatureOld  = computeFPFH(featureOld);
        }
        // Compute correspondences
        teaser::Matcher matcher;
        auto correspondences = matcher.calculateCorrespondences(
            src_cloud, tgt_cloud, *fpfhFeatureOld, *fpfhFeatureLast, true, false, false, 0.95);
        std::cout << "correspondences.size()" << correspondences.size() << std::endl;
        // TEASER预测
        // Prepare solver parameters
        teaser::RobustRegistrationSolver::Params params;
        params.noise_bound = lidarNoiseBound;
        params.cbar2 = 1;
        params.estimate_scaling = false;
        params.rotation_max_iterations = maxOptIteration;
        params.rotation_gnc_factor = costFactor;
        params.rotation_estimation_algorithm =
            // teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::FGR;
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
        params.rotation_cost_threshold = costDiffThre;

        // Solve with TEASER++
        teaser::RobustRegistrationSolver solver(params);
        auto start = std::chrono::system_clock::now();
        solver.solve(src_cloud, tgt_cloud, correspondences);
        teaser::RegistrationSolution solution = solver.getSolution();
        Rotation = solution.rotation;
        Translation = solution.translation;
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        printf("Solver Duration: %f ms.\n" ,elapsed_seconds.count() * 1000);

        // 计算平移不变性
        // Matrix<double, 3, Dynamic> src_diff, dst_diff, pruned_src_diff, pruned_dst_diff;
        // src_diff = computeTIS(MatCloudInfoOld, TI_map_old);
        // dst_diff = computeTIS(MatCloudInfoLast, TI_map_last);
        // 计算内点
        // computeInliers(src_diff, dst_diff, &inliers1);
        // 剔除外点
        // removeOutliers(TI_map_old, inliers1, src_diff, dst_diff, pruned_src_diff, pruned_dst_diff);

        // 计算旋转
        // rotationOptimization(pruned_src_diff, pruned_dst_diff);
        // 计算平移
        // translationOptimization();
    }

    // 最近邻搜索
    // template <class T>
    // void cloudPreprocessing(const pcl::PointCloud<PointType>::Ptr &cloudLast, const pcl::PointCloud<PointType>::Ptr &cloudOld,
    //                         Matrix<T, 3, Dynamic> &last_mat, Matrix<T, 3, Dynamic> &old_mat){
    //     auto LastCloudSize = cloudLast->size();
    //     auto OldCloudSize = cloudOld->size();
    //     Matrix<T, 3, Dynamic> LastCloudMatrix;
    //     Matrix<T, 3, Dynamic> OldCloudMatrix;
    //     LastCloudMatrix.resize(3, LastCloudSize);
    //     OldCloudMatrix.resize(3, LastCloudSize);
    //     int column = 0;
    //     std::vector<bool> oldCloudSelected(OldCloudSize, false);
    //     std::vector<int> pointInd;
    //     std::vector<float> pointDis;
    //     // 设置kdtree
    //     kdtreeOldCloud->setInputCloud(cloudOld);
    //     for(int i=0; i<LastCloudSize; i++){
    //         // 搜索最近点
    //         int searchedNum = kdtreeOldCloud->radiusSearch(cloudLast->points[i], (double)nearestSearchRadius, pointInd, pointDis, 0);
    //         if(searchedNum != 0){
    //             int nearestInd = pointInd[0];
    //             if(!oldCloudSelected[nearestInd]){
    //                 LastCloudMatrix.col(column) << cloudLast->points[i].x, cloudLast->points[i].y, cloudLast->points[i].z;
    //                 OldCloudMatrix.col(column) << cloudOld->points[nearestInd].x, cloudOld->points[nearestInd].y, cloudOld->points[nearestInd].z;
    //                 column++;
    //                 oldCloudSelected[nearestInd] = true;
    //             }
    //         }
    //     }
    //     last_mat.resize(3, column);
    //     old_mat.resize(3, column);
    //     last_mat.leftCols(column) = LastCloudMatrix.leftCols(column);
    //     old_mat.leftCols(column) = OldCloudMatrix.leftCols(column);
    // }

    //! 点云转矩阵
    template <class T>
    void cloudPreprocessing(pcl::PointCloud<PointType>::Ptr cloudLast, pcl::PointCloud<PointType>::Ptr cloudOld,
                                             Matrix<T, 3, Dynamic> &last_mat, Matrix<T, 3, Dynamic> &old_mat) {
        // Convert to teaser point cloud
        teaser::PointCloud src_cloud;
        pcd_to_teaser(cloudOld, src_cloud);
        teaser::PointCloud tgt_cloud;
        pcd_to_teaser(cloudLast, tgt_cloud);
        // 计算FPFH特征
        fpfhFeatureLast = computeFPFH(cloudLast);
        if(fpfhFeatureOld == NULL){
            fpfhFeatureOld  = computeFPFH(cloudOld);
        }
        // Compute correspondences
        teaser::Matcher matcher;
        auto correspondences = matcher.calculateCorrespondences(
            src_cloud, tgt_cloud, *fpfhFeatureOld, *fpfhFeatureLast, true, false, false, 0.95);
        std::cout << "correspondences.size()" << correspondences.size() << std::endl;
        // 得到点云矩阵
        MatCloudInfoLast.resize(3, correspondences.size());
        MatCloudInfoOld.resize(3, correspondences.size());
        for (size_t i = 0; i < correspondences.size(); ++i) {
            auto src_idx = std::get<0>(correspondences[i]);
            auto dst_idx = std::get<1>(correspondences[i]);
            MatCloudInfoOld.col(i) << src_cloud[src_idx].x, src_cloud[src_idx].y, src_cloud[src_idx].z;
            MatCloudInfoLast.col(i) << tgt_cloud[dst_idx].x, tgt_cloud[dst_idx].y, tgt_cloud[dst_idx].z;
        }
    }

    FPFHFeature::Ptr computeFPFH(const pcl::PointCloud<PointType>::Ptr &Cloud){
        // 计算法线
        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        //------------------FPFH估计-------------------------------
        FPFHFeature::Ptr fpfh(new FPFHFeature());
        pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> f;

        //-------------------------法向量估计-----------------------
        pcl::NormalEstimationOMP<PointType, pcl::Normal> n;
        n.setInputCloud(Cloud);
        n.setNumberOfThreads(numberOfCores); //设置openMP的线程数
        n.setSearchMethod(tree);
        n.setKSearch(10);
        n.compute(*normals);

        f.setNumberOfThreads(numberOfCores); // 指定8核计算
        f.setInputCloud(Cloud);
        f.setInputNormals(normals);
        f.setSearchMethod(tree);
        f.setKSearch(10);
        f.compute(*fpfh);
        return fpfh;
    } 

    // 计算平移不变性，输入为点云对应的坐标矩阵，输出：TI_map：每一列存储相连的索引，v_diff：每一列存储的相连坐标值的差
    Matrix<double, 3, Dynamic> computeTIS(const Matrix<double, 3, Dynamic> &matIn,
                                  Matrix<int, 2, Dynamic> &TI_map){

        auto N = matIn.cols();  // 点的个数
        Eigen::Matrix<double, 3, Eigen::Dynamic> v_diff(3, N * (N - 1) / 2);
        TI_map.resize(2, N * (N - 1) / 2);
        #pragma omp parallel for default(none) shared(N, matIn, v_diff, TI_map)
        for (size_t i = 0; i < N - 1; i++) {
            // For each measurement, we compute the TIMs between itself and all the measurements after it.
            size_t start_idx = i * N - i * (i + 1) / 2;
            size_t cols_num = N - 1 - i;

            // calculate TI_map
            Eigen::Matrix<double, 3, 1> m = matIn.col(i); // 每一列是一个点的3d坐标
            Eigen::Matrix<double, 3, Eigen::Dynamic> temp = matIn - m * Eigen::MatrixXd::Ones(1, N);

            // concatenate to the end of the tilde vector
            v_diff.middleCols(start_idx, cols_num) = temp.rightCols(cols_num);

            // save the index map
            Eigen::Matrix<int, 2, Eigen::Dynamic> map_addition(2, N);
            for (size_t j = 0; j < N; ++j) {
                map_addition(0, j) = i;
                map_addition(1, j) = j;
            }
            TI_map.middleCols(start_idx, cols_num) = map_addition.rightCols(cols_num);
        }

        return v_diff;
    }
    //! 根据tis计算inliers
    void computeInliers(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                       const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst,
        Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers){

        Eigen::Matrix<double, 1, Eigen::Dynamic> v1_dist =
            src.array().square().colwise().sum().array().sqrt();
        Eigen::Matrix<double, 1, Eigen::Dynamic> v2_dist =
            dst.array().square().colwise().sum().array().sqrt();

        // A pair-wise correspondence is an inlier if it passes the following test:
        // abs(|dst| - |src|) is within maximum allowed bound
        *inliers = (v1_dist.array() - v2_dist.array()).array().abs() <= inlierDiffBound;
    }

    //! 通过最大团的方式删除外点
    void removeOutliers(const Matrix<int, 2, Dynamic> &TI_map, 
                        const Matrix<bool, 1, Dynamic> &inlier_mask, 
                        const Matrix<double, 3, Dynamic> &src_diff,
                        const Matrix<double, 3, Dynamic> &dst_diff,
                        Matrix<double, 3, Dynamic> &src_diff_pruned,
                        Matrix<double, 3, Dynamic> &dst_diff_pruned
                        ){
        // 构建图
        UndirectedGraph inner_graph(MatCloudInfoLast, TI_map, inlier_mask);

        std::vector<int> max_clique;
        max_clique = inner_graph.findMaxClique();
        std::cout << "findMaxClique finished!! " << std::endl;
        std::cout << "max_clique.size: " << max_clique.size() << std::endl;
        // std::copy(max_clique.begin(), max_clique.end(), std::ostream_iterator<int>(std::cout, " "));
        std::cout << std::endl;
        if(max_clique.size() <= minOptConstrains){
            // 最大团节点过少，随机抽取固定数量点作为旋转约束
            int ptSize = src_diff.cols();
            src_diff_pruned.resize(3, maxOptContrains);
            dst_diff_pruned.resize(3, maxOptContrains);
            InliersOld.resize(3, maxOptContrains);
            InliersLast.resize(3, maxOptContrains);
            MatrixXd mask = MatrixXd::Random(1, 2*maxOptContrains);
            Matrix<int, 1, Dynamic> pt_selected = MatrixXi::Zero(1, ptSize); 
            int pt_count = 0, i=0;
            while(pt_count < maxOptContrains && i < 2*maxOptContrains){
                int ind = int((mask(1,i)+1)/2 * ptSize);
                if(pt_selected(0, ind)!=0){
                    src_diff_pruned.col(pt_count) = src_diff.col(ind);
                    dst_diff_pruned.col(pt_count) = dst_diff.col(ind);
                    InliersOld.col(pt_count) = MatCloudInfoOld.col(ind);
                    InliersLast.col(pt_count) = MatCloudInfoLast.col(ind);   
                    pt_count++;
                }
                i++;
            }
            return;
        }
        // 删除外点约束
        Matrix<double, 3, Dynamic> temp_diff1, temp_diff2;
        src_diff_pruned.resize(3, max_clique.size());
        dst_diff_pruned.resize(3, max_clique.size());
        InliersOld.resize(3, max_clique.size());
        InliersLast.resize(3, max_clique.size());

        std::sort(max_clique.begin(), max_clique.end());
        for(int i=0; i<max_clique.size(); i++){
            int j = i+1;
            if(i == max_clique.size()-1)
                j = 0;
            src_diff_pruned.col(i) = MatCloudInfoOld.col(max_clique[j]) - MatCloudInfoOld.col(max_clique[i]);
            dst_diff_pruned.col(i) = MatCloudInfoLast.col(max_clique[j]) - MatCloudInfoLast.col(max_clique[i]);            
            InliersOld.col(i) = MatCloudInfoOld.col(max_clique[i]);
            InliersLast.col(i) = MatCloudInfoLast.col(max_clique[i]);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               }
    }

    void rotationOptimization(Matrix<double, 3, Dynamic> &src_diff,
                              Matrix<double, 3, Dynamic> &dst_diff){
        std_msgs::Float64MultiArray plot_data;
        /**
        Loop: terminate when:
            * 1. cost_diff less than a threshold
            * 2. iterations exceeds the maximum iteration
        
        Using GNC&TLS solve the R and W, total two step:
            * 1. fix weights and solve for R
            * 2. fix R and solve for weights
        **/
        // Prepare some variables
        auto corresponse_size = src_diff.cols();
        
        double mu = 1;  // arbitrary number
        
        double prev_cost = std::numeric_limits<double>::infinity();
        double last_cost = 0;
        double sq_noise_bound = std::pow(lidarNoiseBound, 2);

        Matrix<double, 3, Dynamic> residuals(3, corresponse_size);
        Matrix<double, 1, Dynamic> weights(1, corresponse_size);
        inliers2.resize(1, corresponse_size);
        weights.setOnes(1, corresponse_size);
        Matrix<double, 1, Dynamic> sq_residuals(1, corresponse_size);
        // Loop for performing rotation optimization
        for (int i = 0; i < maxOptIteration; i++) {
            
            // Fix weights and perform SVD rotation estimation
            Rotation = svdSolver(src_diff, dst_diff, weights);

            // Calculate residuals squared
            // std::cout << "src_diff: " << src_diff << std::endl;
            // std::cout << "dst_diff: " << dst_diff << std::endl;

            residuals = (dst_diff - (Rotation) * src_diff).array().square();
            sq_residuals = residuals.colwise().sum();
            std::cout << "sq_residual max: " << sq_residuals.maxCoeff() << std::endl;
            // plot_data.data.push_back(sq_residuals.maxCoeff());
            if (i == 0) {
                // Initialize rule for mu
                double max_residual = sq_residuals.maxCoeff();

                mu = 1 / (2 * max_residual / sq_noise_bound - 1);
                // Degenerate case: mu = -1 because max_residual is very small
                // i.e., little to none noise
                if (mu <= 0) {
                    printf("Loop terminated because maximum residual at initialization is very small.\n");
                    break;
                }
            }

            // Fix R and solve for weights in closed form
            double th1 = (mu + 1) / mu * sq_noise_bound;
            double th2 = mu / (mu + 1) * sq_noise_bound;

            for (size_t j = 0; j < corresponse_size; ++j) {
                // Also calculate cost in this loop
                // Note: the cost calculated is using the previously solved weights
                last_cost += weights(j) * sq_residuals(j);

                if (sq_residuals(j) >= th1) {
                    weights(j) = 0;
                } else if (sq_residuals(j) <= th2) {
                    weights(j) = 1;
                } else {
                    weights(j) = sqrt(sq_noise_bound * mu * (mu + 1) / sq_residuals(j)) - mu;
                    assert(weights(j) >= 0 && weights(j) <= 1);
                }
            }

            // Calculate cost
            double cost_diff = std::abs(last_cost - prev_cost);
            plot_data.data.push_back(cost_diff);
            // std::cout << "cost_diff: " << cost_diff << std::endl;

            // Increase mu
            mu = mu * costFactor;
            prev_cost = last_cost;

            if (cost_diff < costDiffThre) {
                printf("GNC-TLS solver terminated due to cost convergence. \n");
                printf("Cost diff: %f \n", cost_diff);
                printf("Iterations: %d \n", i);
                break;
            }
        }
        pubPlotData.publish(plot_data);
        // 筛选内点
        inliers2 = weights.array() >= 0.5;
        int count = 0;
        for(int it=0;it<inliers2.cols();it++){
            if(inliers2(0, it)){
                count++;
            }
        }
        // std::cout << "final inliers num: " << count << std::endl;
    }

    Matrix3d svdSolver(const Matrix<double, 3, Dynamic>& X,
                              const Matrix<double, 3, Dynamic>& Y,
                              const Matrix<double, 1, Dynamic>& W) {
        // Assemble the correlation matrix H = X * Y'
        Matrix3d H = X * W.asDiagonal() * Y.transpose();

        JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
        Matrix3d U = svd.matrixU();
        Matrix3d V = svd.matrixV();

        if (U.determinant() * V.determinant() < 0) {
            V.col(2) *= -1;
        }

        return V * U.transpose();
    }

    void translationOptimization(){
        std::cout << "------------------------" << std::endl;
        std_msgs::Float64MultiArray plot_data;
        Translation = Vector3d::Zero();
        auto inliersSize = MatCloudInfoLast.cols();
        Matrix<double, 3, 1> last_resual = Matrix3Xd::Zero(3,1);
        Matrix<double, 3, 3> E = Matrix3d::Zero();
        Matrix<double, 3, 3> U = Matrix3d::Zero();
        Eigen::Vector3d Delt = Vector3d::Zero();
        // Matrix<double, 3, Dynamic> translation_diff = InliersLast - Rotation*InliersOld;
        Matrix<double, 3, Dynamic> translation_diff = MatCloudInfoLast - Rotation*MatCloudInfoOld;

        // 迭代优化
        for(int j=0; j < maxOptIteration; j++){
            // 计算残差
            // auto res_max = translation_diff.rowwise().maxCoeff();
            // auto res_min = translation_diff.rowwise().minCoeff();
            // std::cout << "res_max: " << res_max << std::endl;
            // std::cout << "res_min: " << res_min << std::endl;
            // Matrix<double, 3, 1> resual = res_max.array() * res_min.array();
            // // resual为列向量
            // for(int i=0; i<3; i++){
            //     if(fabs(res_max(i,0)) < fabs(res_min(i,0))){
            //         resual(i,0) = res_min(i,0);
            //     }
            //     else{
            //         resual(i,0) = res_max(i,0);
            //     }
            // }
             auto resual = translation_diff.rowwise().mean();
            // std::cout << "resual: " << resual << std::endl;
            // 计算补偿量
            // 计算矩阵E
            std::cout << "resual: " << std::endl << resual << std::endl;
            E.row(0) = resual.transpose();  // P
            E.row(1) += resual.transpose(); // I
            E.row(2) = resual.transpose() - last_resual.transpose(); // D
            if(j==0){
                E.row(2) = MatrixXd::Zero(1,3);
            }
            // 计算U(t)
            U = KScalarMat.array() * E.array();
            auto deltU = U.colwise().sum().transpose(); // 3X1
            Delt += deltU;
            // 更新残差和转移矩阵
            translation_diff -= Delt * Eigen::MatrixXd::Ones(1, inliersSize);
            Translation += Delt;
            // 判断是否收敛
            double total_res = resual.array().square().sum();
            double resual_diff = (resual-last_resual).array().square().sum();
            double deltT = deltU.array().square().sum();
            std::cout << "total_res: " << total_res << std::endl;
            std::cout << "resual_diff: " << resual_diff << std::endl;
            std::cout << "deltU: " << deltU.transpose() << std::endl;
            std::cout << "deltT: " << deltT << std::endl;

            plot_data.data.push_back(total_res);

            if(deltT < 0.01){
                printf("Translation optimization finished.\n");
                printf("The minimal resual: %f\n", total_res);
                printf("The resual_diff: %f\n", resual_diff);
                printf("Total iteration: %d\n", j+1);
                break;
            }
            last_resual = resual;
        }
        pubPlotData.publish(plot_data);
    }
        

    void cloudHandler(const rolo_sam::CloudInfoStampConstPtr &cloudIn){
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
        // pcl::fromROSMsg(laserCloudInfoLast.extracted_ground, *CloudGroundLast);
        pcl::fromROSMsg(laserCloudInfoLast.cloud_projected, *FullCloudLast);
        // std::cout << "corner size: " << CloudCornerLast->size() << std::endl;
        // std::cout << "surf size: " << CloudSurfLast->size() << std::endl;
        // std::cout << "ground size: " << CloudGroundLast->size() << std::endl;
        // std::cout << "full cloud size: " << FullCloudLast->size() << std::endl;
        *featureLast = *CloudCornerLast + *CloudSurfLast;
        *ground_and_cornerLast = *CloudCornerLast+*CloudGroundLast;

        if(isFirstFrame){
            isFirstFrame = false;
            *FullCloudOld = *FullCloudLast;
            *CloudCornerOld = *CloudCornerLast;
            *CloudSurfOld = *CloudSurfLast;
            *CloudGroundOld = *CloudGroundLast;
            *featureOld = *featureLast;
            *ground_and_cornerOld = *ground_and_cornerLast;
            return;
        }

        // 是否完成第一次全图优化
        if (doneFirstOpt == false)
            return;
        scanRegeistration();
        // std::cout << Rotation << std::endl;
        std::cout << "Translation: " << std::endl <<  Translation << std::endl;
        updateTransform();

        if(!failureFrameFlag){
            // 发布ROS消息和TF
            pubMessage();
            // pubTranform();
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
        // trans << Translation;
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
        Affine3f transformed_pose = transform_affine * transformStep.inverse();
        
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        if(!failureDetection(transform_affine, transformed_pose, elapsed_seconds.count()*1.0e6)){
            failureFrameFlag = true;
            return;
        }
        start_time = end_time;
        Vector3f rotation_euler;
        float x, y, z;
        pcl::getTranslationAndEulerAngles<float>(transformStep.inverse(), 
                                                 x, y, z,
                                                 rotation_euler[0],
                                                 rotation_euler[1], 
                                                 rotation_euler[2]); 
        std::cout << "rotation angles: " << std::endl << rotation_euler*180/M_PI << std::endl;

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
        *CloudGroundOld = *CloudGroundLast;
        *featureOld = *featureLast;
        *ground_and_cornerOld = *ground_and_cornerLast;
        *fpfhFeatureOld = *fpfhFeatureLast;
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
    
    // void pubTranform(){
    //     // 发布TF
    //     // Publish TF
    //     static tf::TransformBroadcaster br;
    //     tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(LaserOdomPose[3], LaserOdomPose[4], LaserOdomPose[5]),
    //                                                   tf::Vector3(LaserOdomPose[0], LaserOdomPose[1], LaserOdomPose[2]));
    //     tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, cloudTimeStamp, odometryFrame, baselinkFrame);
    //     br.sendTransform(trans_odom_to_lidar);
    // }

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
        laser_odom_incremental.header.stamp = ros::Time::now();
        laser_odom_incremental.child_frame_id = "lidar_odometry";
        laser_odom_incremental.pose.pose = laser_pose.pose;
        pubLidarOdometry.publish(laser_odom_incremental);

        // 发布初始位姿估计
        rolo_sam::CloudInfoStamp odometry_cloud;
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

    void pcd_to_teaser(pcl::PointCloud<PointType>::Ptr &input_cloud, teaser::PointCloud &output_cloud) {
        for (size_t i = 0; i < input_cloud->points.size(); ++i) {
            output_cloud.push_back(
                {input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z});
        }
    }

    void voxel_filter(pcl::PointCloud<PointType>::Ptr &cloud, pcl::PointCloud<PointType>::Ptr &cloud_filtered,
                    float leaf_size = 1.0f) {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel.setInputCloud(cloud);
        voxel.filter(*cloud_filtered);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rolo_sam");
    
    LidarOdometry LO;
    TransformFusion TF;
    
    ROS_INFO("\033[1;32m----> Laser Odometry Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}