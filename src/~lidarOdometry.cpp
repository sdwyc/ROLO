#include "rolo/utility.h"
#include "rolo/undirected_graph.h"
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

using namespace Eigen;

class LidarOdometry : public ParamLoader
{
private:
    mutex mtx;
    bool doneFirstOpt;
    bool isFirstFrame;
    ros::Time cloudTimeStamp;
    double  cloudTimeCur;
    // ROS wrraper
    ros::Subscriber subCloudInfo;
    //TODO 接受后端的优化位姿
    // ros::Subscriber subOdometryMapped;
    ros::Publisher pubLidarOdometry;
    ros::Publisher pubLaserPath;
    ros::Publisher pubRegScan;
    ros::Publisher pubPlotData;
    
    nav_msgs::Path path;
    nav_msgs::Odometry laser_odom_incremental;
    pcl::PointCloud<PointType>::Ptr RegCloud;
    
    // 当前帧数据
    Matrix<double, 3, Dynamic> MatCloudInfoLast;
    Matrix<int, 2, Dynamic> TI_map_last;
    rolo::CloudInfoStamp laserCloudInfoLast;
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
    rolo::CloudInfoStamp laserCloudInfoOld;
    pcl::PointCloud<PointType>::Ptr FullCloudOld;
    pcl::PointCloud<PointType>::Ptr CloudCornerOld;
    pcl::PointCloud<PointType>::Ptr CloudSurfOld;
    pcl::PointCloud<PointType>::Ptr CloudGroundOld;
    pcl::PointCloud<PointType>::Ptr ground_and_cornerOld;
    pcl::PointCloud<PointType>::Ptr featureOld;
    FPFHFeature::Ptr fpfhFeatureOld;
    FLANNKDTree *fpfhTree;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeOldCloud;
    std::queue<rolo::CloudInfoStamp> laserCloudInfoBuf;
    Matrix<bool, 1, Dynamic> inliers1, inliers2;
    Matrix<double, 3, Dynamic> InliersLast, InliersOld;
    
    Matrix3d Rotation;
    Vector3d Translation;


public:  
    LidarOdometry():
    doneFirstOpt(true),
    isFirstFrame(true)
    {
        // 接受imu原始数据
        subCloudInfo = nh.subscribe<rolo::CloudInfoStamp>("rolo/feature/cloud_info", 10, &LidarOdometry::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        // mapOptimization传来的里程计数据
        // subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        // 发布imu预测里程计
        pubLidarOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);
        pubLaserPath = nh.advertise<nav_msgs::Path> (odomTopic+"_incremental/path", 2000);
        pubRegScan = nh.advertise<sensor_msgs::PointCloud2> (odomTopic+"/registration_scan", 10);
        pubPlotData = nh.advertise<std_msgs::Float64MultiArray> ("rolo/data_test", 10);

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
    }

    void scanRegeistration(){
        // 点云预处理
        // Convert to teaser point cloud
        teaser::PointCloud src_cloud;
        pcd_to_teaser(featureOld, src_cloud);
        teaser::PointCloud tgt_cloud;
        pcd_to_teaser(featureLast, tgt_cloud);
        // 计算FPFH特征
        fpfhFeatureLast = computeFPFH(featureLast);
        // if(fpfhFeatureOld == NULL){
        fpfhFeatureOld  = computeFPFH(featureOld);
        // }
        // Compute correspondences
        teaser::Matcher matcher;
        auto correspondences = matcher.calculateCorrespondences(
            src_cloud, tgt_cloud, *fpfhFeatureOld, *fpfhFeatureLast, true, true, false, 0.95);

        MatCloudInfoLast.resize(3, correspondences.size());
        MatCloudInfoOld.resize(3, correspondences.size());
        for (size_t i = 0; i < correspondences.size(); ++i) {
            auto src_idx = std::get<0>(correspondences[i]);
            auto dst_idx = std::get<1>(correspondences[i]);
            MatCloudInfoOld.col(i) << src_cloud[src_idx].x, src_cloud[src_idx].y, src_cloud[src_idx].z;
            MatCloudInfoLast.col(i) << tgt_cloud[dst_idx].x, tgt_cloud[dst_idx].y, tgt_cloud[dst_idx].z;
        }

        // cloudPreprocessing<double>(featureLast, featureOld, MatCloudInfoLast, MatCloudInfoOld);
        
        // 计算平移不变性
        Matrix<double, 3, Dynamic> src_diff, dst_diff, pruned_src_diff, pruned_dst_diff;
        src_diff = computeTIS(MatCloudInfoOld, TI_map_old);
        dst_diff = computeTIS(MatCloudInfoLast, TI_map_last);
        // 计算内点
        computeInliers(src_diff, dst_diff, &inliers1);
        // std::cout << "inliers1 = " << std::endl << inliers1 << std::endl;
        // 剔除外点
        removeOutliers(TI_map_old, inliers1, src_diff, dst_diff, pruned_src_diff, pruned_dst_diff);

        // 计算旋转
        rotationOptimization(pruned_src_diff, pruned_dst_diff);
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
        int column = 0;
        auto LastCloudSize = cloudLast->size();
        auto OldCloudSize = cloudOld->size();
        // constexpr int dims = 33;
        Matrix<T, 3, Dynamic> LastCloudMatrix;
        Matrix<T, 3, Dynamic> OldCloudMatrix;
        LastCloudMatrix.resize(3, LastCloudSize);
        OldCloudMatrix.resize(3, LastCloudSize);
        std::vector<bool> oldCloudSelected(OldCloudSize, false);
        // 计算FPFH特征
        fpfhFeatureLast = computeFPFH(cloudLast);
        if(fpfhFeatureOld == NULL){
            fpfhFeatureOld  = computeFPFH(cloudOld);
        }
        // 计算关联
        // 建立KDtree
        // using eigen_mat = Matrix<float, Dynamic, Dynamic>;
        // eigen_mat cloud_feature(OldCloudSize, dims);
        // for (int i; i<OldCloudSize; i++) {
        //     for (int j = 0; i < dims; i++)
        //         cloud_feature(i,j) = fpfhFeatureOld->points[i].histogram[j];
        // }
        // using fpfhKdtree = nanoflann::KDTreeEigenMatrixAdaptor<eigen_mat, dims, nanoflann::metric_L2>;
        // fpfhKdtree mat_index(dims, std::cref(cloud_feature), 10);
        
        // std::vector<Eigen::VectorXf> cloud_features;
        // for (auto& f : *fpfhFeatureOld) {
        //     VectorXf fpfh(33);
        //     for (int i = 0; i < 33; i++)
        //         fpfh(i) = f.histogram[i];
        //     cloud_features.push_back(fpfh);
        // }
        // buildKDTree(cloud_features, fpfhTree);
        int rows = (int)fpfhFeatureOld->size();
        int dims = 33;
        std::vector<float> dataset(rows * dims);
        for (int i = 0; i < rows; i++){
            for (int j = 0; j < dims; j++){
                dataset[i * dims + j] = (fpfhFeatureOld->points[i].histogram[j]);         
                // std::cout << bool(dataset.size()==int(rows * dims)) << std::endl;
            }
        }
        flann::Matrix<float> dataset_mat(&dataset[0], rows, dims);
        // std::copy(dataset.begin(), dataset.end(), std::ostream_iterator<float>(std::cout, " "));
        // std::cout << std::endl;

        // FLANNKDTree fpfhTree(flann::KDTreeSingleIndexParams(15));
        FLANNKDTree temp_tree(dataset_mat, flann::KDTreeSingleIndexParams(15));
        temp_tree.buildIndex();
        *fpfhTree = temp_tree;

        // 特征搜索关联点
        // size_t num_results = 1;
        // std::vector<size_t> ret_indexes(num_results);
        // std::vector<float> out_dists_sqr(num_results);
        // for(int i=0; i<LastCloudSize; i++){
        //     // 对第i个点进行特征近邻搜索
        //     // 待查询点
        //     std::vector<float> query;
        //     for (int h=0; h < 33; h++){
        //         query.push_back(fpfhFeatureLast->points[i].histogram[h]);
        //     }
        //     nanoflann::KNNResultSet<float> resultSet(num_results);
        //     resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        //     mat_index.index_->findNeighbors(resultSet, &query[0]);
            
        //     // 建立关联
        //     int corres_ind = ret_indexes[0];
        //     if(!oldCloudSelected[corres_ind]){
        //         LastCloudMatrix.col(column) << cloudLast->points[i].x, cloudLast->points[i].y, cloudLast->points[i].z;
        //         OldCloudMatrix.col(column) << cloudOld->points[corres_ind].x, cloudOld->points[corres_ind].y, cloudOld->points[corres_ind].z;
        //         column++;
        //         oldCloudSelected[corres_ind] = true;
        //     }
        // }
        

        std::vector<int> ind;
        std::vector<float> dis;
        for(int i=0; i<LastCloudSize; i++){
            // 对第i个点进行特征近邻搜索
            std::vector<float> query;
            for (int h=0; h < 33; h++){
                query.push_back(fpfhFeatureLast->points[i].histogram[h]);
            }

            flann::Matrix<float> query_mat(&query[0], 1, 33);

            ind.resize(1);
            dis.resize(1);
            flann::Matrix<int> indices_mat(&ind[0], 1, 1);
            flann::Matrix<float> dists_mat(&dis[0], 1, 1);

            fpfhTree->knnSearch(query_mat, indices_mat, dists_mat, 1, flann::SearchParams(128));
            int corres_ind = ind[0];
            if(!oldCloudSelected[corres_ind]){
                LastCloudMatrix.col(column) << cloudLast->points[i].x, cloudLast->points[i].y, cloudLast->points[i].z;
                OldCloudMatrix.col(column) << cloudOld->points[corres_ind].x, cloudOld->points[corres_ind].y, cloudOld->points[corres_ind].z;
                Vector3d last_diff = LastCloudMatrix.col(column)-OldCloudMatrix.col(column);
                float two_point_distance = sqrt(last_diff.norm());
                printf("two_point_distance = %f\n", two_point_distance);
                column++;
                oldCloudSelected[corres_ind] = true;
            }
        }

        // 关联特征转为矩阵
        last_mat.resize(3, column);
        old_mat.resize(3, column);
        last_mat.leftCols(column) = LastCloudMatrix.leftCols(column);
        old_mat.leftCols(column) = OldCloudMatrix.leftCols(column);
        std::cout << "column: " << column << std::endl;
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
            std::cout << "resual: " << resual << std::endl;
            // 计算补偿量
            // 计算矩阵E
            // std::cout << "resual: " << std::endl << resual << std::endl;
            E.row(0) = resual.transpose();  // P
            E.row(1) += resual.transpose(); // I
            E.row(2) = resual.transpose() - last_resual.transpose(); // D
            if(j==0){
                E.row(2) = MatrixXd::Zero(1,3);
            }
            // 计算U(t)
            U = KScalarMat.array() * E.array();
            auto deltU = U.colwise().sum().transpose(); // 3X1
            // 更新残差和转移矩阵
            translation_diff -= deltU * Eigen::MatrixXd::Ones(1, inliersSize);
            Translation += deltU;
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
        pcl::fromROSMsg(laserCloudInfoLast.extracted_ground, *CloudGroundLast);
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
        

        pubMessage();
    }

    void updateTransform(){
        Vector3d rotation_euler = Rotation.eulerAngles(2,1,0);
        std::cout << "rotation angles: " << std::endl << rotation_euler*180/M_PI << std::endl;
        Matrix4d trans = Matrix4d::Zero();
        trans << Rotation;
        trans.col(3) << Translation(0,0), Translation(1,0), Translation(2,0);
        // trans << Translation;
        size_t cloudSize = FullCloudLast->points.size();

        RegCloud->clear();
        RegCloud->resize(cloudSize);
        RegCloud->points = FullCloudLast->points;
        pcl::transformPointCloud(*FullCloudLast, *RegCloud, trans);
        // #pragma omp parallel for num_threads(numberOfCores)
        // for (int i = 0; i < cloudSize; ++i)
        // {
        //     const auto &pointFrom = FullCloudLast->points[i];
        //     // RegCloud->points[i].x = trans(0,0) * pointFrom.x + trans(0,1) * pointFrom.y + trans(0,2) * pointFrom.z + trans(0,3);
        //     // RegCloud->points[i].y = trans(1,0) * pointFrom.x + trans(1,1) * pointFrom.y + trans(1,2) * pointFrom.z + trans(1,3);
        //     // RegCloud->points[i].z = trans(2,0) * pointFrom.x + trans(2,1) * pointFrom.y + trans(2,2) * pointFrom.z + trans(2,3);
        //     RegCloud->points[i].x = trans(0,0) * pointFrom.x + trans(0,1) * pointFrom.y + trans(0,2) * pointFrom.z + trans(0,3);
        //     RegCloud->points[i].y = trans(1,0) * pointFrom.x + trans(1,1) * pointFrom.y + trans(1,2) * pointFrom.z + trans(1,3);
        //     RegCloud->points[i].z = trans(2,0) * pointFrom.x + trans(2,1) * pointFrom.y + trans(2,2) * pointFrom.z + trans(2,3);
        //     RegCloud->points[i].intensity = pointFrom.intensity;
        // }
        
        // 新旧信息交换
        *FullCloudOld = *FullCloudLast;
        *CloudCornerOld = *CloudCornerLast;
        *CloudSurfOld = *CloudSurfLast;
        *CloudGroundOld = *CloudGroundLast;
        *featureOld = *featureLast;
        *ground_and_cornerOld = *ground_and_cornerLast;
        *fpfhFeatureOld = *fpfhFeatureLast;
    }
    
    // void pubTranform(){

    // }

    void pubMessage(){
        publishCloud(pubRegScan, RegCloud, cloudTimeStamp, "map");
    }

    void pcd_to_teaser(pcl::PointCloud<PointType>::Ptr &input_cloud, teaser::PointCloud &output_cloud) {
        for (size_t i = 0; i < input_cloud->points.size(); ++i) {
            output_cloud.push_back(
                {input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z});
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rolo");
    
    LidarOdometry LO;
    
    ROS_INFO("\033[1;32m----> Laser Odometry Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}