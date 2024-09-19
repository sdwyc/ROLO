#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <string>
#include <teaser/registration.h>
#include <teaser/matcher.h>
#include <Eigen/Core>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <chrono>

using namespace std;
typedef pcl::PointXYZI  PointType;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
string src_path;
string dst_path;

const int systemDelay = 0; // 系统延时开启时间
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 16;    // 激光雷达线数
float cloudCurvature[400000];   // 存储每个点的平滑度
int cloudSortInd[400000];       // 存储点的索引，根据平滑度由小到大排序
int cloudNeighborPicked[400000]; // 如果点周围没有选为角点或平面点，则为0；反之为1
int cloudLabel[400000];         // 标记点的状态，2：角点，1：轻微角点，0：普通，-1，平面点

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out) // 初始化点云header
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i) // 遍历所有输入的点云
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1; // 对于无组织点云数据集，高度设置为1；对于一个有组织的点云数据集，高度等于行数。
    cloud_out.width = static_cast<uint32_t>(j);// 对于无组织点云数据集，宽度特指该点云所有点的个数；对于有组织的点云数据集，宽度是指一行有多少个点。
    cloud_out.is_dense = true; // 指定点云中的所有数据是有效的则为true，否则为false (e.g., have NaN or Inf values).）
}

void featureExtraction(const pcl::PointCloud<PointType> &inputCloud, pcl::PointCloud<PointType> &featureCloud){
    //创建一个vector，包含16个值为0的元素
    std::vector<int> scanStartInd(N_SCANS, 0); // 记录了每一线中开始迭代计算某点平滑度的起始索引
    std::vector<int> scanEndInd(N_SCANS, 0);   // 记录了每一线中开始迭代计算某点平滑度的结束索引

    pcl::PointCloud<PointType> laserCloudIn; // 输入cloud
    laserCloudIn = inputCloud;
    
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices); // indices为对应保留的点索引
    removeClosedPointCloud(laserCloudIn, laserCloudIn, 0.5); // 滤除距离机器人过近的点云

    int cloudSize = laserCloudIn.points.size(); // 点云规模
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    // 限制endOri与startOri的角度在0～360度以内
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false; // 用于判断是否超过startOri 180度
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS); //vector包含了每一线的点云
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI; // 点到基座的俯仰角，单位：degree
        int scanID = 0;
        // 判断一个点属于哪个线上的点，scanID为线数的序列号
        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri); // 偏航角的角度占比，与intensity有关
        point.intensity = scanID + 0.1 * relTime;    // intensity包含了扫瞄线数和朝向角信息
        laserCloudScans[scanID].push_back(point); 
    }
    
    cloudSize = count;

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    // 计算所有点的平滑度（前后5个为一组，一组共11个）
    for (int i = 5; i < cloudSize - 5; i++)
    {
        // 计算第i个点的前后5X5区域的平滑度（曲率，cloudCurvature）, 数越大代表越尖，数越小代表平坦
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }


    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        // 在横向上将每一线都分成六个扇区
        for (int j = 0; j < 6; j++)
        {
            // 求每个扇区的起始索引和结束索引
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            // 第i个扇区的索引按照平滑度由小到大排序
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            // 开始筛选角点
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k]; 
                // 当平滑度>0.1，且邻居没有选择，则选为角点，每个扇区数量最大为20
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {                        
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        featureCloud.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        featureCloud.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1; 

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // 开始筛选平面点
            int smallestPickedNum = 0;
            // 当点的邻居没有被选择，且平滑度<0.1时，每个扇区最大数量为4个，被选为平面点
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud->points[ind]);
                    featureCloud.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }
        // 对轻微平面点进行体素滤波
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
        featureCloud += surfPointsLessFlatScanDS;
    }
}

/**
 * @brief: 绕 axis 轴旋转 theta 度
 * @param {float} theta 单位为度
 * @param {string} axis 可选 x, y, z
 * @param {Ptr} &source
 * @param {Ptr} &target
 * @return {*}
 * @note:
 * @warning:
 */
void euclidean_rotate(float theta, std::string axis, pcl::PointCloud<PointType>::Ptr &source, pcl::PointCloud<PointType>::Ptr &target) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    theta = M_PI / 180 * theta;
    if (axis == "x") {
        T(1, 1) = cos(theta);
        T(1, 2) = sin(theta);
        T(2, 1) = -sin(theta);
        T(2, 2) = cos(theta);
    } else if (axis == "y") {
        T(0, 0) = cos(theta);
        T(0, 2) = -sin(theta);
        T(2, 0) = sin(theta);
        T(2, 2) = cos(theta);
    } else if (axis == "z") {
        T(0, 0) = cos(theta);
        T(0, 1) = -sin(theta);
        T(1, 0) = sin(theta);
        T(1, 1) = cos(theta);
    }
    pcl::transformPointCloud(*source, *target, T);
}

/**
 * @brief: 平移变换
 * @param {float} x
 * @param {float} y
 * @param {float} z
 * @param {Ptr} &source
 * @param {Ptr} &target
 * @return {*}
 * @note:
 * @warning:
 */
void euclidean_translate(float x, float y, float z, pcl::PointCloud<PointType>::Ptr &source, pcl::PointCloud<PointType>::Ptr &target) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;
    pcl::transformPointCloud(*source, *target, T);
}

void voxel_filter(pcl::PointCloud<PointType>::Ptr &cloud, pcl::PointCloud<PointType>::Ptr &cloud_filtered,
                  float leaf_size = 1.0f) {
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    // pcl::UniformSampling<PointType> voxel;
    // voxel.setRadiusSearch(leaf_size);
    voxel.setInputCloud(cloud);
    voxel.filter(*cloud_filtered);
    // voxel.getRemovedIndices(*inliers);
}

static Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R) {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d rpy(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        rpy(0) = r*180/M_PI;
        rpy(1) = p*180/M_PI;
        rpy(2) = y*180/M_PI;
        return rpy;
    }

void readPcdFile(const string &in_path, Eigen::Matrix3Xd &out_mat){
    pcl::PointCloud<PointType>::Ptr temp_cloud_pt(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr temp_cloudDS(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> DS;
    DS.setInputCloud(temp_cloud_pt);
    DS.setLeafSize(0.3,0.3,0.3);
    pcl::io::loadPCDFile(in_path, *temp_cloud_pt);
    DS.filter(*temp_cloudDS);
    int point_num = temp_cloudDS->size();
    out_mat.resize(3, point_num);
    for(int i=0; i<point_num; i++){
        out_mat(0,i) = temp_cloudDS->points[i].x;
        out_mat(1,i) = temp_cloudDS->points[i].y;
        out_mat(2,i) = temp_cloudDS->points[i].z;
    }
}

void teaser_to_correspondence(std::vector<std::pair<int, int>> &input,
                              pcl::Correspondences &output) {
    for (size_t i = 0; i < input.size(); ++i) {
        output.push_back(pcl::Correspondence(input[i].first, input[i].second, 0.0));
    }
}

/**
 * @brief: 计算法向量
 * @param {Ptr} &cloud
 * @param {Ptr} &normals 返回法向坐标和表面曲率估计的点结构
 * @return {*}
 * @note:
 * @warning:
 */
void compute_normal(pcl::PointCloud<PointType>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    //-------------------------法向量估计-----------------------
    pcl::NormalEstimationOMP<PointType, pcl::Normal> n;
    n.setInputCloud(cloud);
    n.setNumberOfThreads(8); //设置openMP的线程数
    // n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
    n.setSearchMethod(tree);
    n.setKSearch(10);
    // n.setRadiusSearch(0.03);//半径搜素
    n.compute(*normals);
}

/**
 * @brief: 计算 FPFH 特征描述子
 * @param {Ptr} input_cloud 输入点云
 * @param {Ptr} normals 输入点云的法向量
 * @return {*}
 * @note:
 * @warning:
 */
fpfhFeature::Ptr compute_fpfh_feature(pcl::PointCloud<PointType>::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    //------------------FPFH估计-------------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> f;
    f.setNumberOfThreads(8); // 指定8核计算
    f.setInputCloud(input_cloud);
    f.setInputNormals(normals);
    f.setSearchMethod(tree);
    f.setKSearch(10);
    f.compute(*fpfh);

    return fpfh;
}

void pcd_to_teaser(pcl::PointCloud<PointType>::Ptr &input_cloud, teaser::PointCloud &output_cloud) {
    for (size_t i = 0; i < input_cloud->points.size(); ++i) {
        output_cloud.push_back(
            {input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z});
    }
}

teaser::RegistrationSolution teaserpp_registration(pcl::PointCloud<PointType>::Ptr &SCloud,
                                                   pcl::PointCloud<PointType>::Ptr &TCloud,
                                                   pcl::Correspondences &cru_correspondences) {
    // Convert to teaser point cloud
    teaser::PointCloud src_cloud;
    pcd_to_teaser(SCloud, src_cloud);

    teaser::PointCloud tgt_cloud;
    pcd_to_teaser(TCloud, tgt_cloud);

    // Compute FPFH
    pcl::PointCloud<pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr tgt_normals(new pcl::PointCloud<pcl::Normal>);

    compute_normal(SCloud, src_normals);
    fpfhFeature::Ptr obj_descriptors = compute_fpfh_feature(SCloud, src_normals);
    compute_normal(TCloud, tgt_normals);
    fpfhFeature::Ptr scene_descriptors = compute_fpfh_feature(TCloud, tgt_normals);

    std::cout << "obj_descriptors size: " << obj_descriptors->points.size() << std::endl;
    std::cout << "scene_descriptors size: " << scene_descriptors->points.size() << std::endl;

    // Compute correspondences
    teaser::Matcher matcher;
    auto correspondences = matcher.calculateCorrespondences(
        src_cloud, tgt_cloud, *obj_descriptors, *scene_descriptors, true, true, false, 0.95);

    std::cout << "correspondences size: " << correspondences.size() << std::endl;
    teaser_to_correspondence(correspondences, cru_correspondences);

    // Run TEASER++ registration
    // Prepare solver parameters
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = 0.05;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.005;

    // Solve with TEASER++
    teaser::RobustRegistrationSolver solver(params);
    auto start = std::chrono::system_clock::now();
    solver.solve(src_cloud, tgt_cloud, correspondences);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    printf("Solver Duration: %f ms.\n" ,elapsed_seconds.count() * 1000);

    teaser::RegistrationSolution solution = solver.getSolution();
    return solution;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teaser_feature_matching");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher source_pub = nh.advertise<sensor_msgs::PointCloud2>("source_cloud", 1);
    ros::Publisher target_pub = nh.advertise<sensor_msgs::PointCloud2>("target_cloud", 1);
    ros::Publisher trans_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);
    float t_x, t_y, t_z, r_r, r_p, r_y;
    bool use_priored_transformed = false;
    ros::param::param<bool>("~/use_priored_transformed", use_priored_transformed, false);
    ros::param::param<float>("~/t_x", t_x, 0.0);
    ros::param::param<float>("~/t_y", t_y, 0.0);
    ros::param::param<float>("~/t_z", t_z, 0.0);
    ros::param::param<float>("~/r_r", r_r, 0.0);
    ros::param::param<float>("~/r_p", r_p, 0.0);
    ros::param::param<float>("~/r_y", r_y, 0.0);

    if(argc >= 3){
        src_path = argv[1]; // src_cloud – source point cloud (to be transformed)
        dst_path = argv[2]; // dst_cloud – target point cloud (after transformation)
    }
    else{
        ROS_ERROR("Too less args!!");
        return 0;
    }
    // Load the .pcd file
    pcl::PointCloud<PointType>::Ptr SourceCloud(new pcl::PointCloud<PointType>); // 源点云
    pcl::PointCloud<PointType>::Ptr TargetCloud(new pcl::PointCloud<PointType>); // 目标点云   
    pcl::PointCloud<PointType>::Ptr SCloud(new pcl::PointCloud<PointType>); // 源点云
    pcl::PointCloud<PointType>::Ptr TCloud(new pcl::PointCloud<PointType>); // 目标点云
    pcl::PointCloud<PointType>::Ptr TransformCloud(new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile(src_path, *SourceCloud);
    pcl::io::loadPCDFile(dst_path, *TargetCloud);
    featureExtraction(*SourceCloud, *SCloud);
    featureExtraction(*TargetCloud, *TCloud);

    if(use_priored_transformed){
        // 旋转平移
        // TCloud->clear();
        euclidean_rotate(r_r, "x", TCloud, TCloud);
        euclidean_rotate(r_p, "y", TCloud, TCloud);
        euclidean_rotate(r_y, "z", TCloud, TCloud);
        euclidean_translate(t_x, t_y, t_z, TCloud, TCloud);
    }
    // 降采样
    voxel_filter(SCloud, SCloud, 0.5);
    voxel_filter(TCloud, TCloud, 0.5);
    printf("source cloud size: \nraw(%d), raw_feature(%d)\ntarget(%d), tar_feature(%d)\n"
                                          , SourceCloud->size(), SCloud->size()
                                          , TargetCloud->size(), TCloud->size());

    ROS_INFO("\033[1;32m----> Starting Matching.\033[0m");
    // Teaser++ 配准
    auto start = std::chrono::system_clock::now();
    pcl::Correspondences cru_correspondences;
    teaser::RegistrationSolution solution =
        teaserpp_registration(SCloud, TCloud, cru_correspondences);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    printf("Point Registration Duration: %f ms.\n" ,elapsed_seconds.count() * 1000);

    Eigen::Matrix3d R = solution.rotation;
    Eigen::Vector3d R_E =R2rpy(R);// R.eulerAngles(2,1,0); 
    
    Eigen::Vector3d T = solution.translation;
    std::cout << R << std::endl;
    printf("Rotation Mat:\n %f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n", 
            R(0,0),R(0,1),R(0,2),R(1,0),R(1,1),R(1,2),R(2,0),R(2,1),R(2,2));
    printf("Rotation Eula: [%f\t%f\t%f]\n", 
            R_E(0),R_E(1),R_E(2));    
    printf("Translation Mat: [%f\t%f\t%f]\n", 
            T(0),T(1),T(2));
    // 执行变换
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = solution.rotation;
    transformation.block<3, 1>(0, 3) = solution.translation;
    pcl::transformPointCloud(*SCloud, *TransformCloud, transformation);
    // 可视化
    // pcl::visualization::PCLVisualizer viewer("Alignment - Teaser");
    // pcl::visualization::PointCloudColorHandlerCustom<PointType> source_color(SCloud, 0, 255, 255);
    // viewer.addPointCloud(SCloud, source_color, "SCloud");
    // pcl::visualization::PointCloudColorHandlerCustom<PointType> target_color(TCloud, 255, 0, 0);
    // viewer.addPointCloud(TCloud, target_color, "TCloud");
    // pcl::visualization::PointCloudColorHandlerCustom<PointType> reg_color(SCloud, 0, 255, 0);
    // viewer.addPointCloud(TransformCloud, reg_color, "RegCloud");
    // 对应关系可视化
    // viewer.setWindowName("基于特征描述子的对应");
    // viewer.addCorrespondences<PointType>(TransformCloud, TCloud, cru_correspondences,
    //                                          "correspondence");
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2,
    //                                    "correspondence");
    // viewer.spin();
    sensor_msgs::PointCloud2 source_point, target_point, transformed_point;
    pcl::toROSMsg<PointType>(*SCloud, source_point);
    pcl::toROSMsg<PointType>(*TCloud, target_point);
    pcl::toROSMsg<PointType>(*TransformCloud, transformed_point);
    source_point.header.frame_id = "map";
    source_point.header.stamp = ros::Time::now();
    target_point.header.frame_id = "map";
    target_point.header.stamp = ros::Time::now();
    transformed_point.header.frame_id = "map";
    transformed_point.header.stamp = ros::Time::now();

    Eigen::Matrix<double, 3, Eigen::Dynamic> mat_test(3,3);
    Eigen::MatrixXd mat1 = Eigen::MatrixXd::Ones(3,2);
    mat_test.col(0) << 1,2,3;
    std::cout << mat_test << std::endl;
    std::cout << mat1 << std::endl;
    
    mat1.array().row(0) *= mat_test(0, 0);
    mat1.array().row(1) *= mat_test(1, 0);
    mat1.array().row(2) *= mat_test(2, 0);
    mat_test << mat1;
    std::cout << mat_test << std::endl;

    while(ros::ok()){
        source_pub.publish(source_point);
        target_pub.publish(target_point);
        trans_pub.publish(transformed_point);
    }
    return 0;
}