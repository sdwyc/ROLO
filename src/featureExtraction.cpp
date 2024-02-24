#include "rolo/utility.h"

struct smoothness_t{ 
    float value;    // 平滑度大小
    size_t ind;     // 在点云中的索引
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamLoader
{

public:

    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud; // 输入点云
    pcl::PointCloud<PointType>::Ptr cornerCloud;    // 角点集合
    pcl::PointCloud<PointType>::Ptr surfaceCloud;   // 平面点集合

    pcl::VoxelGrid<PointType> downSizeFilter;

    rolo::CloudInfoStamp cloudInfo;
    std_msgs::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;  // 存储输入点云里每个点的平滑度和索引值，并按照平滑度从小到大排序
    float *cloudCurvature;      // 数组，存储输入点云的平滑度
    int *cloudNeighborPicked;   // 数组，用来标记有效点，选择过的点和异常点，0表示当前点可选为特征点，1代表当前点不可选为特征点
    int *cloudLabel;            // 数组，用来标记特征点类型，0为普通点，1为角点，-1为平面点
    //! 输入：image_projection传来的cloud_info，输出：角点点云，平面点点云，feature_cloud_info
    FeatureExtraction()
    {
        subLaserCloudInfo = nh.subscribe<rolo::CloudInfoStamp>("rolo/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        pubLaserCloudInfo = nh.advertise<rolo::CloudInfoStamp> ("rolo/feature/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("rolo/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("rolo/feature/cloud_surface", 1);
        
        initializationValue();
    }
    //! 初始参数，体素滤波器，变量
    void initializationValue()
    {
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
    }
    //! range_image回调函数，主要完成：平滑度计算，特征提取，输出点云
    void laserCloudInfoHandler(const rolo::CloudInfoStampConstPtr& msgIn)
    {
        // 存储msgIn
        cloudInfo = *msgIn; // new cloud info
        cloudHeader = msgIn->header; // new cloud header
        pcl::fromROSMsg(msgIn->cloud_projected, *extractedCloud); // new cloud for extraction
        // 遍历输入点云，计算每个点的平滑度，并存储
        calculateSmoothness();
        // 标记异常点，保证异常点不被提取为特征
        markOccludedPoints();
        // 提取角点和平面点
        extractFeatures();
        // 发布输出点云消息
        publishFeatureCloud();
    }
    //! 遍历输入点云，计算每个点的平滑度，并标记存储
    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        // 使用和LOAM相同的平滑度计算公式
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;
            // 标记取过邻居的点
            cloudNeighborPicked[i] = 0;
            // 标记计算过平滑度的点
            cloudLabel[i] = 0;
            // cloudSmoothness for sorting
            // 存储点的平滑度
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }
    //! 标记当前输入点云中的异常点（遮挡点和平行点）
    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // occluded points
            // 相邻索引，判断遮挡点
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));
            // 筛选并剔除被遮挡的点，因为被遮挡的点可能在下一帧就没有了，不能选为特征点
            if (columnDiff < 10){
                // 10 pixel diff in range image
                if (depth1 - depth2 > 0.3){ // depth1所代表的第i个点把第i+1个点遮挡了，因此剔除第i前面的几个点
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){ // depth1所代表的第i个点把第i+1个点遮挡了，因此剔除第i+1后面的几个点
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            // parallel beam
            // 筛选并剔除平行点
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));
            // 如果i+1到i的连线与激光线接近平行，那么i+1和i的深度差将会很大
            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1; // 标记平行点
        }
    }
    
    //! 根据平滑度，提取输入点云中的角点和平面点，并存储
    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());     // 当前帧原始平面点
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());   // 滤波后的平面点
        // 筛选角点和平面点，方法类似LOAM
        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();
            // 横向上分为6个扇区
            for (int j = 0; j < 6; j++)
            {
                // 求每个扇区的起始索引和结束索引
                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;
                // 按照平滑度由小到大排序
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());
                // 筛选角点
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20){
                            cloudLabel[ind] = 1;    // 标记为角点
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }
                        // 标记选过的点，周围的点不再选择
                        cloudNeighborPicked[ind] = 1;
                        // 检查ind周围的节点
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10) // 说明两点横向上相差较远，不需要再标记邻近节点
                                break;
                            cloudNeighborPicked[ind + l] = 1; // 周围的点不再选择
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                // 筛选平面点
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {

                        cloudLabel[ind] = -1;           // 标记为角点
                        cloudNeighborPicked[ind] = 1;   // 标记为选择过的点
                        // 周围点不再选择
                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }
            // 对筛选的平面点进行体素滤波，减少点的数量
            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
        }
    }
    //! 特征提取完成后，清除输入点云中的无用信息
    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }
    //! 发布输出点云消息
    void publishFeatureCloud()
    {
        // free cloud info memory 清除输入点云中的无用信息
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.extracted_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.extracted_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rolo");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

    ros::spin();

    return 0;
}