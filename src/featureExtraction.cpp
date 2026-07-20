#include "rolo/utility.h"

struct smoothness_ind{ 
    float value;    // Point smoothness
    size_t ind;     // Point index
};

struct comp_rule{ 
    bool operator()(smoothness_ind const &left, smoothness_ind const &right) { 
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
    ros::Publisher pubNormalPoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud; // Input cloud
    pcl::PointCloud<PointType>::Ptr cornerCloud;    // Corner cloud
    pcl::PointCloud<PointType>::Ptr surfaceCloud;   // Surface cloud
    pcl::PointCloud<PointType>::Ptr normalCloud;    // Surface cloud

    pcl::VoxelGrid<PointType> downSizeFilter;


    rolo::CloudInfoStamp cloudInfo;
    std_msgs::Header cloudHeader;

    std::vector<smoothness_ind> cloudSmoothness;  // Point index
    float *cloudCurvature;      // Point smoothness
    int *cloudNeighborPicked;   // Cloud neighbor picked
    int *cloudLabel;            // Surface cloud
    
    FeatureExtraction()
    {
        //! Feature cloud info input
        subLaserCloudInfo = nh.subscribe<rolo::CloudInfoStamp>("rolo/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        pubLaserCloudInfo = nh.advertise<rolo::CloudInfoStamp> ("rolo/feature/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("rolo/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("rolo/feature/cloud_surface", 1);
        pubNormalPoints = nh.advertise<sensor_msgs::PointCloud2>("rolo/feature/cloud_normal", 1);

        initializationValue();
    }
    //! Voxel processing
    void initializationValue()
    {
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());
        normalCloud.reset(new pcl::PointCloud<PointType>());


        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
    }
    //! Extracted cloud
    void laserCloudInfoHandler(const rolo::CloudInfoStampConstPtr& cloudIn)
    {
        // Store input message
        cloudInfo = *cloudIn; // new cloud info
        cloudHeader = cloudIn->header; // new cloud header
        pcl::fromROSMsg(cloudIn->cloud_projected, *extractedCloud); // new cloud for extraction
        // Point smoothness
        calculateSmoothness();
        // Mark outlier points
        markOccludedPoints();
        // Surface cloud
        extractFeatures();
        // Publish output cloud
        publishFeatureCloud();
    }
    //! Point smoothness
    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        // Point smoothness
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;
            // Mark picked neighbors
            cloudNeighborPicked[i] = 0;
            // Point smoothness
            cloudLabel[i] = 0;
            // cloudSmoothness for sorting
            // Point smoothness
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }
    //! Mark occluded points
    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // occluded points
            // Point index
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));
            // Picked-neighbor flag
            if (columnDiff < 10){
                // 10 pixel diff in range image
                if (depth1 - depth2 > 0.3){ // Mark occluded points
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){ // Mark occluded points
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            // parallel beam
            // Picked-neighbor flag
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));
            // Mark occluded points
            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1; // I
        }
    }
    
    //! Extracted cloud
    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();
        normalCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());     // Surface cloud
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());   // Surface cloud
        pcl::PointCloud<PointType>::Ptr normalCloudScan(new pcl::PointCloud<PointType>());     // Surface cloud
        pcl::PointCloud<PointType>::Ptr normalCloudScanDS(new pcl::PointCloud<PointType>());   // Surface cloud
        
        // Surface cloud
        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();
            normalCloudScan->clear();
            // Split scan into sectors
            for (int j = 0; j < 6; j++)
            {
                // Split scan into sectors
                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;
                // Point smoothness
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, comp_rule());
                // Picked-neighbor flag
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20){
                            cloudLabel[ind] = 1;    // Index
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }
                        // Mark selected point and neighbors
                        cloudNeighborPicked[ind] = 1;
                        // Check neighbor points
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10) // Picked-neighbor flag
                                break;
                            cloudNeighborPicked[ind + l] = 1; // L
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
                // Surface cloud
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {
                        cloudLabel[ind] = -1;           // Surface cloud
                        cloudNeighborPicked[ind] = 1;   // Index
                        // Suppress neighboring points
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
                        // if(cloudLabel[k] == -2){
                        //     normalCloudScan->push_back(extractedCloud->points[k]);
                        // }
                    }

                    // if(cloudLabel[k] == 0){
                    //     normalCloudScan->push_back(extractedCloud->points[k]);
                    // }
                }
            }
            // Surface cloud
            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);
            normalCloudScanDS->clear();
            downSizeFilter.setInputCloud(normalCloudScan);
            downSizeFilter.filter(*normalCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
            *normalCloud += *normalCloudScanDS;
        }
    }
    //! Extracted cloud
    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }
    //! Publish output cloud
    void publishFeatureCloud()
    {
        // Clear unused input cloud info
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.extracted_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.extracted_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        cloudInfo.extracted_normal = publishCloud(pubNormalPoints, normalCloud, cloudHeader.stamp, lidarFrame);

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