#include "utility.hpp"

#include <opencv2/opencv.hpp>

struct smoothness_ind{
    float value;
    size_t ind;
};

struct comp_rule{
    bool operator()(smoothness_ind const &left, smoothness_ind const &right) {
        return left.value < right.value;
    }
};

const int queueLength = 2000;

class ScanPreprocess : public ParamLoader
{
private:

    ros::Subscriber subLaserCloud;
    ros::Subscriber subOdom;

    ros::Publisher pubFeatureCloudInfo;
    ros::Publisher pubProjectedCloud;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;
    ros::Publisher pubNormalPoints;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    std::deque<nav_msgs::Odometry> odomQueue;
    std::mutex odomLock;
    sensor_msgs::PointCloud2 currentCloudMsg;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterInputPointType>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr deskewCloud;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;
    pcl::PointCloud<PointType>::Ptr normalCloud;

    cv::Mat rangeMat;
    pcl::VoxelGrid<PointType> downSizeFilter;

    rolo::CloudInfoStamp cloudInfoStamp;
    std_msgs::Header cloudHeader;

    std::vector<int> columnIdnCountVec;
    std::vector<smoothness_ind> cloudSmoothness;

    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    std::string timeField;
    int timeFlag = 0;
    float scanPeriod = 0.1;
    double odomTimeDiff = -1.0;
    float odomIncreX, odomIncreY, odomIncreZ, odomIncreRoll, odomIncrePitch, odomIncreYaw;
    bool odomAvailable = false;
    double timeScanCur;
    double timeScanEnd;

public:
    ScanPreprocess()
    {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 10, &ScanPreprocess::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ScanPreprocess::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        pubFeatureCloudInfo = nh.advertise<rolo::CloudInfoStamp> ("rolo/feature/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("rolo/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("rolo/feature/cloud_surface", 1);
        pubNormalPoints = nh.advertise<sensor_msgs::PointCloud2>("rolo/feature/cloud_normal", 1);

        allocateMemory();
        resetParameters();
        timeField = "time";
    }

    ~ScanPreprocess()
    {
        delete[] cloudCurvature;
        delete[] cloudNeighborPicked;
        delete[] cloudLabel;
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterInputPointType>());
        deskewCloud.reset(new pcl::PointCloud<PointType>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());
        normalCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfoStamp.startRingIndex.assign(N_SCAN, 0);
        cloudInfoStamp.endRingIndex.assign(N_SCAN, 0);
        cloudInfoStamp.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfoStamp.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);
        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        deskewCloud->clear();
        extractedCloud->clear();
        cornerCloud->clear();
        surfaceCloud->clear();
        normalCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        columnIdnCountVec.assign(N_SCAN, 0);
    }

    void odometryHandler(const nav_msgs::OdometryConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(odomLock);
        odomQueue.push_back(*odomMsg);
        if (odomQueue.size() > queueLength)
            odomQueue.pop_front();
        if (odomQueue.size() >= 2)
            odomAvailable = true;
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewCloudInfo())
            return;

        projectPointCloud();
        cloudExtraction();
        calculateSmoothness();
        markOccludedPoints();
        extractFeatures();
        publishFeatureCloud();
        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        if (sensor == lidarType::VELODYNE)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == lidarType::OUSTER)
        {
            timeField = "t";
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
#if HasRGB
                dst.rgb = src.rgb;
#endif
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    ROS_WARN("Point cloud ring field available!");
                    break;
                }
            }
        }

        if (timeFlag == 0)
        {
            timeFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == timeField)
                {
                    timeFlag = 1;
                    ROS_WARN("Point cloud time field available!");
                    break;
                }
            }
        }
        return true;
    }

    bool deskewCloudInfo()
    {
        if (!deskewEnabled || !odomAvailable)
            return true;

        int cloudSize = laserCloudIn->points.size();
        std::lock_guard<std::mutex> lock(odomLock);
        if (odomQueue.size() < 2)
            return true;

        float startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        float endOri   = -atan2(laserCloudIn->points[cloudSize - 1].y, laserCloudIn->points[cloudSize - 1].x) + 2 * M_PI;
        if (endOri - startOri > 3 * M_PI)
            endOri -= 2 * M_PI;
        else if (endOri - startOri < M_PI)
            endOri += 2 * M_PI;
        float orientationDiff = endOri - startOri;

        while (!odomQueue.empty())
        {
            double maxDiff = timeFlag == -1 ? 0.25 : 0.3;
            if (fabs(timeScanCur - odomQueue.front().header.stamp.toSec()) > maxDiff)
                odomQueue.pop_front();
            else
                break;
        }
        if (odomQueue.size() < 2)
            return true;

        Eigen::Affine3f lidarOdomAffineFront = odom2affine(odomQueue.front());
        Eigen::Affine3f lidarOdomAffineBack = odom2affine(odomQueue.back());
        Eigen::Affine3f lidarOdomAffineIncre = lidarOdomAffineFront.inverse() * lidarOdomAffineBack;
        odomTimeDiff = odomQueue.back().header.stamp.toSec() - odomQueue.front().header.stamp.toSec();
        if (odomTimeDiff <= 0)
            return true;
        pcl::getTranslationAndEulerAngles(lidarOdomAffineIncre, odomIncreX, odomIncreY, odomIncreZ, odomIncreRoll, odomIncrePitch, odomIncreYaw);

        bool halfPassed = false;
        PointType point;
        deskewCloud->points.resize(cloudSize);
        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn->points[i].y;
            point.y = laserCloudIn->points[i].z;
            point.z = laserCloudIn->points[i].x;
#if HasRGB
            point.rgb = laserCloudIn->points[i].rgb;
#endif

            if (timeFlag == -1)
            {
                float ori = -atan2(point.x, point.z);
                if (!halfPassed)
                {
                    if (ori < startOri - M_PI / 2)
                        ori += 2 * M_PI;
                    else if (ori > startOri + M_PI * 3 / 2)
                        ori -= 2 * M_PI;

                    if (ori - startOri > M_PI)
                        halfPassed = true;
                }
                else
                {
                    ori += 2 * M_PI;
                    if (ori < endOri - M_PI * 3 / 2)
                        ori += 2 * M_PI;
                    else if (ori > endOri + M_PI / 2)
                        ori -= 2 * M_PI;
                }
                point.intensity = scanPeriod * (ori - startOri) / orientationDiff;
            }
            else
            {
                point.intensity = fabs(laserCloudIn->points[i].time);
            }
            deskewCloud->points[i] = point;
        }
        return true;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (!deskewEnabled || !odomAvailable || odomTimeDiff <= 0)
            return *point;

        float ratio = relTime / scanPeriod;
        Eigen::Matrix<float, 6, 1> trans;
        trans.col(0) << odomIncreX, odomIncreY, odomIncreZ, odomIncreRoll, odomIncrePitch, odomIncreYaw;
        trans = trans.eval() * (scanPeriod / odomTimeDiff) * ratio;
        Eigen::Affine3f transBt = pcl::getTransformation(0.0, 0.0, 0.0, -trans(3), -trans(4), -trans(5));

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;
#if HasRGB
        newPoint.rgb = point->rgb;
#endif
        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].ring * laserCloudIn->points[i].z;
#if HasRGB
            thisPoint.rgb = laserCloudIn->points[i].rgb;
#endif

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == lidarType::VELODYNE || sensor == lidarType::OUSTER)
            {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0 / float(Horizon_SCAN);
                columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            if (deskewEnabled && odomAvailable && !deskewCloud->empty())
                thisPoint = deskewPoint(&thisPoint, deskewCloud->points[i].intensity);

            rangeMat.at<float>(rowIdn, columnIdn) = range;
            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfoStamp.startRingIndex[i] = count - 1 + 5;
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    cloudInfoStamp.pointColInd[count] = j;
                    cloudInfoStamp.pointRange[count] = rangeMat.at<float>(i,j);
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    ++count;
                }
            }
            cloudInfoStamp.endRingIndex[i] = count - 1 - 5;
        }
        cloudInfoStamp.header = cloudHeader;
        cloudInfoStamp.cloud_projected = publishCloud(pubProjectedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
    }

    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffRange = cloudInfoStamp.pointRange[i-5] + cloudInfoStamp.pointRange[i-4]
                            + cloudInfoStamp.pointRange[i-3] + cloudInfoStamp.pointRange[i-2]
                            + cloudInfoStamp.pointRange[i-1] - cloudInfoStamp.pointRange[i] * 10
                            + cloudInfoStamp.pointRange[i+1] + cloudInfoStamp.pointRange[i+2]
                            + cloudInfoStamp.pointRange[i+3] + cloudInfoStamp.pointRange[i+4]
                            + cloudInfoStamp.pointRange[i+5];

            cloudCurvature[i] = diffRange * diffRange;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            float depth1 = cloudInfoStamp.pointRange[i];
            float depth2 = cloudInfoStamp.pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfoStamp.pointColInd[i+1] - cloudInfoStamp.pointColInd[i]));
            if (columnDiff < 10)
            {
                if (depth1 - depth2 > 0.3)
                {
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
                else if (depth2 - depth1 > 0.3)
                {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::abs(float(cloudInfoStamp.pointRange[i-1] - cloudInfoStamp.pointRange[i]));
            float diff2 = std::abs(float(cloudInfoStamp.pointRange[i+1] - cloudInfoStamp.pointRange[i]));
            if (diff1 > 0.02 * cloudInfoStamp.pointRange[i] && diff2 > 0.02 * cloudInfoStamp.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();
        normalCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr normalCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr normalCloudScanDS(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();
            normalCloudScan->clear();
            for (int j = 0; j < 6; j++)
            {
                int sp = (cloudInfoStamp.startRingIndex[i] * (6 - j) + cloudInfoStamp.endRingIndex[i] * j) / 6;
                int ep = (cloudInfoStamp.startRingIndex[i] * (5 - j) + cloudInfoStamp.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, comp_rule());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20)
                        {
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        }
                        else
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfoStamp.pointColInd[ind + l] - cloudInfoStamp.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfoStamp.pointColInd[ind + l] - cloudInfoStamp.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {
                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfoStamp.pointColInd[ind + l] - cloudInfoStamp.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfoStamp.pointColInd[ind + l] - cloudInfoStamp.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                }
            }

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

    void publishFeatureCloud()
    {
        rolo::CloudInfoStamp featureInfo = cloudInfoStamp;
        featureInfo.startRingIndex.clear();
        featureInfo.endRingIndex.clear();
        featureInfo.pointColInd.clear();
        featureInfo.pointRange.clear();

        featureInfo.extracted_corner = publishCloud(pubCornerPoints, cornerCloud, cloudHeader.stamp, lidarFrame);
        featureInfo.extracted_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        featureInfo.extracted_normal = publishCloud(pubNormalPoints, normalCloud, cloudHeader.stamp, lidarFrame);

        pubFeatureCloudInfo.publish(featureInfo);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_preprocess");

    ScanPreprocess SP;

    ROS_INFO("\033[1;32m----> Scan Preprocess Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
