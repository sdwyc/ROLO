#include "rolo/utility.h"

#include "rolo/CloudInfoStamp.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D     // XYZ point fields
    PCL_ADD_INTENSITY;  // PCL add intensity
    std::uint16_t ring; // Laser ring count
    float time;         // Extract timestamp
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen alignment
} EIGEN_ALIGN16;    // Memory allocation

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint16_t, ring, ring) (float, time, time)
)

struct VelodynePointXYZIRTRGB
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    PCL_ADD_RGB;
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRTRGB,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, rgb, rgb) (std::uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t noise;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint32_t, t, t) (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring) (std::uint16_t, noise, noise) (std::uint32_t, range, range)
)

struct OusterPointXYZIRTRGB {
    PCL_ADD_POINT4D;
    float intensity;
    PCL_ADD_RGB;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t noise;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRTRGB,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, rgb, rgb) (std::uint32_t, t, t) (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring) (std::uint16_t, noise, noise) (std::uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
#if HasRGB
using PointXYZIRT = VelodynePointXYZIRTRGB;
using OusterInputPointType = OusterPointXYZIRTRGB;
#else
using PointXYZIRT = VelodynePointXYZIRT;
using OusterInputPointType = OusterPointXYZIRT;
#endif

const int queueLength = 2000;

class ImageProjection : public ParamLoader
{
private:

    ros::Subscriber subLaserCloud;
    ros::Subscriber subOdom;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;
    // ros::Publisher pubLaserRangeImg;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    std::deque<nav_msgs::Odometry> odomQueue;
    std::mutex odomLock;
    sensor_msgs::PointCloud2 currentCloudMsg;

    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterInputPointType>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   deskewCloud;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    cv::Mat rangeMat;   // Range image matrix

    rolo::CloudInfoStamp cloudInfoStamp;
    double timeScanCur; // Time update
    double timeScanEnd; // Time update
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;

    std::string timeField; 
    int timeFlag = 0;
    float scanPeriod = 0.1;
    double odomTimeDiff = -1.0;
    float odomIncreX, odomIncreY, odomIncreZ, odomIncreRoll, odomIncrePitch, odomIncreYaw;
    bool odomAvailable = false;

public:
    //! Initialize I/O and point clouds
    ImageProjection()
    {
        // Input point cloud topic
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 10, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        // Feature cloud info input
        // Feature cloud info input
        pubLaserCloudInfo = nh.advertise<rolo::CloudInfoStamp> ("rolo/cloud_info", 1);
        // pubLaserRangeImg = nh.advertise<sensor_msgs::Image> ("rolo/range_image", 1);
        // Initialize state
        allocateMemory();
        resetParameters();
        timeField = "time";
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    ~ImageProjection(){}

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterInputPointType>());
        deskewCloud.reset(new pcl::PointCloud<PointType>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfoStamp.startRingIndex.assign(N_SCAN, 0);
        cloudInfoStamp.endRingIndex.assign(N_SCAN, 0);

        cloudInfoStamp.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfoStamp.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    // Reset clouds and flags
    void resetParameters()
    {
        deskewCloud->clear();
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        // Range matrix layout
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    //! Convert odometry to transform matrix
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom) // Affine transform from rotation and translation
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


    void odometryHandler(const nav_msgs::OdometryConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock2(odomLock);
        odomQueue.push_back(*odomMsg);
        if(odomQueue.size() >= 2)
            odomAvailable = true;
    }


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // Store and convert point cloud
        if (!cachePointCloud(laserCloudMsg)){
            return;
        }
        // IMU data
        if (!deskewCloudInfo()){
            return;
        }
        // Project cloud to range image
        projectPointCloud();
        // Store metadata for feature extraction
        cloudExtraction();
        // Feature cloud info input
        publishClouds();
        // State flag
        resetParameters();
    }

    //! Format conversion
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2) // Need at least three queued clouds
            return false;
        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front()); // Memory allocation
        cloudQueue.pop_front();
        // Convert cloud by lidar type
        if (sensor == lidarType::VELODYNE)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == lidarType::OUSTER)
        {
            timeField = "t";
            // Convert to Velodyne format
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
                // dst.time = src.t * 1.0f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        // scanPeriod = cloudHeader.stamp.toSec() - timeScanCur; 
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec(); // Time update
        // Time update
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // Check cloud dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }
        static int ringFlag =0;
        // check ring channel
        if (ringFlag == 0)
        {
            ringFlag = -1;
            // Laser ring id
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
        // check point time field
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

    //! Deskew current cloud
    bool deskewCloudInfo()
    {
        if(deskewEnabled && odomAvailable){
            int cloudSize = laserCloudIn->points.size();
            if(timeFlag == -1){
                bool halfPassed = false;
                float startOri = -atan2(laserCloudIn->points[0].y,laserCloudIn->points[0].x); 
                float endOri   = -atan2(laserCloudIn->points[cloudSize - 1].y, laserCloudIn->points[cloudSize - 1].x) + 2 * M_PI;
                if (endOri - startOri > 3 * M_PI) {
                    endOri -= 2 * M_PI;
                } else if (endOri - startOri < M_PI)
                    endOri += 2 * M_PI;
                float orientationDiff = endOri - startOri;
                
                PointType point;
                deskewCloud->points.resize(cloudSize);

                // Deskew with front-end odometry interpolation
                // Time update
                while (!odomQueue.empty())
                {
                    // Time update
                    if (fabs(timeScanCur - odomQueue.front().header.stamp.toSec()) > 0.25)
                        odomQueue.pop_front();
                    else
                        break;
                }

                Eigen::Affine3f lidarOdomAffineFront = odom2affine(odomQueue.front());
                Eigen::Affine3f lidarOdomAffineBack = odom2affine(odomQueue.back());
                // Relative lidar odometry transform
                Eigen::Affine3f lidarOdomAffineIncre = lidarOdomAffineFront.inverse() * lidarOdomAffineBack;
                odomTimeDiff = odomQueue.back().header.stamp.toSec() - odomQueue.front().header.stamp.toSec();
                pcl::getTranslationAndEulerAngles(lidarOdomAffineIncre, odomIncreX, odomIncreY, odomIncreZ, odomIncreRoll, odomIncrePitch, odomIncreYaw);
                
                for (int i = 0; i < cloudSize; i++) {

                    point.x = laserCloudIn->points[i].y;
                    point.y = laserCloudIn->points[i].z;
                    point.z = laserCloudIn->points[i].x;
#if HasRGB
                    point.rgb = laserCloudIn->points[i].rgb;
#endif

                    float ori = -atan2(point.x, point.z);
                    if (!halfPassed) {
                        if (ori < startOri - M_PI / 2)
                            ori += 2 * M_PI;
                        else if (ori > startOri + M_PI * 3 / 2)
                            ori -= 2 * M_PI;

                        if (ori - startOri > M_PI)
                            halfPassed = true;
                    } else {
                        ori += 2 * M_PI;

                        if (ori < endOri - M_PI * 3 / 2)
                            ori += 2 * M_PI;
                        else if (ori > endOri + M_PI / 2)
                            ori -= 2 * M_PI;
                    }
                    float relTime = (ori - startOri) / orientationDiff;
                    point.intensity = scanPeriod * relTime;
                    deskewCloud->points[i] = point;
                }
            }
            else{
                // Extract timestamp
                
                PointType point;
                deskewCloud->points.resize(cloudSize);

                // Deskew with front-end odometry interpolation
                // Time update
                while (!odomQueue.empty())
                {
                    // Time update
                    if (fabs(timeScanCur - odomQueue.front().header.stamp.toSec()) > 0.3) // Need enough queued clouds
                        odomQueue.pop_front();
                    else
                        break;
                }

                Eigen::Affine3f lidarOdomAffineFront = odom2affine(odomQueue.front());
                Eigen::Affine3f lidarOdomAffineBack = odom2affine(odomQueue.back());
                // Relative lidar odometry transform
                Eigen::Affine3f lidarOdomAffineIncre = lidarOdomAffineFront.inverse() * lidarOdomAffineBack;
                odomTimeDiff = odomQueue.back().header.stamp.toSec() - odomQueue.front().header.stamp.toSec();
                pcl::getTranslationAndEulerAngles(lidarOdomAffineIncre, odomIncreX, odomIncreY, odomIncreZ, odomIncreRoll, odomIncrePitch, odomIncreYaw);
                
                for (int i = 0; i < cloudSize; i++) {

                    point.x = laserCloudIn->points[i].y;
                    point.y = laserCloudIn->points[i].z;
                    point.z = laserCloudIn->points[i].x;
#if HasRGB
                    point.rgb = laserCloudIn->points[i].rgb;
#endif

                    float relTime = fabs(laserCloudIn->points[i].time);
                    point.intensity = relTime;
                    deskewCloud->points[i] = point;
                }
            }
        }
        return true;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        // Skip if deskew is unavailable
        if (!deskewEnabled || odomAvailable == false)
            return *point;

        // transform points to start
        float ratio = relTime/scanPeriod;
        Eigen::Matrix<float, 6, 1> trans;
        trans.col(0) << odomIncreX, odomIncreY, odomIncreZ, odomIncreRoll, odomIncrePitch, odomIncreYaw;
        trans = trans.eval() * (scanPeriod/odomTimeDiff) * ratio;
        // trans = trans.eval() * ratio;
        // Eigen::Affine3f transBt = pcl::getTransformation(-trans(0), -trans(1), -trans(2), -trans(3), -trans(4), -trans(5));
        Eigen::Affine3f transBt = pcl::getTransformation(0.0, 0.0, 0.0, -trans(3), -trans(4), -trans(5));

        // Deskew point cloud
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

    //! Project cloud to range image
    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // thisPoint.intensity = laserCloudIn->points[i].intensity;
            thisPoint.intensity = laserCloudIn->points[i].ring*laserCloudIn->points[i].z;
#if HasRGB
            thisPoint.rgb = laserCloudIn->points[i].rgb;
#endif

            float range = pointDistance(thisPoint); // Range to lidar origin
            // Distance check
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;
            // // Index handling
            // float angle = atan(laserCloudIn->points[i].z / sqrt(laserCloudIn->points[i].x * laserCloudIn->points[i].x + laserCloudIn->points[i].y * laserCloudIn->points[i].y)) * 180 / M_PI; // Y
            // int scanID = 0;
            // // Range filter
            // scanID = int((angle + 15) / 2 + 0.5);
            // // std::cout << "point ring: " << scanID << std::endl;
            // if (scanID > (N_SCAN - 1) || scanID < 0)
            // {
            //     continue;
            // }
            // int rowIdn = scanID;
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == lidarType::VELODYNE || sensor == lidarType::OUSTER)
            {
                // Index handling
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0/float(Horizon_SCAN);
                // Point index
                columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                // columnIdn = round(horizonAngle/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN; // Horizon scan
            }
            
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            // Wrap column index
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;
            //TODO deskewPoint Refinement, temporary forbid
            // thisPoint = deskewPoint(&thisPoint, deskewCloud->points[i].intensity);
            // Range image
            rangeMat.at<float>(rowIdn, columnIdn) = range;
            // Point index
            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
        // if(pubLaserRangeImg.getNumSubscribers()!=0){
        //     cv::Mat rangeMatInit;
        //     rangeMat.convertTo(rangeMatInit, CV_16UC1);
        //     int row, col;
        //     row = rangeMatInit.rows;
        //     col = rangeMatInit.cols;
        //     cv::resize(rangeMatInit, rangeMatInit, cv::Size(), 1.0, 10.0);
        //     cv::flip(rangeMatInit, rangeMatInit, 0);
        //     sensor_msgs::ImagePtr range_img = cv_bridge::CvImage(std_msgs::Header(), "mono16", rangeMatInit).toImageMsg();
        //     range_img->header.frame_id = "camera";
        //     range_img->header.stamp = ros::Time::now();
        //     pubLaserRangeImg.publish(range_img);
        // }
    }

    //! Mark deskewed cloud for feature extraction
    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        // Mark deskewed cloud for feature extraction
        for (int i = 0; i < N_SCAN; ++i)
        {
            // Point smoothness
            cloudInfoStamp.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfoStamp.pointColInd[count] = j; // J
                    // save range info
                    cloudInfoStamp.pointRange[count] = rangeMat.at<float>(i,j);  // J
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);   // Column index info
                    // size of extracted cloud
                    // Column index info
                    ++count;
                }
            }
            // Point smoothness
            cloudInfoStamp.endRingIndex[i] = count -1 - 5;
        }
    }
    //! Feature cloud info input
    void publishClouds()
    {
        cloudInfoStamp.header = cloudHeader;
        cloudInfoStamp.cloud_projected  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfoStamp);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_projection");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
