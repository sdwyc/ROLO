#include "rolo_sam/utility.h"

#include "rolo_sam/CloudInfoStamp.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D     // 添加XYZ;
    PCL_ADD_INTENSITY;  // 添加inetnsity
    uint16_t ring;      // 添加扫瞄线数
    float time;         // 添加时间戳，扫描到当前这个点所花的时间
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保new操作符内存对齐
} EIGEN_ALIGN16;    // 确保正确的内存分配

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamLoader
{
private:

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubLaserRangeImg;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    cv::Mat rangeMat;   // 将一帧点云平铺，形成一个矩阵，行数为扫瞄线数，列数由水平扫描角度求得

    rolo_sam::CloudInfoStamp cloudInfoStamp;
    double timeScanCur; // 当前帧第一个点扫描的时间
    double timeScanEnd; // 当前帧最后一个点扫描的时间
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;


public:
    //! 初始化输入输出，和点云变量
    ImageProjection()
    {
        // 输入：激光点云原数据
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 10, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        // 输出：cloud_info
        // cloud_info为从去畸变点云中提取的有效点云信息：行列数，距离和坐标，方便后续提取特征
        pubLaserCloudInfo = nh.advertise<rolo_sam::CloudInfoStamp> ("rolo_sam/cloud_info", 1);
        pubLaserRangeImg = nh.advertise<sensor_msgs::Image> ("rolo_sam/range_image", 1);
        // 重置各变量，初始化
        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    ~ImageProjection(){}

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfoStamp.startRingIndex.assign(N_SCAN, 0);
        cloudInfoStamp.endRingIndex.assign(N_SCAN, 0);

        cloudInfoStamp.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfoStamp.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    // 重置各点云变量的初值，标志位
    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        // 距离矩阵为一个以雷电点云线数为行数，每根扫瞄线的点数为列数，元素为到雷达原点的距离
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // 存储点云，转换格式
        if (!cachePointCloud(laserCloudMsg))
            return;
        // 从imu和imu_odom消息中推断雷达运动，为去畸变作准备
        // if (!deskewInfo())
        //     return;
        // 投影到range image，去畸变
        projectPointCloud();
        // 提取有效点的相关信息，方便后续提取特征
        cloudExtraction();
        // 发布点云和cloud_info
        publishClouds();
        // 重置各变量，标志位，为下一帧作准备
        resetParameters();
    }

    //! 对点云进行格式转换和预检查，并存储到queue
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2) // 存储三帧以上点云后，进行后续操作
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front()); // 相当于引用，只不过不用占用新内存
        cloudQueue.pop_front();
        // 根据雷达类型，转换为相应的点云格式
        if (sensor == lidarType::VELODYNE)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == lidarType::OUSTER)
        {
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
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec(); // 当前帧第一个点扫描的时间
        // 扫描完最后一个点的时间
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag 检查点云有效性
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            // 检查点云消息中的fields中是否有ring字段，velodyne雷达消息默认会有
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            // if (ringFlag == -1)
            // {
            //     ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
            //     ros::shutdown();
            // }
        }

        return true;
    }

    //! 将当前帧点云投影到一个range image中，像素值为到雷达坐标系原点的距离，并对所有点进行去畸变操作。
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

            float range = pointDistance(thisPoint); // 到雷达原点的距离
            // 距离滤波
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;
            // 行索引为扫瞄线数
            float angle = atan(laserCloudIn->points[i].z / sqrt(laserCloudIn->points[i].x * laserCloudIn->points[i].x + laserCloudIn->points[i].y * laserCloudIn->points[i].y)) * 180 / M_PI; // 点到基座的俯仰角，单位：degree
            int scanID = 0;
            // 判断一个点属于哪个线上的点，scanID为线数的序列号
            scanID = int((angle + 15) / 2 + 0.5);
            // std::cout << "point ring: " << scanID << std::endl;
            if (scanID > (N_SCAN - 1) || scanID < 0)
            {
                continue;
            }
            int rowIdn = scanID;
            // int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == lidarType::VELODYNE || sensor == lidarType::OUSTER)
            {
                // 以水平方位角划分，得到列索引
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0/float(Horizon_SCAN);
                // 列索引公式，下面式子是以y轴负半轴为0的列索引，使一帧激光点云沿y轴负半轴展开
                columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                // columnIdn = round(horizonAngle/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN; // 为了能够首尾相接
            }
            // else if (sensor == lidarType::LIVOX)
            // {
            //     columnIdn = columnIdnCountVec[rowIdn];
            //     columnIdnCountVec[rowIdn] += 1;
            // }
            
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            // 填充过的像素位置，不再填充新的值
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            // thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
            // rangeMat填充
            rangeMat.at<float>(rowIdn, columnIdn) = range;
            // fullCloud点云由一维数组进行有序排列，索引公式为：c + r * width
            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
        if(pubLaserRangeImg.getNumSubscribers()!=0){
            cv::Mat rangeMatInit;
            rangeMat.convertTo(rangeMatInit, CV_16UC1);
            int row, col;
            row = rangeMatInit.rows;
            col = rangeMatInit.cols;
            cv::resize(rangeMatInit, rangeMatInit, cv::Size(), 1.0, 10.0);
            cv::flip(rangeMatInit, rangeMatInit, 0);
            sensor_msgs::ImagePtr range_img = cv_bridge::CvImage(std_msgs::Header(), "mono16", rangeMatInit).toImageMsg();
            range_img->header.frame_id = "camera";
            range_img->header.stamp = ros::Time::now();
            pubLaserRangeImg.publish(range_img);
        }
    }

    //! 对去畸变后的点云进行提取标记，方便后续提取特征，标记好每条扫瞄线的提取的点的行列和位置信息
    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        // 遍历每条扫瞄线
        for (int i = 0; i < N_SCAN; ++i)
        {
            // 这条扫瞄线可以计算曲率的起始点（计算曲率需要左右各五个点）
            cloudInfoStamp.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfoStamp.pointColInd[count] = j; // 列数信息
                    // save range info
                    cloudInfoStamp.pointRange[count] = rangeMat.at<float>(i,j);  // range信息
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);   // 3维坐标信息
                    // size of extracted cloud
                    // count只在有效点才会累加
                    ++count;
                }
            }
            // 这条扫瞄线可以计算曲率的终点
            cloudInfoStamp.endRingIndex[i] = count -1 - 5;
        }
    }
    //! 发布去畸变点云和cloud_info
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
