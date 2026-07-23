#include "utility.hpp"
// #include "rolo/save_map.h"
#include "rolo/pose_solver.hpp"
#include "scancontext/Scancontext.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <ros/package.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/registration/icp.h>

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

enum class SCInputType {
    SCAN_RAW,
    SCAN_FEAT
};

SCInputType ParseSCInputType(const std::string &sc_input_type)
{
    if (sc_input_type == "scan_feat")
        return SCInputType::SCAN_FEAT;
    return SCInputType::SCAN_RAW;
}


class backMapping : public ParamLoader
{
public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;    // GTSAM factor graph
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;    // Incremental nonlinear optimizer
    Values isamCurrentEstimate; // Current pose estimate
    Eigen::MatrixXd poseCovariance; // Current pose covariance

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
    ros::Publisher pubGlobalGraph;
    ros::Publisher pubLoopConstraintEdge;
    ros::Publisher pubPriorPredictions;
    ros::Publisher pubPriorPoseHistory;
    ros::Publisher pubPriorPatches;
    ros::Publisher pubCurrentPatch;

    ros::Publisher pubSLAMInfo;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;
    ros::Subscriber subPriorPose;
    ros::Subscriber subGroundMap;

    ros::ServiceServer srvSaveMap;

    ground_factor::GroundModel groundCloudModelFromlaser;
    ground_factor::VehicleModel priorVehicleModel;
    pcl::PointCloud<GroundPatchType>::Ptr GroundCloudFromlaser;

    std::deque<nav_msgs::Odometry> gpsQueue;
    rolo::CloudInfoStamp cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;   // Downsampled keyframe corner clouds
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames; // Downsampled keyframe surface clouds
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;    // Keyframe positions
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;// Keyframe 6D poses
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr laserCloudRaw;
    pcl::PointCloud<PointType>::Ptr laserCloudRawDS;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // Current corner cloud
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // Current surface cloud
    pcl::PointCloud<PointType>::Ptr laserCloudNormalLast;     // Current normal cloud
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // Current corner cloud
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;   // Current surface cloud

    pcl::PointCloud<PointType>::Ptr laserCloudOri;  // Selected feature points
    pcl::PointCloud<PointType>::Ptr coeffSel;       // Selected feature coefficients

    std::vector<PointType> laserCloudOriCornerVec; // Point-to-line candidates
    std::vector<PointType> coeffSelCornerVec;   // Normalize point direction
    std::vector<bool> laserCloudOriCornerFlag;  // Valid point-to-line mask
    std::vector<PointType> laserCloudOriSurfVec; // Point-to-plane candidates
    std::vector<PointType> coeffSelSurfVec;     // Point-to-plane candidates
    std::vector<bool> laserCloudOriSurfFlag;    // Valid point-to-plane mask

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer; // Keyframe corner and surface map
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;    // Surrounding corner map
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;      // Surrounding surface map
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;  // Downsampled surrounding map
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterSC;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];   // Global lidar pose
    float priorPoseCur[6];          // Current matched prior pose

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;
    bool aPriorPose = false;
    deque<pair<std::array<double, 6>, pcl::PointCloud<GroundPatchType>>> priorPosePatchQueue;
    deque<pair<double, int>> priorTimeKeyQueue;
    vector<pair<int, int>> priorIndexQueue;  // Matched prior loop pair
    map<int, pair<int, std::array<float, 6>>> priorVisContainer; // key: current key, value.first: linked key, value.second: linked_pose -> prior_pose relative transform
    vector<gtsam::Pose3> priorPoseQueue; // Matched prior pose transform
    vector<gtsam::noiseModel::Diagonal::shared_ptr> priorNoiseQueue; // Matched prior noise model


    bool aLoopIsClosed = false; // Loop factor added flag
    map<int, int> loopIndexContainer; // Loop pair index map
    vector<pair<int, int>> loopIndexQueue;  // Matched prior loop pair
    vector<gtsam::Pose3> loopPoseQueue; // Matched prior pose transform
    vector<gtsam::SharedNoiseModel> loopNoiseQueue; // Matched prior noise model
    deque<std_msgs::Float64MultiArray> loopInfoVec; // External loop pair buffer

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront; // Previous global odometry pose
    Eigen::Affine3f incrementalOdometryAffineBack;
    SCManager scManager;

    backMapping(){
        // ISAM optimizer
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters); // Create ISAM optimizer

        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/trajectory", 1);  // Global trajectory publisher
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/map_global", 1);  // Global map publisher
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("rolo/mapping/odometry", 1);         // Odometry publisher
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("rolo/mapping/odometry_incremental", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("rolo/mapping/path", 1);  // Global trajectory publisher
        // ISAM parameters
        subCloud = nh.subscribe<rolo::CloudInfoStamp>(odomTopic+"/cloud_info", 1, &backMapping::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subPriorPose = nh.subscribe<rolo::CloudInfoStamp>("vehicle_prior_info", 1, &backMapping::priorInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGroundMap = nh.subscribe<sensor_msgs::PointCloud2>(priorPoseNodePcdTopic, 1, &backMapping::groundMapHandler, this, ros::TransportHints().tcpNoDelay());

        // Feature cloud info input
        // subLoop  = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &backMapping::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // srvSaveMap  = nh.advertiseService("rolo/save_map", &backMapping::saveMapService, this);

        // Loop closure data
        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/icp_loop_closure_history_cloud", 1);
        // Historical keyframe clouds
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/icp_loop_closure_corrected_cloud", 1);
        pubGlobalGraph        = nh.advertise<visualization_msgs::MarkerArray>("rolo/mapping/global_graph", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/rolo/mapping/loop_closure_constraints", 1);
        pubPriorPredictions   = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/rolo/mapping/prior_predictions", 1);
        pubPriorPoseHistory   = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/rolo/mapping/prior_pose_history", 1);
        pubPriorPatches       = nh.advertise<sensor_msgs::PointCloud2>("/rolo/mapping/prior_patches", 1);
        pubCurrentPatch       = nh.advertise<sensor_msgs::PointCloud2>("extracted_patch_current", 1);
        // Associated keyframe clouds
        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/map_local", 1);
        // Local map
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("rolo/mapping/cloud_registered_raw", 1);

        pubSLAMInfo           = nh.advertise<rolo::CloudInfoStamp>("rolo/mapping/slam_info", 1);

        const float kSCFilterSize = 0.5f;
        downSizeFilterSC.setLeafSize(kSCFilterSize, kSCFilterSize, kSCFilterSize);
        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        if (!priorWheelXY.empty())
            priorVehicleModel = ground_factor::VehicleModel(priorWheelXY, priorVehicleComZ,
                                                            priorLidarOffsetTrans, priorLidarOffsetRot);
        else
            priorVehicleModel = ground_factor::VehicleModel::FromSquare(priorVehicleSizeXY, priorVehicleComZ,
                                                                        priorLidarOffsetTrans, priorLidarOffsetRot);

        if (savePCD)
        {
            const std::string pkg_path = ros::package::getPath("rolo");
            if (pkg_path.empty())
            {
                ROS_ERROR("Failed to resolve ROLO package path for saving PCDs.");
            }
            else
            {
                if (!savePCDDirectory.empty() && savePCDDirectory.front() == '/')
                    savePCDDirectory = pkg_path + savePCDDirectory;
                else
                    savePCDDirectory = pkg_path + "/" + savePCDDirectory;

                if (!savePCDDirectory.empty() && savePCDDirectory.back() != '/')
                    savePCDDirectory += "/";

                if (!boost::filesystem::exists(savePCDDirectory))
                    boost::filesystem::create_directories(savePCDDirectory);

                const std::string keyframes_dir = savePCDDirectory + "keyframes/";
                if (!boost::filesystem::exists(keyframes_dir))
                    boost::filesystem::create_directories(keyframes_dir);
            }
        }

        // Allocate and reset buffers
        allocateMemory();
    }

    //! Clamp input value
    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    //! Transform point to map frame
    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
#if HasRGB
        po->rgb = pi->rgb;
#endif
    }
    //! Apply point cloud transform
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);
        // Build transform matrix
        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        // Parallelize the next loop
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
#if HasRGB
            cloudOut->points[i].rgb = pointFrom.rgb;
#endif
        }
        return cloudOut;
    }
    void allocateMemory(){
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        laserCloudRaw.reset(new pcl::PointCloud<PointType>());
        laserCloudRawDS.reset(new pcl::PointCloud<PointType>());

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
        GroundCloudFromlaser.reset(new pcl::PointCloud<GroundPatchType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
            priorPoseCur[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void groundMapHandler(const sensor_msgs::PointCloud2ConstPtr& msgIn)
    {
        groundCloudModelFromlaser.UpdateFromCloud(*msgIn, false);
        if (GroundCloudFromlaser == nullptr) {
            GroundCloudFromlaser.reset(new pcl::PointCloud<GroundPatchType>());
        }

        if (!groundCloudModelFromlaser.ExtractPatch(Eigen::Vector2d::Zero(),
                                                    static_cast<double>(groundPatchSize),
                                                    GroundCloudFromlaser)) {
            GroundCloudFromlaser->clear();
        }

        publishCloud(pubCurrentPatch, GroundCloudFromlaser, ros::Time(msgIn->header.stamp), lidarFrame);
    }

    //! Lidar callback
    void laserCloudInfoHandler(const rolo::CloudInfoStampConstPtr& msgIn){
        // Extract timestamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        // Extract current feature clouds
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->extracted_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->extracted_surface, *laserCloudSurfLast);
        pcl::fromROSMsg(msgIn->extracted_normal, *laserCloudNormalLast);
        *laserCloudSurfLast += *laserCloudNormalLast;

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        // Run backend only after the time threshold
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            // Update timestamp
            timeLastProcessing = timeLaserInfoCur;
            // Use front-end pose as prior
            updateInitialGuess();
            // Extract surrounding keyframes
            extractSurroundingKeyFrames();
            // Downsample current features
            downsampleCurrentScan();
            // Scan-to-submap registration
            scan2MapOptimization();
            // Optimize pose with feature constraints
            saveKeyFramesAndFactor();
            // Add factors and save optimized state
            correctPoses();
            // Update history after loop closure
            publishOdometry();
            // Publish odometry and TF
            publishFrames();
        }
    }

    void priorInfoHandler(const rolo::CloudInfoStampConstPtr& msgIn)
    {
        const double msgTime = msgIn->header.stamp.toSec();
        double latestKeyTime = 0.0;
        int latestKeyID = -1;
        mtx.lock();
        if (!cloudKeyPoses6D->empty())
        {
            latestKeyTime = cloudKeyPoses6D->back().time;
            latestKeyID = static_cast<int>(cloudKeyPoses6D->size()) - 1;
        }
        mtx.unlock();
        if (latestKeyID < 0)
            return;

        // printf("time diff: %f \n", std::abs(msgIn->header.stamp.toSec() - latestKeyTime));
        if (latestKeyID <= 9 || std::abs(msgIn->header.stamp.toSec() - latestKeyTime) >= 1e-2)
        // if (std::abs(msgIn->header.stamp.toSec() - latestKeyTime) >= 1e-2)
            return;

        mtx.lock();
        if (!priorTimeKeyQueue.empty() && priorSyncedInterval > 0.0f)
        {
            const double lastPriorTime = priorTimeKeyQueue.back().first;
            if (msgTime - lastPriorTime < static_cast<double>(priorSyncedInterval))
            {
                mtx.unlock();
                return;
            }
        }
        mtx.unlock();

        priorPoseCur[0] = msgIn->initialGuessRoll;
        priorPoseCur[1] = msgIn->initialGuessPitch;
        priorPoseCur[2] = msgIn->initialGuessYaw;
        priorPoseCur[3] = msgIn->initialGuessX;
        priorPoseCur[4] = msgIn->initialGuessY;
        priorPoseCur[5] = msgIn->initialGuessZ;

        pcl::PointCloud<GroundPatchType> prior_ground_patch;
        pcl::fromROSMsg(msgIn->extracted_ground, prior_ground_patch);

        std::array<double, 6> prior_pose = {
            static_cast<double>(priorPoseCur[0]),
            static_cast<double>(priorPoseCur[1]),
            static_cast<double>(priorPoseCur[2]),
            static_cast<double>(priorPoseCur[3]),
            static_cast<double>(priorPoseCur[4]),
            static_cast<double>(priorPoseCur[5]),
        };
        mtx.lock();
        priorPosePatchQueue.push_back(std::make_pair(prior_pose, prior_ground_patch));
        priorTimeKeyQueue.push_back(std::make_pair(msgTime, latestKeyID));
        mtx.unlock();
    }

    void updateInitialGuess(){
        // save current transformation before any processing
    //! Initial pose from odometry fusion
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[3] = initPose[0];
            transformTobeMapped[4] = initPose[1];
            transformTobeMapped[5] = initPose[2];

            transformTobeMapped[0] = initPose[3];
            transformTobeMapped[1] = initPose[4];
            transformTobeMapped[2] = initPose[5];
            return;
        }

        // use LiDAR odometry estimation for pose guess
        static bool lastOdomTransAvailable = false; 
        static Eigen::Affine3f lastOdomTransformation;
        if (cloudInfo.odomAvailable == true)    // Odometry pose available
        {   // Odometry available branch
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastOdomTransAvailable == false)
            {
                lastOdomTransformation = transBack;
                lastOdomTransAvailable = true;
            } else { // Build transform matrix
                Eigen::Affine3f transIncre = lastOdomTransformation.inverse() * transBack; // Trans back
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastOdomTransformation = transBack;
                return;
            }
        }
    }

    //! Extract surrounding keyframes
    void extractSurroundingKeyFrames()
    {
        // Keyframe processing
        if (cloudKeyPoses3D->points.empty() == true)
            return;
        
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        extractNearby();  // Extract surrounding keyframes
    }

    //! Surface cloud
    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());  // Keyframe processing
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());// Keyframe processing
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        // Nearby keyframe pose cloud
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]); // Keyframe processing
        }
        // Keyframe processing
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        // Intensity field
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            // Intensity field
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        // Point intensity stores cloud index
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }
        // Surface cloud
        extractCloud(surroundingKeyPosesDS);
    }

    //! Surface cloud
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        // Find nearest neighbors with KD-tree
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            // Distance check
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity; // Index handling
            // Keyframe processing
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // Keyframe processing
                // transformed cloud available
                // Surrounding corner map
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // Keyframe processing
                // transformed cloud not available
                // Get nearby corner and surface clouds
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

    //! Downsample current features
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

    //! Scan-to-submap registration
    void scan2MapOptimization()
    {
    //! Optimize pose with feature constraints
        if (cloudKeyPoses3D->points.empty())
            return;
        // Require enough current-frame features
        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
            // Run scan-to-map iterations
            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();
                // Find valid point-to-line matches
                cornerOptimization();
                // Find valid point-to-plane matches
                surfOptimization();
                // Merge optimization coefficients
                combineOptimizationCoeffs();
                // Run LM pose optimization
                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    //! Update point-to-map transform
    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    //! Transform pose
    void cornerOptimization()
    {
        updatePointAssociateToMap();    // Matrix update

        #pragma omp parallel for num_threads(numberOfCores) // Thread count
        for (int i = 0; i < laserCloudCornerLastDSNum; i++) // Downsampled current corner cloud
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            // Traverse downsampled current corners
            pointAssociateToMap(&pointOri, &pointSel);
            // Traverse downsampled corner points
            // Find nearest neighbors with KD-tree
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0)); // Current pose covariance
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0)); // Matrix update
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0)); // Matrix update
                    
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5; // Cz
                // Current pose covariance
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

                cv::eigen(matA1, matD1, matV1); // Mat v1

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) { // At
                // Compute eigenvalues and eigenvectors
                    // Compute eigenvalues and eigenvectors
                    //        A
                    //   B        C
                    // Build three points for line fitting
                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    // Point A and two line points
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    // Point A coordinates
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);
                    // Point B coordinates
                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
                    // Point C coordinates
                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
                    // Normalize point direction
                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12; // Distance check
                    // Point-to-line unit direction
                    float s = 1 - 0.9 * fabs(ld2);
                    // Point-to-line distance
                    // Distance check
                    // Intensity field
                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    // Distance check
                    // Kernel weight from residual distance
                    if (s > 0.1) {
                        // Kernel weight from residual distance
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    //! Surface cloud
    void surfOptimization()
    {
        // Transform pose
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores) // Number of cores
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            // Parallel processing
            pointAssociateToMap(&pointOri, &pointSel); 
            // Fit local plane
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
                // Normal cloud
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                // Normalize point direction
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                // Distance check
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    // Surface cloud
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));
                    // Intensity field
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;
                    // Distance check
                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    //! Accept points near the plane
    void combineOptimizationCoeffs()
    {
        // Select corner features
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // Surface cloud
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        // Extract valid surface matches
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }
    //! Reset match masks
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
        // Skip optimization with too few features
        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }
        // Jacobian matrix
        // Matrix update
        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0)); // Matrix update
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));// All
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));   // All
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0)); // Matrix update
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));   // All
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0)); // Matrix update

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
            // Jacobian matrix
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
            matB.at<float>(i, 0) = -coeff.intensity; // Intensity
        }

        // Negative sign for GN solve
        // Negative sign for GN solve
        // Hessian matrix
        // Negative sign for GN solve
        cv::transpose(matA, matAt); // Mat at
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {   // Initialize state
            // Hessian matrix
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0)); // Hessian matrix
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0)); // Hessian matrix
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);  // Hessian matrix
            matV.copyTo(matV2);
            // Hessian eigenvalues
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
        // Update current pose by delta
        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);
        // Update current pose by delta
        float deltaR = sqrt(
                            pow(radTodeg(matX.at<float>(0, 0)), 2) +
                            pow(radTodeg(matX.at<float>(1, 0)), 2) +
                            pow(radTodeg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));
        // Compute pose increments
        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void transformUpdate()
    {
    //! Optimize pose with feature constraints
        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

        // Clamp roll, pitch, and z
    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty()) // Keyframe processing
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());   // Current frame keyframe check
    //! Keyframe processing
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;   // Matrix update
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
        // Optimize pose with feature constraints
        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    //! GPS data
    void saveKeyFramesAndFactor()
    {
    //! Current keyframe
        if (saveFrame() == false)
            return;

        // odom factor
        // Keyframe processing
        addOdomFactor();

        // loop factor
        // Add prior or odometry factor
        addLoopFactor();

        // prior factor
        addPriorFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);  // Initial estimate
        isam->update(); // Update

        if (aLoopIsClosed == true) // Loop closure
        {
            // Add factors and initial values to ISAM
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }
        // Run optimization
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();    // Current pose estimate
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");
        // Current optimized pose estimate
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);
        // Save optimized state to cloudKeyPoses3D
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
        // Current pose covariance
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        // Current covariance matrix
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
        // Surface cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        const SCInputType currentSCInputType = ParseSCInputType(scInputType);
        if (currentSCInputType == SCInputType::SCAN_RAW)
        {
            laserCloudRaw->clear();
            laserCloudRawDS->clear();
            if (!cloudInfo.cloud_projected.data.empty())
            {
                pcl::fromROSMsg(cloudInfo.cloud_projected, *laserCloudRaw);
                if (!laserCloudRaw->empty())
                {
                    downSizeFilterSC.setInputCloud(laserCloudRaw);
                    downSizeFilterSC.filter(*laserCloudRawDS);
                    if (!laserCloudRawDS->empty())
                    {
                        pcl::PointCloud<SCPointType> scCloud;
                        pcl::copyPointCloud(*laserCloudRawDS, scCloud);
                        scManager.makeAndSaveScancontextAndKeys(scCloud);
                    }
                    else
                        ROS_WARN_THROTTLE(5.0, "SC loop skipped: downsampled cloud_projected is empty.");
                }
                else
                {
                    ROS_WARN_THROTTLE(5.0, "SC loop skipped: cloud_projected is empty.");
                }
            }
            else
            {
                ROS_WARN_THROTTLE(5.0, "SC loop skipped: cloud_projected message has no data.");
            }
        }
        else
        {
            if (!thisSurfKeyFrame->empty())
            {
                pcl::PointCloud<SCPointType> scCloud;
                pcl::copyPointCloud(*thisSurfKeyFrame, scCloud);
                scManager.makeAndSaveScancontextAndKeys(scCloud);
            }
            else
                ROS_WARN_THROTTLE(5.0, "SC loop skipped: current surface feature cloud is empty.");
        }

        // save path for visualization
        // Update path visualization
        updatePath(thisPose6D);
    }

    //! Update path visualization
    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty()) // Keyframe processing
        {
            // Add prior or odometry factor
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            // Add prior or odometry factor
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            // Initial keyframe prior
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            // Insert optimized value
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            // Insert optimized value
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty()) // Loop closure
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i) // Loop closure
        {
            int indexFrom = loopIndexQueue[i].first; // First
            int indexTo = loopIndexQueue[i].second;  // Second
            gtsam::Pose3 poseBetween = loopPoseQueue[i];  // Transform pose
            gtsam::SharedNoiseModel noiseBetween = loopNoiseQueue[i];
            // Loop closure
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;   // Loop factor added flag
    }

    void addPriorFactor()
    {
        if (priorIndexQueue.empty()) // Loop closure
            return;

        for (int i = 0; i < (int)priorIndexQueue.size(); ++i) // Loop closure
        {
            int indexFrom = priorIndexQueue[i].first; // First
            int indexTo = priorIndexQueue[i].second;  // Second
            gtsam::Pose3 poseBetween = priorPoseQueue[i];  // Transform pose
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = priorNoiseQueue[i];

            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        priorIndexQueue.clear();
        priorPoseQueue.clear();
        priorNoiseQueue.clear();
    }

    //! Prior pose transform
    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true) // Update historical keyframe poses after loop closure
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            // Correct history after loop closure
            globalPath.poses.clear();
            // update key poses
            // Correct historical poses after loop closure
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
                // Append pose to path
                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false; // Loop closure
        }
    }

    //! Append pose to path
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

    // Publish odometry and TF
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

        // Publish odometry for ROS (incremental)   // Incremental
        static bool lastIncreOdomPubFlag = false;  // State flag
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            // Transform pose
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);

            // Transform pose
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

    //! Publish related point clouds
    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        // Keyframe processing
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        // Surrounding surface map
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            // Surface cloud
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            // Extract current feature clouds
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_projected, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            // Publish current feature cloud
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        // Publish data
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

    //! Global map visualization thread
    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            // Global map publisher
            publishGlobalMap();
            publishGlobalGraph();
            publishePriorPoseHistory();
        }

        if (savePCD == false)
            return;

        saveGlobalPCDs();
    }

    std::string zeroPadIndex(int idx) const
    {
        std::ostringstream oss;
        oss << std::setw(6) << std::setfill('0') << idx;
        return oss.str();
    }

    void writeG2OVertex(std::ostream &os, int node_idx, const gtsam::Pose3 &pose) const
    {
        const gtsam::Point3 t = pose.translation();
        const gtsam::Rot3 R = pose.rotation();
        os << "VERTEX_SE3:QUAT " << node_idx << " "
           << t.x() << " " << t.y() << " " << t.z() << " "
           << R.toQuaternion().x() << " " << R.toQuaternion().y() << " "
           << R.toQuaternion().z() << " " << R.toQuaternion().w() << "\n";
    }

    void writeG2OEdge(std::ostream &os, int from_idx, int to_idx, const gtsam::Pose3 &relative_pose) const
    {
        const gtsam::Point3 t = relative_pose.translation();
        const gtsam::Rot3 R = relative_pose.rotation();
        os << "EDGE_SE3:QUAT " << from_idx << " " << to_idx << " "
           << t.x() << " " << t.y() << " " << t.z() << " "
           << R.toQuaternion().x() << " " << R.toQuaternion().y() << " "
           << R.toQuaternion().z() << " " << R.toQuaternion().w() << "\n";
    }

    void saveGlobalPCDs()
    {
        pcl::PointCloud<PointType>::Ptr keyPoses3DCopy(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointTypePose>::Ptr keyPoses6DCopy(new pcl::PointCloud<PointTypePose>());
        std::vector<pcl::PointCloud<PointType>::Ptr> cornerKeyFramesCopy;
        std::vector<pcl::PointCloud<PointType>::Ptr> surfKeyFramesCopy;
        std::map<int, int> loopIndexContainerCopy;
        std::map<int, std::pair<int, std::array<float, 6>>> priorVisContainerCopy;

        mtx.lock();
        if (cloudKeyPoses6D->points.empty())
        {
            mtx.unlock();
            return;
        }

        *keyPoses3DCopy = *cloudKeyPoses3D;
        *keyPoses6DCopy = *cloudKeyPoses6D;
        cornerKeyFramesCopy = cornerCloudKeyFrames;
        surfKeyFramesCopy = surfCloudKeyFrames;
        loopIndexContainerCopy = loopIndexContainer;
        priorVisContainerCopy = priorVisContainer;
        mtx.unlock();

        const std::string keyframes_dir = savePCDDirectory + "keyframes/";

        const size_t num_keyframes = std::min(keyPoses3DCopy->size(),
                                              std::min(keyPoses6DCopy->size(),
                                                       std::min(cornerKeyFramesCopy.size(), surfKeyFramesCopy.size())));
        if (num_keyframes == 0)
            return;

        pcl::PointCloud<PointType>::Ptr trajectoryCopy(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointTypePose>::Ptr transformationsCopy(new pcl::PointCloud<PointTypePose>());
        trajectoryCopy->points.assign(keyPoses3DCopy->points.begin(), keyPoses3DCopy->points.begin() + num_keyframes);
        transformationsCopy->points.assign(keyPoses6DCopy->points.begin(), keyPoses6DCopy->points.begin() + num_keyframes);
        trajectoryCopy->width = trajectoryCopy->points.size();
        trajectoryCopy->height = 1;
        trajectoryCopy->is_dense = keyPoses3DCopy->is_dense;
        transformationsCopy->width = transformationsCopy->points.size();
        transformationsCopy->height = 1;
        transformationsCopy->is_dense = keyPoses6DCopy->is_dense;

        pcl::io::savePCDFileBinary(savePCDDirectory + "trajectory.pcd", *trajectoryCopy);
        pcl::io::savePCDFileBinary(savePCDDirectory + "transformations.pcd", *transformationsCopy);

        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (size_t i = 0; i < num_keyframes; ++i)
        {
            pcl::PointCloud<PointType>::Ptr keyframeCloud(new pcl::PointCloud<PointType>());
            *keyframeCloud += *cornerKeyFramesCopy[i];
            *keyframeCloud += *surfKeyFramesCopy[i];
            pcl::io::savePCDFileBinary(keyframes_dir + zeroPadIndex(static_cast<int>(i)) + ".pcd",
                                       *keyframeCloud);

            *globalMapCloud += *transformPointCloud(keyframeCloud, &keyPoses6DCopy->points[i]);
        }
        pcl::io::savePCDFileBinary(savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);

        std::ofstream g2o_file(savePCDDirectory + "global_graph.g2o");
        if (!g2o_file.is_open())
        {
            ROS_ERROR_STREAM("Failed to open g2o file in '" << savePCDDirectory << "'.");
            return;
        }

        for (size_t i = 0; i < num_keyframes; ++i)
            writeG2OVertex(g2o_file, static_cast<int>(i), pclPointTogtsamPose3(transformationsCopy->points[i]));

        for (size_t i = 1; i < num_keyframes; ++i)
        {
            const gtsam::Pose3 prev_pose = pclPointTogtsamPose3(transformationsCopy->points[i - 1]);
            const gtsam::Pose3 cur_pose = pclPointTogtsamPose3(transformationsCopy->points[i]);
            writeG2OEdge(g2o_file, static_cast<int>(i - 1), static_cast<int>(i), prev_pose.between(cur_pose));
        }

        for (const auto &loop_pair : loopIndexContainerCopy)
        {
            const int key_cur = loop_pair.first;
            const int key_pre = loop_pair.second;
            if (key_cur < 0 || key_pre < 0 ||
                key_cur >= static_cast<int>(num_keyframes) ||
                key_pre >= static_cast<int>(num_keyframes))
            {
                continue;
            }
            const gtsam::Pose3 pose_cur = pclPointTogtsamPose3(transformationsCopy->points[key_cur]);
            const gtsam::Pose3 pose_pre = pclPointTogtsamPose3(transformationsCopy->points[key_pre]);
            writeG2OEdge(g2o_file, key_cur, key_pre, pose_cur.between(pose_pre));
        }

        for (const auto &prior_pair : priorVisContainerCopy)
        {
            const int key_cur = prior_pair.first;
            const int key_linked = prior_pair.second.first;
            if (key_cur < 0 || key_linked < 0 ||
                key_cur >= static_cast<int>(num_keyframes) ||
                key_linked >= static_cast<int>(num_keyframes))
            {
                continue;
            }

            const gtsam::Pose3 linked_pose_gtsam = pclPointTogtsamPose3(transformationsCopy->points[key_linked]);
            const gtsam::Pose3 current_pose_gtsam = pclPointTogtsamPose3(transformationsCopy->points[key_cur]);
            writeG2OEdge(g2o_file, key_linked, key_cur, linked_pose_gtsam.between(current_pose_gtsam));
        }

        cout << "Saved global outputs to " << savePCDDirectory << endl;
    }

    //! Global map publisher
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
        // Show global poses in fixed range
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        // Keyframe processing
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        // Keyframe processing
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
            // Find feature cloud for keyframe pose
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // Transform to global frame
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }

    void publishGlobalGraph()
    {
        if (pubGlobalGraph.getNumSubscribers() == 0)
            return;

        pcl::PointCloud<PointTypePose>::Ptr graphKeyPoses(new pcl::PointCloud<PointTypePose>());
        std::map<int, int> loopIndexContainerCopy;
        std::map<int, std::pair<int, std::array<float, 6>>> priorVisContainerCopy;
        ros::Time markerStamp;

        mtx.lock();
        if (cloudKeyPoses6D->points.empty())
        {
            mtx.unlock();
            return;
        }
        *graphKeyPoses = *cloudKeyPoses6D;
        loopIndexContainerCopy = loopIndexContainer;
        priorVisContainerCopy = priorVisContainer;
        markerStamp = timeLaserInfoStamp;
        mtx.unlock();

        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.header.frame_id = odometryFrame;
        clearMarker.header.stamp = markerStamp;
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        markerArray.markers.push_back(clearMarker);

        const float nodeScale = 0.30f;
        const float axisLength = 0.60f;
        const float axisWidth = 0.05f;
        const float edgeWidth = 0.10f;
        const float factorScale = 0.22f;

        auto makePoint = [](float x, float y, float z) {
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            return p;
        };

        auto makeListMarker =
            [&](const std::string &ns, int id, int type, float scale_x, float scale_y, float scale_z,
                float r, float g, float b, float a) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = odometryFrame;
                marker.header.stamp = markerStamp;
                marker.ns = ns;
                marker.id = id;
                marker.action = visualization_msgs::Marker::ADD;
                marker.type = type;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = scale_x;
                marker.scale.y = scale_y;
                marker.scale.z = scale_z;
                marker.color.r = r;
                marker.color.g = g;
                marker.color.b = b;
                marker.color.a = a;
                return marker;
            };

        auto appendSegment = [&](visualization_msgs::Marker &marker,
                                 const Eigen::Vector3f &start,
                                 const Eigen::Vector3f &end) {
            marker.points.push_back(makePoint(start.x(), start.y(), start.z()));
            marker.points.push_back(makePoint(end.x(), end.y(), end.z()));
        };

        auto appendCube = [&](visualization_msgs::Marker &marker, const Eigen::Vector3f &position) {
            marker.points.push_back(makePoint(position.x(), position.y(), position.z()));
        };

        visualization_msgs::Marker nodeMarker = makeListMarker(
            "global_graph_nodes", 0, visualization_msgs::Marker::SPHERE_LIST,
            nodeScale, nodeScale, nodeScale,
            0.10f, 0.35f, 1.00f, 1.00f);

        visualization_msgs::Marker axisXMarker = makeListMarker(
            "global_graph_axes", 1, visualization_msgs::Marker::LINE_LIST,
            axisWidth, 0.0f, 0.0f,
            1.00f, 0.15f, 0.15f, 1.00f);
        visualization_msgs::Marker axisYMarker = makeListMarker(
            "global_graph_axes", 2, visualization_msgs::Marker::LINE_LIST,
            axisWidth, 0.0f, 0.0f,
            0.15f, 0.85f, 0.15f, 1.00f);
        visualization_msgs::Marker axisZMarker = makeListMarker(
            "global_graph_axes", 3, visualization_msgs::Marker::LINE_LIST,
            axisWidth, 0.0f, 0.0f,
            0.10f, 0.45f, 1.00f, 1.00f);

        visualization_msgs::Marker odomEdgeMarker = makeListMarker(
            "global_graph_edges", 10, visualization_msgs::Marker::LINE_LIST,
            edgeWidth, 0.0f, 0.0f,
            0.00f, 1.00f, 1.00f, 1.00f);
        visualization_msgs::Marker loopEdgeMarker = makeListMarker(
            "global_graph_edges", 11, visualization_msgs::Marker::LINE_LIST,
            edgeWidth, 0.0f, 0.0f,
            0.95f, 1.00f, 0.20f, 1.00f);
        visualization_msgs::Marker priorEdgeMarker = makeListMarker(
            "global_graph_edges", 12, visualization_msgs::Marker::LINE_LIST,
            edgeWidth, 0.0f, 0.0f,
            0.78f, 0.08f, 0.52f, 1.00f);

        visualization_msgs::Marker odomFactorMarker = makeListMarker(
            "global_graph_factors", 20, visualization_msgs::Marker::CUBE_LIST,
            factorScale, factorScale, factorScale,
            0.10f, 0.35f, 1.00f, 1.00f);
        visualization_msgs::Marker loopFactorMarker = makeListMarker(
            "global_graph_factors", 21, visualization_msgs::Marker::CUBE_LIST,
            factorScale, factorScale, factorScale,
            1.00f, 0.68f, 0.08f, 1.00f);
        visualization_msgs::Marker priorFactorMarker = makeListMarker(
            "global_graph_factors", 22, visualization_msgs::Marker::CUBE_LIST,
            factorScale, factorScale, factorScale,
            0.95f, 0.05f, 0.05f, 1.00f);
        visualization_msgs::Marker priorPoseMarker = makeListMarker(
            "global_graph_prior_pose", 23, visualization_msgs::Marker::SPHERE_LIST,
            nodeScale, nodeScale, nodeScale,
            0.65f, 0.15f, 0.90f, 1.00f);
        visualization_msgs::Marker priorAxisXMarker = makeListMarker(
            "global_graph_prior_axes", 24, visualization_msgs::Marker::LINE_LIST,
            axisWidth, 0.0f, 0.0f,
            1.00f, 0.15f, 0.15f, 1.00f);
        visualization_msgs::Marker priorAxisYMarker = makeListMarker(
            "global_graph_prior_axes", 25, visualization_msgs::Marker::LINE_LIST,
            axisWidth, 0.0f, 0.0f,
            0.15f, 0.85f, 0.15f, 1.00f);
        visualization_msgs::Marker priorAxisZMarker = makeListMarker(
            "global_graph_prior_axes", 26, visualization_msgs::Marker::LINE_LIST,
            axisWidth, 0.0f, 0.0f,
            0.10f, 0.45f, 1.00f, 1.00f);

        for (const auto &pose : graphKeyPoses->points)
        {
            const Eigen::Vector3f origin(pose.x, pose.y, pose.z);
            nodeMarker.points.push_back(makePoint(pose.x, pose.y, pose.z));

            const Eigen::Affine3f poseAffine = pclPointToAffine3f(pose);
            const Eigen::Matrix3f rotation = poseAffine.rotation();
            appendSegment(axisXMarker, origin, origin + rotation * Eigen::Vector3f(axisLength, 0.0f, 0.0f));
            appendSegment(axisYMarker, origin, origin + rotation * Eigen::Vector3f(0.0f, axisLength, 0.0f));
            appendSegment(axisZMarker, origin, origin + rotation * Eigen::Vector3f(0.0f, 0.0f, axisLength));
        }

        for (size_t i = 1; i < graphKeyPoses->points.size(); ++i)
        {
            const auto &posePrev = graphKeyPoses->points[i - 1];
            const auto &poseCur = graphKeyPoses->points[i];
            const Eigen::Vector3f start(posePrev.x, posePrev.y, posePrev.z);
            const Eigen::Vector3f end(poseCur.x, poseCur.y, poseCur.z);
            const Eigen::Vector3f midpoint = 0.5f * (start + end);

            appendSegment(odomEdgeMarker, start, end);
            appendCube(odomFactorMarker, midpoint);
        }

        for (const auto &loopPair : loopIndexContainerCopy)
        {
            const int keyCur = loopPair.first;
            const int keyPre = loopPair.second;
            if (keyCur < 0 || keyPre < 0 ||
                keyCur >= static_cast<int>(graphKeyPoses->points.size()) ||
                keyPre >= static_cast<int>(graphKeyPoses->points.size()))
            {
                continue;
            }

            const auto &poseCur = graphKeyPoses->points[keyCur];
            const auto &posePre = graphKeyPoses->points[keyPre];
            const Eigen::Vector3f start(poseCur.x, poseCur.y, poseCur.z);
            const Eigen::Vector3f end(posePre.x, posePre.y, posePre.z);
            const Eigen::Vector3f midpoint = 0.5f * (start + end);

            appendSegment(loopEdgeMarker, start, end);
            appendCube(loopFactorMarker, midpoint);
        }

        for (const auto &priorPair : priorVisContainerCopy)
        {
            const int keyCur = priorPair.first;
            const int keyLinked = priorPair.second.first;
            const std::array<float, 6> &relativePriorPoseArray = priorPair.second.second;
            if (keyCur < 0 || keyLinked < 0 ||
                keyCur >= static_cast<int>(graphKeyPoses->points.size()) ||
                keyLinked >= static_cast<int>(graphKeyPoses->points.size()))
            {
                continue;
            }

            const auto &poseCur = graphKeyPoses->points[keyCur];
            const auto &poseLinked = graphKeyPoses->points[keyLinked];
            const Eigen::Affine3f linkedPoseAffine = pclPointToAffine3f(poseLinked);
            const Eigen::Vector3f prior_offset{0.0f, 0.0f, static_cast<float>(priorVehicleComZ)};
            const Eigen::Affine3f relativePriorPose = pcl::getTransformation(
                relativePriorPoseArray[3]+prior_offset(0), relativePriorPoseArray[4]+prior_offset(1), relativePriorPoseArray[5]+prior_offset(2),
                relativePriorPoseArray[0], relativePriorPoseArray[1], relativePriorPoseArray[2]);
            const Eigen::Affine3f globalPriorPose = linkedPoseAffine * relativePriorPose;

            const Eigen::Vector3f start(poseCur.x, poseCur.y, poseCur.z);
            const Eigen::Vector3f middle = globalPriorPose.translation();
            const Eigen::Vector3f end(poseLinked.x, poseLinked.y, poseLinked.z);

            appendSegment(priorEdgeMarker, start, middle);
            appendSegment(priorEdgeMarker, middle, end);
            appendCube(priorFactorMarker, 0.5f * (start + middle));
            appendCube(priorFactorMarker, 0.5f * (middle + end));
            priorPoseMarker.points.push_back(makePoint(middle.x(), middle.y(), middle.z()));

            const Eigen::Matrix3f priorRotation = globalPriorPose.rotation();
            appendSegment(priorAxisXMarker, middle, middle + priorRotation * Eigen::Vector3f(axisLength, 0.0f, 0.0f));
            appendSegment(priorAxisYMarker, middle, middle + priorRotation * Eigen::Vector3f(0.0f, axisLength, 0.0f));
            appendSegment(priorAxisZMarker, middle, middle + priorRotation * Eigen::Vector3f(0.0f, 0.0f, axisLength));
        }

        markerArray.markers.push_back(nodeMarker);
        markerArray.markers.push_back(axisXMarker);
        markerArray.markers.push_back(axisYMarker);
        markerArray.markers.push_back(axisZMarker);
        markerArray.markers.push_back(odomEdgeMarker);
        markerArray.markers.push_back(loopEdgeMarker);
        markerArray.markers.push_back(priorEdgeMarker);
        markerArray.markers.push_back(odomFactorMarker);
        markerArray.markers.push_back(loopFactorMarker);
        markerArray.markers.push_back(priorFactorMarker);
        markerArray.markers.push_back(priorPoseMarker);
        markerArray.markers.push_back(priorAxisXMarker);
        markerArray.markers.push_back(priorAxisYMarker);
        markerArray.markers.push_back(priorAxisZMarker);

        pubGlobalGraph.publish(markerArray);
    }


    //! Matched prior pose transform
    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            if (loopCloseType == "rs"){
                performRSLoopClosure();}

            else if (loopCloseType == "sc")
            {
                performSCLoopClosure();
            }
            else {
                performSCLoopClosure();
                performRSLoopClosure();
            }
            // Visualization
            // visualizeLoopClosure();
        }
    }

    void priorThread()
    {
        if (priorFactorEnableFlag == false)
            return;

        ros::Rate rate(priorFactorFrequency);
        while (ros::ok())
        {
            rate.sleep();
            performPriorAssociation();
            visualizePrior();
        }
    }

    void performPriorAssociation()
    {
        priorFilter();

        printf("Prior pose size: %zu\n", priorPosePatchQueue.size());
        if (cloudKeyPoses6D->points.empty() || priorPosePatchQueue.empty())
            return;

        pcl::PointCloud<PointTypePose>::Ptr copy_KeyPoses6D;
        copy_KeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        std::deque<std::pair<std::array<double, 6>, pcl::PointCloud<GroundPatchType>>> copy_priorPosePatchQueue;
        std::deque<std::pair<double, int>> copy_priorTimeKeyQueue;
        mtx.lock();
        // Thread count
        *copy_KeyPoses6D = *cloudKeyPoses6D;
        copy_priorPosePatchQueue = priorPosePatchQueue;
        copy_priorTimeKeyQueue = priorTimeKeyQueue;
        mtx.unlock();

        
        // Protect shared resources
        auto pose_patch_iterator = copy_priorPosePatchQueue.cbegin();
        auto time_key_iterator = copy_priorTimeKeyQueue.cbegin();
        while (pose_patch_iterator != copy_priorPosePatchQueue.cend() &&
               time_key_iterator != copy_priorTimeKeyQueue.cend())
        {
            int linked_key_id = time_key_iterator->second;
            Eigen::Affine3f linked_key_pose = pclPointToAffine3f(copy_KeyPoses6D->points[linked_key_id]);
            
            std::array<double, 6> prior_pose = pose_patch_iterator->first;
            Eigen::Affine3f relative_prior_pose = pcl::getTransformation(
                prior_pose[3], prior_pose[4], prior_pose[5],
                prior_pose[0], prior_pose[1], prior_pose[2]);
            Eigen::Affine3f global_prior_pose = linked_key_pose * relative_prior_pose;

            int current_key_id = copy_KeyPoses6D->size() - 1;
            Eigen::Affine3f current_key_pose = pclPointToAffine3f(copy_KeyPoses6D->points.back());

            double prior_dist = (global_prior_pose.translation().head(2) - current_key_pose.translation().head(2)).norm();
            if (prior_dist < nearPriorRadius)
            {
                // Match succeeded
                pcl::PointCloud<GroundPatchType>::Ptr source_patch(new pcl::PointCloud<GroundPatchType>(pose_patch_iterator->second));
                pcl::PointCloud<GroundPatchType>::Ptr target_patch(new pcl::PointCloud<GroundPatchType>());
                mtx.lock();
                *target_patch = *GroundCloudFromlaser;
                mtx.unlock();

                if (!source_patch->empty() && !target_patch->empty())
                {
                    static pcl::IterativeClosestPoint<GroundPatchType, GroundPatchType> prior_icp;
                    prior_icp.setMaxCorrespondenceDistance(groundPatchSize);
                    prior_icp.setMaximumIterations(100);
                    prior_icp.setTransformationEpsilon(1e-6);
                    prior_icp.setEuclideanFitnessEpsilon(1e-6);
                    prior_icp.setRANSACIterations(0);
                    prior_icp.setInputSource(source_patch);
                    prior_icp.setInputTarget(target_patch);

                    pcl::PointCloud<GroundPatchType>::Ptr aligned_prior_patch(new pcl::PointCloud<GroundPatchType>());
                    prior_icp.align(*aligned_prior_patch);

                    const bool prior_icp_converged = prior_icp.hasConverged();
                    Eigen::Matrix4d prior_patch_transform = prior_icp.getFinalTransformation().cast<double>();
                    double prior_fitness_score = prior_icp.getFitnessScore();

                    printf("prior_patch_fitness_score = %f \n", prior_fitness_score);

                    if (!prior_icp_converged || prior_fitness_score > priorFitnessScore)
                    {
                        ++pose_patch_iterator;
                        ++time_key_iterator;
                        continue;
                    }
                    Eigen::Affine3f prior_patch_transform_f(prior_patch_transform.cast<float>());
                    
                    float patch_x, patch_y, patch_z, patch_roll, patch_pitch, patch_yaw;
                    pcl::getTranslationAndEulerAngles(prior_patch_transform_f,
                                                      patch_x, patch_y, patch_z,
                                                      patch_roll, patch_pitch, patch_yaw);
                    printf("prior_patch_transform xyzrpy: [%f, %f, %f, %f, %f, %f]\n",
                           patch_x, patch_y, patch_z,
                           patch_roll, patch_pitch, patch_yaw);
                    
                    Eigen::Affine3f tbbPrevToCur = current_key_pose * linked_key_pose.inverse();
                    Eigen::Affine3f tbbPrevToCur_prior = prior_patch_transform_f * relative_prior_pose;

                    float priorWeight = 0.2f;
                    Eigen::Vector3f odom_translation = tbbPrevToCur.translation();
                    Eigen::Vector3f blended_translation = odom_translation;
                    // blended_translation.z() = (1.0f - priorWeight) * odom_translation.z() + priorWeight * prior_translation.z();

                    Eigen::Quaternionf odom_quat(tbbPrevToCur.rotation());
                    Eigen::Quaternionf prior_quat(tbbPrevToCur_prior.rotation());
                    odom_quat.normalize();
                    prior_quat.normalize();

                    float odom_dx, odom_dy, odom_dz, odom_roll, odom_pitch, odom_yaw;
                    float prior_dx, prior_dy, prior_dz, prior_roll, prior_pitch, prior_yaw;
                    pcl::getTranslationAndEulerAngles(tbbPrevToCur,
                                                      odom_dx, odom_dy, odom_dz,
                                                      odom_roll, odom_pitch, odom_yaw);
                    pcl::getTranslationAndEulerAngles(tbbPrevToCur_prior,
                                                      prior_dx, prior_dy, prior_dz,
                                                      prior_roll, prior_pitch, prior_yaw);

                    float diff_z = std::fabs(odom_dz - prior_dz);
                    // Avoid overide ±pi bound
                    float diff_roll = std::fabs(std::atan2(std::sin(odom_roll - prior_roll), std::cos(odom_roll - prior_roll)));
                    float diff_pitch = std::fabs(std::atan2(std::sin(odom_pitch - prior_pitch), std::cos(odom_pitch - prior_pitch)));
                    if (diff_z > priorTransDiffTolerance ||
                        diff_roll > priorRotDiffTolerance ||
                        diff_pitch > priorRotDiffTolerance)
                    {
                        ++pose_patch_iterator;
                        ++time_key_iterator;
                        continue;
                    }

                    Eigen::Quaternionf blended_target_quat(
                        Eigen::AngleAxisf(odom_yaw, Eigen::Vector3f::UnitZ()) *
                        Eigen::AngleAxisf(prior_pitch, Eigen::Vector3f::UnitY()) *
                        Eigen::AngleAxisf(prior_roll, Eigen::Vector3f::UnitX()));
                    blended_target_quat.normalize();

                    Eigen::Quaternionf blended_quat = odom_quat.slerp(priorWeight, blended_target_quat);
                    blended_quat.normalize();

                    Eigen::Affine3f priorTrans = Eigen::Affine3f::Identity();
                    priorTrans.linear() = blended_quat.toRotationMatrix();
                    priorTrans.translation() = blended_translation;

                    float x, y, z, roll, pitch, yaw;
                    pcl::getTranslationAndEulerAngles(linked_key_pose, x, y, z, roll, pitch, yaw);
                    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));

                    Eigen::Affine3f corrected_current_pose = priorTrans * linked_key_pose;
                    pcl::getTranslationAndEulerAngles(corrected_current_pose, x, y, z, roll, pitch, yaw);
                    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));

                    gtsam::Vector Vector6(6);
                    float noiseScore = std::max(static_cast<float>(prior_fitness_score), 1e-6f);
                    noiseScore *= priorFactorWeight;
                    // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
                    Vector6 << noiseScore, noiseScore, 1e-6f, 1e-6f, 1e-6f, noiseScore;
                    // Vector6 << 1e-6f, 1e-6f, noiseScore, noiseScore, noiseScore, 1e-6f;
                    noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

                    mtx.lock();
                    priorIndexQueue.push_back(make_pair(linked_key_id, current_key_id));
                    // priorPoseQueue.push_back(priorTrans);
                    priorPoseQueue.push_back(poseFrom.between(poseTo));
                    priorNoiseQueue.push_back(constraintNoise);
	                    std::array<float, 6> relativePriorPose = {
	                        static_cast<float>(prior_pose[0]),
	                        static_cast<float>(prior_pose[1]),
	                        static_cast<float>(prior_pose[2]),
	                        static_cast<float>(prior_pose[3]),
	                        static_cast<float>(prior_pose[4]),
	                        static_cast<float>(prior_pose[5])
	                    };
	                    priorVisContainer[current_key_id] = std::make_pair(linked_key_id, relativePriorPose);
                    mtx.unlock();
                    printf("Patch Mathced!! \n");
                    break; // Matching only one prior at a time is allowed
                }
                
            }

            // priorPosePatchQueue.pop_front();
            // priorTimeKeyQueue.pop_front();
            ++pose_patch_iterator;
            ++time_key_iterator;
        }
        
        // priorFilter();

    }

    void priorFilter()
    {
        mtx.lock();

        if (cloudKeyPoses6D->empty())
        {
            mtx.unlock();
            return;
        }

        while (priorPosePatchQueue.size() > priorTimeKeyQueue.size())
            priorPosePatchQueue.pop_back();
        while (priorTimeKeyQueue.size() > priorPosePatchQueue.size())
            priorTimeKeyQueue.pop_back();

        auto pose_patch_iterator = priorPosePatchQueue.begin();
        auto time_key_iterator = priorTimeKeyQueue.begin();
        while (pose_patch_iterator != priorPosePatchQueue.end() &&
               time_key_iterator != priorTimeKeyQueue.end())
        {
            double time_diff = std::abs(time_key_iterator->first - timeLaserInfoCur);
            std::array<double, 6> &prior_pose = pose_patch_iterator->first;
            int linked_key_id = time_key_iterator->second;

            Eigen::Affine3f linked_key_pose = pclPointToAffine3f(cloudKeyPoses6D->points[linked_key_id]);
            Eigen::Affine3f relative_prior_pose = pcl::getTransformation(
                prior_pose[3], prior_pose[4], prior_pose[5],
                prior_pose[0], prior_pose[1], prior_pose[2]);
            Eigen::Affine3f global_prior_pose = linked_key_pose * relative_prior_pose;
            Eigen::Affine3f current_key_pose = trans2Affine3f(transformTobeMapped);

            double range_diff = (global_prior_pose.translation() - current_key_pose.translation()).norm();

            if (time_diff > priorTimeValidation || range_diff > priorRangeValidation)
            {
                pose_patch_iterator = priorPosePatchQueue.erase(pose_patch_iterator);
                time_key_iterator = priorTimeKeyQueue.erase(time_key_iterator);
                continue;
            }

            ++pose_patch_iterator;
            ++time_key_iterator;
        }

        mtx.unlock();
    }

    Eigen::Affine3f priorLidarPoseToBodyPose(const Eigen::Affine3f &lidar_pose) const
    {
        return lidar_pose * Eigen::Affine3f(priorVehicleModel.lidar_to_body());
    }

    jsk_recognition_msgs::BoundingBox buildPriorBoundingBox(const Eigen::Affine3f &global_prior_lidar_pose,
                                                            int linked_key_id,
                                                            const ros::Time &stamp) const
    {
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.header.frame_id = odometryFrame;
        bbox.header.stamp = stamp;
        bbox.label = linked_key_id;

        const Eigen::Affine3f global_body_pose = priorLidarPoseToBodyPose(global_prior_lidar_pose);
        const auto &wheel_points_body = priorVehicleModel.wheel_points_body();

        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        for (const auto &wheel_point : wheel_points_body)
        {
            min_x = std::min(min_x, static_cast<float>(wheel_point.x()));
            max_x = std::max(max_x, static_cast<float>(wheel_point.x()));
            min_y = std::min(min_y, static_cast<float>(wheel_point.y()));
            max_y = std::max(max_y, static_cast<float>(wheel_point.y()));
        }

        if (!std::isfinite(min_x) || !std::isfinite(max_x) || !std::isfinite(min_y) || !std::isfinite(max_y))
        {
            min_x = -static_cast<float>(0.5 * priorVehicleSizeX);
            max_x = static_cast<float>(0.5 * priorVehicleSizeX);
            min_y = -static_cast<float>(0.5 * priorVehicleSizeY);
            max_y = static_cast<float>(0.5 * priorVehicleSizeY);
        }

        const float height = std::max(static_cast<float>(priorVehicleComZ), 0.1f);
        const Eigen::Affine3f box_pose = global_body_pose * Eigen::Translation3f(
            0.5f * (min_x + max_x),
            0.5f * (min_y + max_y),
            -0.5f * height);

        bbox.pose.position.x = box_pose.translation().x();
        bbox.pose.position.y = box_pose.translation().y();
        bbox.pose.position.z = box_pose.translation().z();
        Eigen::Quaternionf q_box(box_pose.rotation());
        bbox.pose.orientation.x = q_box.x();
        bbox.pose.orientation.y = q_box.y();
        bbox.pose.orientation.z = q_box.z();
        bbox.pose.orientation.w = q_box.w();
        bbox.dimensions.x = std::max(max_x - min_x, 0.1f);
        bbox.dimensions.y = std::max(max_y - min_y, 0.1f);
        bbox.dimensions.z = height;

        return bbox;
    }

    void visualizePrior()
    {
        jsk_recognition_msgs::BoundingBoxArray box_array;
        box_array.header.frame_id = odometryFrame;
        box_array.header.stamp = timeLaserInfoStamp;

        pcl::PointCloud<PointTypePose>::Ptr copy_KeyPoses6D(new pcl::PointCloud<PointTypePose>());
        std::deque<std::pair<std::array<double, 6>, pcl::PointCloud<GroundPatchType>>> copy_priorPosePatchQueue;
        std::deque<std::pair<double, int>> copy_priorTimeKeyQueue;
        mtx.lock();
        *copy_KeyPoses6D = *cloudKeyPoses6D;
        copy_priorPosePatchQueue = priorPosePatchQueue;
        copy_priorTimeKeyQueue = priorTimeKeyQueue;
        mtx.unlock();

        pcl::PointCloud<GroundPatchType>::Ptr stacked_prior_patches(new pcl::PointCloud<GroundPatchType>());
        auto pose_patch_iterator = copy_priorPosePatchQueue.cbegin();
        auto time_key_iterator = copy_priorTimeKeyQueue.cbegin();
        while (pose_patch_iterator != copy_priorPosePatchQueue.cend() &&
               time_key_iterator != copy_priorTimeKeyQueue.cend())
        {
            const int linked_key_id = time_key_iterator->second;
            if (linked_key_id < 0 || linked_key_id >= static_cast<int>(copy_KeyPoses6D->size()))
            {
                ++pose_patch_iterator;
                ++time_key_iterator;
                continue;
            }

            const std::array<double, 6> &prior_pose = pose_patch_iterator->first;
            Eigen::Affine3f linked_key_pose = pclPointToAffine3f(copy_KeyPoses6D->points[linked_key_id]);
            Eigen::Affine3f relative_prior_pose = pcl::getTransformation(
                prior_pose[3], prior_pose[4], prior_pose[5],
                prior_pose[0], prior_pose[1], prior_pose[2]);
            Eigen::Affine3f global_prior_pose = linked_key_pose * relative_prior_pose;
            box_array.boxes.push_back(buildPriorBoundingBox(global_prior_pose, linked_key_id, timeLaserInfoStamp));

            pcl::PointCloud<GroundPatchType> global_patch;
            pcl::transformPointCloud(pose_patch_iterator->second, global_patch, global_prior_pose);
            *stacked_prior_patches += global_patch;

            ++pose_patch_iterator;
            ++time_key_iterator;
        }

        pubPriorPredictions.publish(box_array);
        publishCloud(pubPriorPatches, stacked_prior_patches, timeLaserInfoStamp, odometryFrame);
    }

    void publishePriorPoseHistory(){
        if (pubPriorPoseHistory.getNumSubscribers() == 0)
            return;

        pcl::PointCloud<PointTypePose>::Ptr copy_KeyPoses6D(new pcl::PointCloud<PointTypePose>());
        std::map<int, std::pair<int, std::array<float, 6>>> priorVisContainerCopy;
        ros::Time markerStamp;

        mtx.lock();
        if (cloudKeyPoses6D->points.empty() || priorVisContainer.empty())
        {
            mtx.unlock();
            return;
        }

        *copy_KeyPoses6D = *cloudKeyPoses6D;
        priorVisContainerCopy = priorVisContainer;
        markerStamp = timeLaserInfoStamp;
        mtx.unlock();

        jsk_recognition_msgs::BoundingBoxArray box_array;
        box_array.header.frame_id = odometryFrame;
        box_array.header.stamp = markerStamp;

        for (const auto &prior_pair : priorVisContainerCopy)
        {
            const int key_cur = prior_pair.first;
            const int key_linked = prior_pair.second.first;
            const std::array<float, 6> &relative_prior_pose_array = prior_pair.second.second;
            if (key_cur < 0 || key_linked < 0 ||
                key_cur >= static_cast<int>(copy_KeyPoses6D->points.size()) ||
                key_linked >= static_cast<int>(copy_KeyPoses6D->points.size()))
            {
                continue;
            }

            Eigen::Affine3f linked_key_pose = pclPointToAffine3f(copy_KeyPoses6D->points[key_linked]);
            Eigen::Affine3f relative_prior_pose = pcl::getTransformation(
                relative_prior_pose_array[3], relative_prior_pose_array[4], relative_prior_pose_array[5],
                relative_prior_pose_array[0], relative_prior_pose_array[1], relative_prior_pose_array[2]);
            Eigen::Affine3f global_prior_pose = linked_key_pose * relative_prior_pose;

            box_array.boxes.push_back(buildPriorBoundingBox(global_prior_pose, key_linked, markerStamp));
            box_array.boxes.back().label = key_cur;
        }

        pubPriorPoseHistory.publish(box_array);
    }

    //! Loop closure
    void performRSLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        // Find loop candidates and add ICP edge
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        // Loop closure
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false) // Loop closure
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)   // Loop closure
                return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);    // Loop key current
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum); // History keyframe search number
            // Reject tiny loop clouds
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        // Run ICP loop alignment
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0); // Set ransac iterations

        // Eigen alignment
        icp.setInputSource(cureKeyframeCloud); // Current loop source cloud
        icp.setInputTarget(prevKeyframeCloud); // Previous loop target cloud
        pcl::PointCloud<PointType>::Ptr temp_result(new pcl::PointCloud<PointType>());
        icp.align(*temp_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return; // Eigen alignment

        // publish corrected cloud
        // Eigen alignment
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            // Eigen alignment
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        // Matrix update
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        // Current keyframe feature set
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        // Reject small feature set
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        // Reject small feature set
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();// Distance check
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        // Loop closure
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    void performSCLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        std::pair<int, float> detectResult;
        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        detectResult = scManager.detectLoopClosureID();
        mtx.unlock();

        const int loopKeyCur = static_cast<int>(copy_cloudKeyPoses3D->size()) - 1;

        const int loopKeyPre = detectResult.first;
        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return;

        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            const int baseKey = 0;
            loopFindNearKeyframesWithRespectTo(cureKeyframeCloud, loopKeyCur, 0, baseKey);
            loopFindNearKeyframesWithRespectTo(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, baseKey);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(150.0);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr temp_result(new pcl::PointCloud<PointType>());
        // icp.align(*temp_result, initialGuess.matrix());
        icp.align(*temp_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;
        else
            std::cout << "ICP fitness test passed (" << icp.getFitnessScore() << " < " << historyKeyframeFitnessScore << "). Add this SC loop." << std::endl;

        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            // const Eigen::Affine3f correctionLidarFrame(icp.getFinalTransformation() * initialGuess.matrix());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame(icp.getFinalTransformation());
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        const float robustNoiseScore = icp.getFitnessScore(); 
        gtsam::Vector robustNoiseVector6(6);
        robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore,
            robustNoiseScore, robustNoiseScore, robustNoiseScore;
        gtsam::SharedNoiseModel robustConstraintNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(robustConstraintNoise);
        mtx.unlock();

        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }
    //! Loop closure
    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        // Search nearby historical frames
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) // Time update
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
        // Record farthest valid history frame
        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }
    //! Loop closure
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
        // Validate loop cloud size
        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)  // Loop closure
            return false;

        // latest key
        // Extract timestamp
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

        auto it = loopIndexContainer.find(loopKeyCur); // Loop closure
        if (it != loopIndexContainer.end()) // Loop closure
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }
    //! Picked-neighbor flag
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )   // Cloud size
                continue;
            // Too few history keyframes for loop closure
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty()) // Empty
            return;

        // downsample near keyframes
        // Find keyframe index by timestamp
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void loopFindNearKeyframesWithRespectTo(pcl::PointCloud<PointType>::Ptr& nearKeyframes,
                                            const int& key,
                                            const int& searchNum,
                                            const int wrt_key)
    {
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize)
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[wrt_key]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[wrt_key]);
            // Matching in the local scale
            // *nearKeyframes += *cornerCloudKeyFrames[keyNear];
            // *nearKeyframes += *surfCloudKeyFrames[keyNear];
        }

        if (nearKeyframes->empty())
            return;

        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }
    //! Loop closure
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

    void saveTUM(){
        ofstream tum_file;
        string pkg_path = ros::package::getPath("rolo");
        tum_file.open(pkg_path + "/tum_traj/rolo.tum");
        tum_file.clear();
        for (std::size_t i = 0; i < cloudKeyPoses6D->size(); ++i){
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(cloudKeyPoses6D->points[i].roll, 
                                                                                  cloudKeyPoses6D->points[i].pitch, 
                                                                                  cloudKeyPoses6D->points[i].yaw);
            tum_file << setprecision(19) << cloudKeyPoses6D->points[i].time << " "
                     << cloudKeyPoses6D->points[i].x << " "
                     << cloudKeyPoses6D->points[i].y << " "
                     << cloudKeyPoses6D->points[i].z << " "
                     << q.x << " "
                     << q.y << " "
                     << q.z << " "
                     << q.w << std::endl;
        }
        tum_file.close();
        printf("Saved .tum file!\n");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rolo");
    // Loop edge visualization
    backMapping BM;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    std::thread loopthread(&backMapping::loopClosureThread, &BM);
    std::thread priorthread(&backMapping::priorThread, &BM);
    std::thread visualizeMapThread(&backMapping::visualizeGlobalMapThread, &BM);

    ros::spin();

    loopthread.join();
    priorthread.join();
    visualizeMapThread.join();
    BM.saveTUM();
    return 0;
}
