#include "rolo/utility.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "cloud_msgs/slope.h"
#define PCL_NO_PRECOMPILE
#include <opencv/cv.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl-1.8/pcl/sample_consensus/sac.h>
#include <pcl-1.8/pcl/sample_consensus/sac_model.h>
#include <pcl-1.8/pcl/sample_consensus/ransac.h>
#include <pcl-1.8/pcl/registration/icp.h>
#include <pcl-1.8/pcl/surface/mls.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>

#include <boost/format.hpp>
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)
using namespace std;

typedef PointXYZIRT PointCloudIn;
class slopeEstimation: public ParamLoader
{
    public:

    float my_angle;
    ros::Publisher normal_pub;
    ros::Publisher test_pub;
    ros::Publisher angle_pub;
    ros::Publisher marker_pub;
    ros::Publisher myangle_pub;
    ros::Publisher myslope_pub;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber laserOdom_sub;
    ros::Publisher slopeMarker_pub;
    ros::Publisher areaMarker_pub;
    ros::Publisher textMarker_pub; 
    visualization_msgs::Marker normalMarker;
    nav_msgs::Odometry tempOdom;
    int numVelo;
    float anglebias;
    std::vector<float> angle_list;
    visualization_msgs::Marker slopeMarker;
    visualization_msgs::Marker areaMarker;
    visualization_msgs::Marker textMarker;

    

    slopeEstimation()
    {
        pointcloud_sub  =nh.subscribe("/velodyne_points",10,&slopeEstimation::laserCloudHandler, this);        
        normal_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_pc",10);
        test_pub = nh.advertise<sensor_msgs::PointCloud2>("testPC",1);
        angle_pub = nh.advertise<std_msgs::Float64>("eps_angle",1);
        marker_pub = nh.advertise<visualization_msgs::Marker>("my_marker",1);
        myangle_pub = nh.advertise<std_msgs::Float64>("my_angle",1);
        myslope_pub = nh.advertise<cloud_msgs::slope>("my_slope",10);
        slopeMarker_pub = nh.advertise<visualization_msgs::Marker>("slopeMarker", 2);
        areaMarker_pub = nh.advertise<visualization_msgs::Marker>("areaMarker", 1);
        textMarker_pub = nh.advertise<visualization_msgs::Marker>("textMarker", 1);

        normalMarker.type = visualization_msgs::Marker::ARROW;
        normalMarker.scale.x = 0.15;
        normalMarker.scale.y = 0.24;
        normalMarker.scale.z = 0.3;
        normalMarker.color.g = 1.0;
        normalMarker.color.a = 1.0;
        normalMarker.header.frame_id = robotFrame;
        normalMarker.ns = "normal";
        normalMarker.id = 14;
        my_angle = 0.0;
        numVelo = 0;
        slopeMarker.id = 100;
        slopeMarker.header.frame_id = "base_link";
        slopeMarker.type = slopeMarker.SPHERE;
        slopeMarker.color.a = 1.0;
        slopeMarker.color.r= 1.0;
        slopeMarker.scale.x = 0.8;
        slopeMarker.scale.y = 0.8;
        slopeMarker.scale.z = 0.8;
        areaMarker.header.frame_id = "base_link";
        areaMarker.type = areaMarker.CUBE;
        areaMarker.lifetime = ros::Duration(0.5);
        areaMarker.id = 102;
        areaMarker.scale.x = 12;
        areaMarker.scale.y = 2.5;
        areaMarker.scale.z = 0.1;
        areaMarker.color.g = 0.6;
        areaMarker.color.a = 0.5;

    }

    void detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
    {      
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // you can modify the parameter below
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.2);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        // double angle;
        // angle = seg.getEpsAngle();
        // angle_pub.publish(angle);
        if (inliers->indices.size() == 0)
        {
            cout<<"error! Could not found any inliers!"<<endl;
        }
        // extract ground
        pcl::ExtractIndices<pcl::PointXYZ> extractor;
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
        extractor.setNegative(false);
        extractor.filter(*cloud_filtered);
        // vise-versa, remove the ground not just extract the ground
        // just setNegative to be true
        cout << "filter done."<<endl;

    }
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<PointCloudIn>::Ptr cloudin(new pcl::PointCloud<PointCloudIn>());
        pcl::PointCloud<PointCloudIn>::Ptr cloud_filtered(new pcl::PointCloud<PointCloudIn>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlanein(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloudin);
        pcl::copyPointCloud(*cloudin,*cloudPlanein);
        normalEstimationPCA(cloudPlanein);
    }


    
    void normalEstimationPCA(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<pcl::PointNormal> mls_points;
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        pcl::CropBox<pcl::PointXYZ> CropB(true);

        CropB.setInputCloud(input_cloud);
        CropB.setMin(Eigen::Vector4f(detectionXMin,detectionYMin,detectionZMin,1.));
        CropB.setMax(Eigen::Vector4f(detectionXMax,detectionYMax,detectionZMax,1.)); 
        // false extract the points in the box
        // true extract the points out of the box    
        CropB.setNegative(false);
        // CropB.setIndices(filter_ind);
        CropB.filter(*input_cloud);

        Eigen::Vector4f xyz_centroid;

        // std::cout<<xyz_centroid<<std::endl;
        detectObjectsOnCloud(input_cloud,input_cloud);
        mls.setComputeNormals (true);

        // Set parameters
        mls.setInputCloud(input_cloud);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(2.0);

        // Reconstruct
        mls.process (mls_points);

        
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(mls_points, pcaCentroid);
        
        sensor_msgs::PointCloud2 outputPC;
        pcl::toROSMsg(mls_points, outputPC);
        outputPC.header.frame_id = robotFrame;

        normal_pub.publish(outputPC);

        pcl::compute3DCentroid(*input_cloud,xyz_centroid);

        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(mls_points, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
        //eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    
        Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
        transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pcaCentroid.head<3>());// 
    
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::transformPointCloud(*cloud, *transformedCloud, transform);
    
        // std::cout << eigenValuesPCA.col(0) << std::endl;
        // std::cout << eigenVectorsPCA.col(0)<< std::endl;
        Eigen::Vector3f Vx(1.0,0.0,0.0);
        Eigen::Vector3f eX = eigenVectorsPCA.col(0);
        if(eX[2]>0)
        {
            ;
        }
        else{
            eX[2] = -eX[2];
            eX[1] = -eX[1];
            eX[0] = -eX[0];
        }
        Vx[0] = eX[0];
        Vx[1] = eX[1];
        float angle = pcl::getAngle3D(eigenVectorsPCA.col(0),Vx,true);
        cloud_msgs::slope my_slope;
        std_msgs::Float64 output;
        // ROS_INFO("%lf",angle);
        output.data = angle-90.0-anglebias;
        my_slope.slope = output;
        my_slope.slope_pos.x = xyz_centroid(0);
        my_slope.slope_pos.y = xyz_centroid(1);
        my_slope.slope_pos.z = xyz_centroid(2);
        slopeMarker.pose.position = my_slope.slope_pos;
        areaMarker.pose.position.x = 9.0;
        areaMarker.pose.position.y = 0.0;
        areaMarker.pose.position.z = xyz_centroid(2);
        // areaMarker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-0.086,0);
        slopeMarker.pose.position.z +=1.2;

        slopeMarker_pub.publish(slopeMarker);
        myslope_pub.publish(my_slope);
        areaMarker_pub.publish(areaMarker);
        textMarker.text =(boost::format("Slope Angle: %2f")%(-output.data)).str();
        textMarker.scale.z = 0.6;

        textMarker.color.r = 1.0;
        textMarker.color.g = 1.0;
        textMarker.color.a = 1.0;
        textMarker.pose = areaMarker.pose;
        textMarker.pose.position.z +=2.3;
        textMarker.header.frame_id = "base_link";
        textMarker.lifetime = ros::Duration(0.5);
        textMarker.id = 15;
        textMarker_pub.publish(textMarker);
        if(fabs(angle-90.0-anglebias)<0.5*plainThre)
        {
            output.data = 0.0;
            angle_pub.publish(output);
            return;
        }
        else{
            angle_pub.publish(output);
            my_angle+=output.data;
            // numVelo += 1;
            // ROS_INFO("My angle is %f",my_angle);
            angle_list.push_back(output.data);
            // ROS_INFO("Number of velo is %d",numVelo);
            std_msgs::Float64 myangle;
            myangle.data = my_angle;
            myangle_pub.publish(myangle);

        }
        geometry_msgs::Point p1,p2;
        normalMarker.points.clear();
        p1.x = 0.0+xyz_centroid(0);
        p1.y = 0.0+xyz_centroid(1);
        p1.z = 0.0+xyz_centroid(2)+1.4;
        p2.x = eX[0]+xyz_centroid(0);
        p2.y = eX[1]+xyz_centroid(1);
        p2.z = eX[2]+xyz_centroid(2)+1.4;
        normalMarker.header.stamp = ros::Time::now();
        normalMarker.points.push_back(p1);
        normalMarker.points.push_back(p2);
        marker_pub.publish(normalMarker);
        textMarker.type = textMarker.TEXT_VIEW_FACING;
        
    }
   
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "slopeEstimation");
    slopeEstimation SE;
    
    while (ros::ok())
    {
        ros::spinOnce();
    }
    

    return 0;
}
