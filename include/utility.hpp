#pragma once
#ifndef ROLO_UTILITY_HPP_
#define ROLO_UTILITY_HPP_

#include "rolo/utils/param_loader.hpp"
#include "rolo/CloudInfoStamp.h"

#include <array>
#include <cmath>
#include <cfloat>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

inline const bool pcl_console_verbosity_configured = []() {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    return true;
}();

// ROS / PCL publishing helpers
template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

template<typename T>
double GET_ROS_TIMESTAMP(T msg)
{
    return msg->header.stamp.toSec();
}

// Angle helpers
inline double radTodeg(double radians)
{
    return radians * 180.0 / M_PI;
}

inline double degTorad(double degrees)
{
    return degrees * M_PI / 180.0;
}

// Point cloud geometry helpers
inline float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

inline float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
}

// Odometry / pose conversion helpers
inline Eigen::Affine3f odom2affine(const nav_msgs::Odometry& odom)
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

inline bool affineToPose(const Eigen::Affine3f& affine, Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
{
    position = affine.translation().cast<double>();
    Eigen::Matrix3d rotation = affine.rotation().cast<double>();
    orientation = Eigen::Quaterniond(rotation);
    orientation.normalize();
    return true;
}

inline gtsam::Pose3 pclPointTogtsamPose3(const PointTypePose& thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

inline gtsam::Pose3 trans2gtsamPose(const float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

inline Eigen::Affine3f pclPointToAffine3f(const PointTypePose& thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z,
                                  thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

inline Eigen::Affine3f trans2Affine3f(const float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5],
                                  transformIn[0], transformIn[1], transformIn[2]);
}

inline PointTypePose trans2PointTypePose(const float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
}

// Eigen matrix helpers
template <class T, int R, int C>
void removeRow(Eigen::Matrix<T, R, C>& matrix, unsigned int rowToRemove)
{
    if (rowToRemove >= matrix.rows())
        return;

    unsigned int numRows = matrix.rows() - 1;
    unsigned int numCols = matrix.cols();
    if (rowToRemove < numRows)
    {
        matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) =
            matrix.bottomRows(numRows - rowToRemove);
    }
    matrix.conservativeResize(numRows, numCols);
}

template <class T, int R, int C>
void removeColumn(Eigen::Matrix<T, R, C>& matrix, unsigned int colToRemove)
{
    if (colToRemove >= matrix.cols())
        return;

    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols() - 1;
    if (colToRemove < numCols)
    {
        matrix.block(0, colToRemove, numRows, numCols - colToRemove) =
            matrix.rightCols(numCols - colToRemove);
    }
    matrix.conservativeResize(numRows, numCols);
}

#endif  // ROLO_UTILITY_HPP_
