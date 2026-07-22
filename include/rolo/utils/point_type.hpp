#pragma once
#ifndef ROLO_UTILS_POINT_TYPE_HPP_
#define ROLO_UTILS_POINT_TYPE_HPP_

#define PCL_NO_PRECOMPILE
#ifdef USE_UNORDERED_MAP
#undef USE_UNORDERED_MAP
#endif
#define USE_UNORDERED_MAP 0

#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef HasRGB
#define HasRGB 1
#endif

enum class lidarType { VELODYNE, OUSTER };

struct EIGEN_ALIGN16 PointXYZIRGB
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    PCL_ADD_RGB;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    inline PointXYZIRGB()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        intensity = 0.0f;
        rgb = 0.0f;
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGB,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, rgb, rgb)
)

#if HasRGB
using PointType = PointXYZIRGB;
#else
using PointType = pcl::PointXYZI;
#endif

struct EIGEN_ALIGN16 GroundPatchType
{
    PCL_ADD_POINT4D;
    float intensity;
    float timestamp;
    std::uint8_t label;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    inline GroundPatchType()
    {
        x = y = z = timestamp = 0.0f;
        intensity = 0.0f;
        label = 0;
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(GroundPatchType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, timestamp, timestamp)
    (std::uint8_t, label, label)
)

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (float, time, time)
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

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRTRGB,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, rgb, rgb)
    (std::uint16_t, ring, ring)
    (float, time, time)
)

struct OusterPointXYZIRT
{
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
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, noise, noise)
    (std::uint32_t, range, range)
)

struct OusterPointXYZIRTRGB
{
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
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, rgb, rgb)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, noise, noise)
    (std::uint32_t, range, range)
)

#if HasRGB
using PointXYZIRT = VelodynePointXYZIRTRGB;
using OusterInputPointType = OusterPointXYZIRTRGB;
#else
using PointXYZIRT = VelodynePointXYZIRT;
using OusterInputPointType = OusterPointXYZIRT;
#endif

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
    (double, time, time)
)

using PointTypePose = PointXYZIRPYT;
using SCPointType = pcl::PointXYZI;

using PoseVector = Eigen::Matrix<double, 7, 1>;
using PoseVectorList = std::vector<PoseVector, Eigen::aligned_allocator<PoseVector>>;

#endif  // ROLO_UTILS_POINT_TYPE_HPP_
