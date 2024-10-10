 ROLO-SLAM:  
 Rotation-Optimized LiDAR-Only SLAM in Uneven Terrain with Ground Vehicle
=
[[IEEE JFR]()][[ArXiv]()][[Video]()]

ROLO-SLAM is a lightweight and robust LiDAR-based SLAM solution designed to improve the accuracy of pose estimation for ground vehicles in rough terrains. It incorporates several algorithmic innovations that reduce pose estimation drifts, particularly in the vertical direction, which are commonly observed when navigating uneven terrains. The method includes forward location prediction to coarsely eliminate the location differences between consecutive scans, enabling separate and accurate localization and orientation determination. Additionally, ROLO-SLAM features a parallel-capable spatial voxelization for correspondence matching, along with a spherical alignment-guided rotation registration to estimate vehicle rotation. By incorporating motion constraints into the optimization process, the algorithm enhances the rapid and effective estimation of LiDAR translation. Extensive experiments conducted across diverse environments demonstrate that ROLO-SLAM consistently achieves accurate pose estimation and outperforms existing state-of-the-art LiDAR SLAM solutions, making it a reliable choice for ground vehicle localization in perceptually-challenging environments.

## Instructions

ROLO requires an input point cloud of type `sensor_msgs::PointCloud2` . 
ROLO-SLAM mitigates vertical pose drift by dividing the front-end into three modules: forward location prediction for coarse translation estimation, voxelization matching for precise rotation estimation, and continuous-time translation estimation for improved accuracy. The back-end integrates scan-to-submap alignment and global factor graph optimization to enhance overall localization performance in challenging terrains.

## Dependencies

Our system has been tested extensively on both Ubuntu 18.04 with ROS Melodic and Ubuntu 20.04 with ROS Noetic, although other versions may work. The following configuration with required dependencies has been verified to be compatible:
-   Ubuntu 18.04 or 20.04
-   ROS Melodic or Noetic (`nav_msgs`, `cv_bridge`, `rospy`, `roscpp`,  `std_msgs`,  `sensor_msgs`,  `geometry_msgs`,  `pcl_ros`, `tf`, `visualization_msgs`, `message_generation`)
-   C++ 14
-   CMake >=  `3.0.2`
-   OpenCV
-   GTSAM
-   Boost
-   OpenMP 
-   Point Cloud Library 
-   Eigen 
## Compiling

Create a catkin workspace, clone the `ROLO` repository into the `src` folder, and compile via the [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) package :
```bash
mkdir rolo_ws && cd rolo_ws && mkdir src && cd src
git clone https://github.com/vectr-ucla/direct_lidar_odometry.git
cd ..
catkin_make
```

## Run the package

1.  Run the launch file:

```
roslaunch rolo rolo_run.launch
```

2.  Play existing bag files:

```
rosbag play your-bag.bag -r 1
```

## Test Data


## Citation


## Acknowledgements


## License
