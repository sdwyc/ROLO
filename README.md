 ROLO-SLAM:  
 Rotation-Optimized LiDAR-Only SLAM in Uneven Terrain with Ground Vehicle
=
[[IEEE JFR]()][[ArXiv]()][[Video]()]

LiDAR-based SLAM is recognized as one effective method to offer localization guidance in rough environments. However, off-and-shelf LiDAR-based SLAM methods suffer from significant pose estimation drifts, particularly components relevant to the vertical direction, when passing to uneven terrains. This deficiency typically leads to a conspicuously distorted global map. In this article, a LiDAR-based SLAM method is presented to improve the accuracy of pose estimations for ground vehicles in rough terrains, which is termed Rotation-Optimized LiDAR-Only (ROLO) SLAM. The method exploits a forward location prediction to coarsely eliminate the location difference of consecutive scans, thereby enabling separate and accurate determination of the location and orientation at the front-end. Furthermore, we adopt a parallel-capable spatial voxelization for correspondence-matching. We develop a spherical alignment-guided rotation registration within each voxel to estimate the rotation of vehicle. By incorporating geometric alignment, we introduce the motion constraint into the optimization formulation to enhance the rapid and effective estimation of LiDAR's translation. Subsequently, we extract several keyframes to construct the submap and exploit an alignment from the current scan to the submap for precise pose estimation. Meanwhile, a global-scale factor graph is established to aid in the reduction of cumulative errors. In various scenes, diverse experiments have been conducted to evaluate our method. The results demonstrate that ROLO-SLAM excels in pose estimation of ground vehicles and outperforms existing state-of-the-art LiDAR SLAM frameworks.

## Instructions



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

