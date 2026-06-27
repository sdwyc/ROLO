<p align="center">
  <h1 align="center">TERRA: Terrain-aware Registration and Association for LiDAR Odometry with Ground Vehicle</h1>
<p align="center">
</p>

# Note
Only the core code of TEERA has been open-sourced. The complete code will be released after paper acceptance.

## Quick Test Guide

### Dependencies
Our system has been tested extensively on both Ubuntu 18.04 with ROS Melodic and Ubuntu 20.04 with ROS Noetic, although other versions may work. The following configuration with required dependencies has been verified to be compatible:
-   Ubuntu 18.04 or 20.04 (recommended)
-   ROS Melodic or Noetic (`nav_msgs`, `cv_bridge`, `rospy`, `roscpp`,  `std_msgs`,  `sensor_msgs`,  `geometry_msgs`,  `pcl_ros`, `tf`, `visualization_msgs`, `message_generation`)
-   C++ 14
-   CMake >= `3.0.2`
-   OpenCV >= `4.10.0`
-   GTSAM >= `4.2.0`
-   Boost >= `1.71`
-   GCC >= `8.4.0`
-   Point Cloud Library >= `1.10.0`
-   Eigen >= `3.3.7`

### Compiling

Create a catkin workspace, clone the `TERRA` repository into the `src` folder, and compile via the [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) package:

```bash
mkdir terra_ws && cd terra_ws && mkdir src && cd src
git clone <this-repository-url> TERRA
cd ..
catkin_make
```

### Test

To run, first launch TERRA via:

```bash
roslaunch terra terra_run.launch
```

In a separate terminal session, play back the bag:

```bash
rosbag play your-bag.bag --clock
```
