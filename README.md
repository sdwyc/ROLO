 ROLO-SLAM:  
 Rotation-Optimized LiDAR-Only SLAM in Uneven Terrain with Ground Vehicle
=
ROLO-SLAM is a lightweight and robust LiDAR-based SLAM solution designed to improve the accuracy of pose estimation for ground vehicles in rough terrains. It incorporates several algorithmic innovations that reduce pose estimation drifts, particularly in the vertical direction, which are commonly observed when navigating uneven terrains. The method includes forward location prediction to coarsely eliminate the location differences between consecutive scans, enabling separate and accurate localization and orientation determination. Additionally, ROLO-SLAM features a parallel-capable spatial voxelization for correspondence matching, along with a spherical alignment-guided rotation registration to estimate vehicle rotation. By incorporating motion constraints into the optimization process, the algorithm enhances the rapid and effective estimation of LiDAR translation. Extensive experiments conducted across diverse environments demonstrate that ROLO-SLAM consistently achieves accurate pose estimation and outperforms existing state-of-the-art LiDAR SLAM solutions, making it a reliable choice for ground vehicle localization in perceptually-challenging environments.

<div align="center">
    <img src="https://github.com/sdwyc/ROLO/blob/main/doc/gif/4scenes-ezgif.com-video-to-gif-converter.gif" alt="GIF Description" width="500" />
</div>

## Instructions

ROLO requires an input point cloud of type `sensor_msgs::PointCloud2` . 
ROLO-SLAM mitigates vertical pose drift by dividing the front-end into three modules: forward location prediction for coarse translation estimation, voxelization matching for precise rotation estimation, and continuous-time translation estimation for improved accuracy. The back-end integrates scan-to-submap alignment and global factor graph optimization to enhance overall localization performance in challenging terrains.

<div align="center">
    <img src="https://github.com/sdwyc/ROLO/blob/main/doc/img/platform_00.png" alt="Example Image" width="600" />
</div>

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


## Test Data
For your convenience, we provide example test data [here](https://ucla.box.com/shared/static/ziojd3auzp0zzcgwb1ucau9anh69xwv9.bag) (9 minutes, ~4.2GB). To run, first launch ROLO via:
```
roslaunch rolo rolo_run.launch
```
In a separate terminal session, play back the downloaded bag:

```
rosbag play your-bag.bag -r 1
```
<div align="center">
		 <img src="https://github.com/sdwyc/ROLO/blob/main/doc/img/off3_mapping_00.png" alt="Example Image" width="750" />
</div>

## Citation
Our work will be published in JOURNAL OF FILLD ROBOTICS, if you found this work useful, please cite our manuscript:
```bibtex
@article{wang2024rolo,
  author={Wang, Yinchuan and Ren, Bin and Zhang, Xiang and Wang, Pengyu and Wang, Chaoqun and Song, Rui and Li, Yibin and Meng, Max Q.-H.},
  journal={JOURNAL OF FIELD ROBOTICS},
  title={ROLO-SLAM: Rotation-Optimized LiDAR-Only SLAM in Uneven Terrain with Ground Vehicle},
  year={2024},
}
```

## Acknowledgements
We thank the authors of the  [FastGICP](https://github.com/SMRT-AIST/fast_gicp)  and  [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)  open-source packages:

-   Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno, “Voxelized GICP for Fast and Accurate 3D Point Cloud Registration,” in  _IEEE International Conference on Robotics and Automation (ICRA)_, IEEE, 2021, pp. 11 054–11 059.
-   T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti and D. Rus, "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping," _2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)_, Las Vegas, NV, USA, 2020, pp. 5135-5142, doi: 10.1109/IROS45743.2020.9341176.

## License
This work is licensed under the terms of the MIT license.
<div align="center">
    <img src="https://github.com/sdwyc/ROLO/blob/main/doc/gif/comparison_3-ezgif.com-video-to-gif-converter.gif" alt="GIF Description" width="500" />
</div>
