```bash
roslaunch rolo rolo_run.launch

```bash
roslaunch point_seg ground_mapping.launch
```
```bash
roslaunch terrain_pose_test terrain_pose_visualize.launch 

cd src/ROLO/src
python3 1111.py (预测未来位置的python文件)

rosbag play offroad.bag  --clock 

学长，我已经再point_seg过滤了距离近的点，在min_point_distance这个参数。
然后在lidar_accumulator.cpp最后的几行把latest改成body，让地面稠密很多。
然后改了pose_solver去拟合车轮附近的曲面，现在仍然有抽搐，估算姿态会飘，歪，或者沉到地下等情况，不知道该怎么解决，需要您帮忙看一下。
所有的launch，cpp复件都是之前的原版，现在用的是我修改后的版本
