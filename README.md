 ROLO-SLAM:  
 Rotation-Optimized LiDAR-Only SLAM in Uneven Terrain with Ground Vehicle
=
[[IEEE JFR]()][[ArXiv]()][[Video]()]

LiDAR-based SLAM is recognized as one effective method to offer localization guidance in rough environments. However, off-and-shelf LiDAR-based SLAM methods suffer from significant pose estimation drifts, particularly components relevant to the vertical direction, when passing to uneven terrains. This deficiency typically leads to a conspicuously distorted global map. In this article, a LiDAR-based SLAM method is presented to improve the accuracy of pose estimations for ground vehicles in rough terrains, which is termed Rotation-Optimized LiDAR-Only (ROLO) SLAM. The method exploits a forward location prediction to coarsely eliminate the location difference of consecutive scans, thereby enabling separate and accurate determination of the location and orientation at the front-end. Furthermore, we adopt a parallel-capable spatial voxelization for correspondence-matching. We develop a spherical alignment-guided rotation registration within each voxel to estimate the rotation of vehicle. By incorporating geometric alignment, we introduce the motion constraint into the optimization formulation to enhance the rapid and effective estimation of LiDAR's translation. Subsequently, we extract several keyframes to construct the submap and exploit an alignment from the current scan to the submap for precise pose estimation. Meanwhile, a global-scale factor graph is established to aid in the reduction of cumulative errors. In various scenes, diverse experiments have been conducted to evaluate our method. The results demonstrate that ROLO-SLAM excels in pose estimation of ground vehicles and outperforms existing state-of-the-art LiDAR SLAM frameworks.

## Instructions

The file explorer is accessible using the button in left corner of the navigation bar. You can create a new file by clicking the **New file** button in the file explorer. You can also create folders by clicking the **New folder** button.

## Dependencies

Our system has been tested extensively on both Ubuntu 18.04 with ROS Melodic and Ubuntu 20.04 with ROS Noetic, although other versions may work. The following configuration with required dependencies has been verified to be compatible:
-   Ubuntu 18.04 or 20.04
-   ROS Melodic or Noetic (`roscpp`,  `std_msgs`,  `sensor_msgs`,  `geometry_msgs`,  `pcl_ros`)
-   C++ 14
-   CMake >=  `3.16.3`
-   OpenMP >=  `4.5`
-   Point Cloud Library >=  `1.10.0`
-   Eigen >=  `3.3.7`
## Compiling

You can rename the current file by clicking the file name in the navigation bar or by clicking the **Rename** button in the file explorer.

## Execution

You can delete the current file by clicking the **Remove** button in the file explorer. The file will be moved into the **Trash** folder and automatically deleted after 7 days of inactivity.

## Services

You can export the current file by clicking **Export to disk** in the menu. You can choose to export the file as plain Markdown, as HTML using a Handlebars template or as a PDF.


## Test Data

Synchronization is one of the biggest features of StackEdit. It enables you to synchronize any file in your workspace with other files stored in your **Google Drive**, your **Dropbox** and your **GitHub** accounts. This allows you to keep writing on other devices, collaborate with people you share the file with, integrate easily into your workflow... The synchronization mechanism takes place every minute in the background, downloading, merging, and uploading file modifications.

There are two types of synchronization and they can complement each other:

- The workspace synchronization will sync all your files, folders and settings automatically. This will allow you to fetch your workspace on any other device.
	> To start syncing your workspace, just sign in with Google in the menu.

- The file synchronization will keep one file of the workspace synced with one or multiple files in **Google Drive**, **Dropbox** or **GitHub**.
	> Before starting to sync files, you must link an account in the **Synchronize** sub-menu.

## Citation

You can open a file from **Google Drive**, **Dropbox** or **GitHub** by opening the **Synchronize** sub-menu and clicking **Open from**. Once opened in the workspace, any modification in the file will be automatically synced.

## Acknowledgements

You can save any file of the workspace to **Google Drive**, **Dropbox** or **GitHub** by opening the **Synchronize** sub-menu and clicking **Save on**. Even if a file in the workspace is already synced, you can save it to another location. StackEdit can sync one file with multiple locations and accounts.

## License

Once your file is linked to a synchronized location, StackEdit will periodically synchronize it by downloading/uploading any modification. A merge will be performed if necessary and conflicts will be resolved.

If you just have modified your file and you want to force syncing, click the **Synchronize now** button in the navigation bar.

> **Note:** The **Synchronize now** button is disabled if you have no file to synchronize.

