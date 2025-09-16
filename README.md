# :snowman: CVAT: A Curved Voxel Association Triangle Descriptor for Universal LiDAR Place Recognition

The official implementation of CVAT (A Curved Voxel Association Triangle Descriptor for Universal LiDAR Place Recognition), a universal LiDAR place recognition method. CVAT is a robust descriptor combining curved-surface feature and triangle feature. Through FOV Alignment strategy and fine triangle verification, we achieve universal LiDAR place recognition and accuracy 6-DOF pose estimation.

Welcome to our [website](https://yaepiii.github.io/CVAT/) for more details.

[![[Demo] CVAT: A Curved Voxel Association Triangle Descriptor for Universal LiDAR Place Recognition](./web/resources/figure3.png)](https://www.youtube.com/watch?v=BgZGm0iwlHU "(1) [Demo] CVAT: A Curved Voxel Association Triangle Descriptor for Universal LiDAR Place Recognition - YouTube")

## :gear: Installation
This project was tested on Ubuntu 20.04 and ROS Noetic.

### :bookmark_tabs: Dependence
Make sure the following dependencies are installed:
- CMake >= 3.20
- PCL >= 1.8.0
- Eigen >= 3.2.1

### :pencil: Build

Clone this repository and build:
```
mkdir cvat_ws/src -p
cd cvat_ws/src
git clone https://github.com/Yanpiii/CVAT.git
cd .. && catkin_make
source ~/cvat_ws/src/devel/setup.bash
```

### :heart: Single LiDAR Place Recognition

**KITTI/KITTI360**

Modify the `lidar_path` and `pose_path` in the `launch` file. Note that for KITTI and KITTI360, when loading point clouds in the `.bin` format and poses in the `tum` format (timestamp x y z qx qy qz qw), it is recommended to refer to the format of Semantic KITTI.
```
roslaunch cvat_descriptor cvat_kitti.launch
```

**HeLiPR**

Modify the `lidar_path`, `pose_path`, and `lidar_type` (Ouster, Velodyne, Avia, Aeva) in the `launch` file.
```
roslaunch cvat_descriptor cvat_helipr.launch
```

**MCD**

Modify the `bag_path` in the `launch` file. Note that for the MCD, we create a `.bag` file with submaps and localization groundtruth based on the official provided `.pcd` files. The demo data can be found here. **We just test Livox Mid 70 LiDAR**.
```
roslaunch cvat_descriptor cvat_mcd.launch
```

### :two_hearts: Multi LiDAR Place Recognition

We test cross-LiDAR place recognition on the HeLiPR dataset. Modify the `src_lidar_path`, `tgt_lidar_path`, `src_pose_path`, and `tgt_pose_path` in the `launch` file.

And, you should modify `src_type` and `tgt_type` (Ouster, Velodyne, Avia, Aeva) in the `launch` file for achieving "from source lidar to target lidar place recognition".
```
roslaunch cvat_descriptor cvat_multi_lidar.launch
```

# Website License
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
