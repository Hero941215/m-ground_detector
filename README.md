# m-ground_detector

**Implementation of the m-ground_detector as described in the paper "Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications", of Zermas et al. (ICRA 2017).**

## 0. Features
Adaptive detection of the long axis of a single frame point cloud. Improve the ability of ground detection algorithms to cope with slope changes through multi-threaded block processing. For 64 line laser point cloud input, the algorithm can achieve an average time of less than 15ms.

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 18.04**

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.3, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

## 2. Build

Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/Hero941215/m-ground_detector
    cd m-ground_detector
    mkdir build
    cd build
    cmake ..
    make -j8
```

## 3. Run
### 3.1. **run demo**

    ./test_ground_detector2 /your_pcd_path

## 4. Acknowledgments

Thanks for [Run_based_segmentation](https://github.com/VincentCheungM/Run_based_segmentation).
