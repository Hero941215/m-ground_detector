# m-ground_detector

**Implementation of the m-ground_detector as described in the paper "IMLS-SLAM: scan-to-model matching based on 3D data", of Deschaud et al. (ICRA 2018).**

## 0. Features
In order to ensure the spatial uniformity of plane point extraction, this algorithm uses kd-tree to control the distance of sampling points. In addition, openmp is used to accelerate metric calculation and feature extraction steps. For the case of 20000 3D point inputs, the average time cost of algorithm is less than 30ms.

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
    git clone https://github.com/Hero941215/m-imls_sample
    cd m-imls_sample
    mkdir build
    cd build
    cmake ..
    make -j8
```

## 3. Run
### 3.1. **run demo**

    ./test_imls_sampling /your_pcd_path

## 4. Acknowledgments

Thanks for "IMLS-SLAM: scan-to-model matching based on 3D data"(Deschaud et al.).
