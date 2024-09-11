#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <map>
#include <set> 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI  PointType;

// VLP-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 2.0;
// extern const float ang_bottom = 15.0+0.1;
// extern const int groundScanInd = 7;

// HDL-64E
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 360.0/float(Horizon_SCAN);
extern const float ang_res_y = 0.427;
extern const float ang_bottom = 24.9;
extern const int groundScanInd = 60;//lidar光束在地面下的数量

// CH32R
// const int N_SCAN = 32;
// const int Horizon_SCAN = 1800;
// const float ang_res_x = 360.0/float(Horizon_SCAN);
// const float ang_res_y = 2.61;
// const float ang_bottom = 2.487;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

// 传感器采样频率
extern const float scanPeriod = 0.1;

// 分割器参数
const double dCandEdgeRatio = 0.05;
const double dDisjointTheshold = 5;

// 检测器参数
const double plane_delta = 0.5;
const double plane_epsilon = 0.0012;
const double plane_gamma = 0.1;
const int  plane_theta = 30;

// struct smoothness_t{ 
//     float value;
//     size_t ind;
// };

// struct by_value{ 
//     bool operator()(smoothness_t const &left, smoothness_t const &right) { 
//         return left.value < right.value;
//     }
// };

namespace velodyne_pcl
{
    struct PointXYZIRT
    {
        PCL_ADD_POINT4D;                    // quad-word XYZ
        float         intensity;            ///< laser intensity reading
        std::uint16_t ring;                 ///< laser ring number
        float         time;                 ///< laser time reading
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
    }
    EIGEN_ALIGN16;
}  // namespace velodyne_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time))

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

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
