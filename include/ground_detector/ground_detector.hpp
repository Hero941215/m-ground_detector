#ifndef GROUNDDETECTOR_H
#define GROUNDDETECTOR_H



#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vector>

#include <mutex>

#include <thread>

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

typedef pcl::PointXYZI PointType;

class GroundDetector
{
    
public:
    GroundDetector(double sh, int ni, int nl, double ts, double td);
    
    void DetectGround(pcl::PointCloud<PointType>::Ptr RawPointCloud);  // 输入原始点云数据

    pcl::PointCloud<PointType>::Ptr getRawPointCloud();

protected:
    void mirror_reflection_();
    
    // estimate plane 
    void estimate_plane_(void);
    void extract_initial_seeds_();
    
    // loop for org points segment
    void plane_fitter_();
    
protected:
    
    pcl::PointCloud<PointType>::Ptr mRawPointCloud; // 接口输入的原始点云 
    pcl::PointCloud<PointType>::Ptr mSortAxisZMea;  // 测量值对Z轴进行排序
    pcl::PointCloud<PointType>::Ptr mGroundMea;     // 后端用于迭代当前关键帧测量值中的地面点
    
    double sensor_height_;
    int num_iter_;
    int num_lpr_;
    double th_seeds_;
    double th_dist_;
    
    float d_;
    MatrixXf normal_;
    float th_dist_d_;
    
};


#endif