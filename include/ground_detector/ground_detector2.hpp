#ifndef GROUNDDETECTOR2_H
#define GROUNDDETECTOR2_H



#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/common/common.h>

#include <vector>

#include <mutex>

#include <thread>
#include "tic_toc.h"

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

typedef pcl::PointXYZI PointType;

class GroundDetector2
{
    
public:
    GroundDetector2(int ns, double sh, int ni, int nl, double ts, double td);
    
    void DetectGround(pcl::PointCloud<PointType>::Ptr RawPointCloud);  // 输入原始点云数据

    void DetectGroundBySegment(pcl::PointCloud<PointType>::Ptr rpc, pcl::PointCloud<PointType>::Ptr sazm,
                            Eigen::Vector4d &plane, pcl::PointCloud<PointType>::Ptr gm);

    pcl::PointCloud<PointType>::Ptr getMergedRawPointCloud();

    std::vector<pcl::PointCloud<PointType>::Ptr> getDetect3dGround();

protected:

    // 填充 mRPCs, mSortAxisZMeas
    void pc_segment_by_long_axis();

    // 下面的函数改成向量相关
    void mirror_reflection_(pcl::PointCloud<PointType>::Ptr pc);
    
    // estimate plane 
    void estimate_plane_(pcl::PointCloud<PointType>::Ptr gm, Eigen::Vector4d &plane);
    void extract_initial_seeds_(pcl::PointCloud<PointType>::Ptr sazm, pcl::PointCloud<PointType>::Ptr gm);
    
    // loop for org points segment
    void plane_fitter_(pcl::PointCloud<PointType>::Ptr rpc, pcl::PointCloud<PointType>::Ptr gm, Eigen::Vector4d &plane);
    
protected:
    
    pcl::PointCloud<PointType>::Ptr mRawPointCloud; // 接口输入的原始点云 
    std::vector<pcl::PointCloud<PointType>::Ptr> mRPCs;           // 分组后的点云， intensity仍然用于存储是否是地面点
    std::vector<pcl::PointCloud<PointType>::Ptr> mSortAxisZMeas;  // 测量值对Z轴进行排序
    std::vector<pcl::PointCloud<PointType>::Ptr> mGroundMeas;     // 后端用于迭代当前关键帧测量值中的地面点
    std::vector<Eigen::Vector4d> mPlanes;  // 储存估计参数

    pcl::PointCloud<PointType>::Ptr mMergedRawPointCloud; // 将 mRPCs 合并
    pcl::PointCloud<PointType>::Ptr mMergedGroundPointCloud; // 将 mGroundMeas 合并

    int num_segments_;
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