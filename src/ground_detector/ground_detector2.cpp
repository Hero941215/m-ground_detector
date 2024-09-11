
#include "ground_detector/ground_detector2.hpp"

/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool point_cmp(PointType a, PointType b){
    return a.z<b.z;
}

GroundDetector2::GroundDetector2(int ns, double sh, int ni, int nl, double ts, double td): num_segments_(ns),
sensor_height_(sh), num_iter_(ni), num_lpr_(nl), th_seeds_(ts), th_dist_(td)
{
    mRawPointCloud.reset(new pcl::PointCloud<PointType>());

    // 基于要求的 ns 初始化参数
    for(int i=0; i<num_segments_; i++)
    {
        pcl::PointCloud<PointType>::Ptr rpc_seg(new pcl::PointCloud<PointType>());
        mRPCs.push_back(rpc_seg);

        pcl::PointCloud<PointType>::Ptr sazm_seg(new pcl::PointCloud<PointType>());
        mSortAxisZMeas.push_back(sazm_seg);

        pcl::PointCloud<PointType>::Ptr gm_seg(new pcl::PointCloud<PointType>());
        mGroundMeas.push_back(gm_seg);
    }

    mPlanes.resize(num_segments_);
    mMergedRawPointCloud.reset(new pcl::PointCloud<PointType>());
    mMergedGroundPointCloud.reset(new pcl::PointCloud<PointType>());
}

void GroundDetector2::DetectGround(pcl::PointCloud<PointType>::Ptr RawPointCloud)
{
    mRawPointCloud->clear();
    mMergedRawPointCloud->clear();
    mMergedGroundPointCloud->clear();
    pcl::copyPointCloud(*RawPointCloud, *mRawPointCloud);

    // 按照 激光系的 x 轴分割点云
    pc_segment_by_long_axis();

    // 检测有效分段
    std::vector<std::thread*> mthreads(num_segments_);

    // 对分段后的点云，分别进行地面检测
    for(int i=0; i<num_segments_; i++)
    {
        mthreads[i] = new std::thread(&GroundDetector2::DetectGroundBySegment, this, mRPCs[i], mSortAxisZMeas[i], std::ref(mPlanes[i]), mGroundMeas[i]);
    }

    // 合并检测结果
    for(int i=0; i<num_segments_; i++)
    {
        mthreads[i]->join();
        // *mMergedRawPointCloud += *mRPCs[i];
        // *mMergedGroundPointCloud += *mSortAxisZMeas[i];
        delete mthreads[i];
    }

    // std::cout << "mMergedRawPointCloud->size(): " << mMergedRawPointCloud->size() << std::endl;

}

// 填充 mRPCs, mSortAxisZMeas
void GroundDetector2::pc_segment_by_long_axis()
{

    TicToc t_minmax;
    float seg_ratio_from_zero = 0.1; // 以0为中心，向前向后 5%

    // 找到点云边界值
    PointType p_min;//用于存放三个轴的最小值
    PointType p_max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*mRawPointCloud, p_min, p_max);

    // printf("whole minmax time %f ms \n \n", t_minmax.toc());

    std::string filterField;
    float x_range = p_max.x - p_min.x;
    float y_range = p_max.y - p_min.y;
    float min_th, min_mid_th, mid_max_th, max_th;

    // std::cout << "p_min.z: " << p_min.z << " p_max.z: " << p_max.z << std::endl;

    // 同时确定直通滤波器边界
    if(x_range>y_range)
    {
       filterField = "x";
       min_th = p_min.x;
       max_th = p_max.x;
    }
    else 
    {
       filterField = "y";
       min_th = p_min.y;
       max_th = p_max.y;
    }

    // 判断上下界是否异号, 异号则执行预设比例分割
    if(min_th*max_th<0)
    {
        min_mid_th = seg_ratio_from_zero * min_th;
        mid_max_th = seg_ratio_from_zero * max_th;
    }
    else
    {
        float range;
        if(filterField == "x")
            range = x_range;
        else
            range = y_range;

        // 进行等比例分割
        float mid = (max_th+min_th) / 2;
        min_mid_th = mid - range / 6;
        mid_max_th = mid + range / 6; 
    }

    std::cout << "max_th: " << max_th << " mid_max_th: " << mid_max_th 
              << " min_mid_th: " << min_mid_th << " min_th: " << min_th << std::endl;
    
    pcl::PassThrough<PointType> passthrough;  //设置滤波器对象
	passthrough.setInputCloud(mRawPointCloud);//输入点云

	passthrough.setFilterFieldName(filterField);//对z轴进行操作，也可以对"x"和"y"轴进行操作    
	
    // 中间组 
    pcl::PointCloud<PointType>::Ptr mid_cloud_after_PassThrough(new pcl::PointCloud<PointType>);
    passthrough.setFilterLimits(min_mid_th, mid_max_th);//设置直通滤波器操作范围
	//passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
	passthrough.filter(*mid_cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough
    pcl::copyPointCloud(*mid_cloud_after_PassThrough, *mRPCs[0]);
    pcl::copyPointCloud(*mid_cloud_after_PassThrough, *mSortAxisZMeas[0]);

    std::cout << "mid_cloud_after_PassThrough->size(): " << mid_cloud_after_PassThrough->size() << std::endl;
	
    // 向前组 
    pcl::PointCloud<PointType>::Ptr forward_cloud_after_PassThrough(new pcl::PointCloud<PointType>);
    passthrough.setFilterLimits(mid_max_th, max_th);//设置直通滤波器操作范围
    passthrough.filter(*forward_cloud_after_PassThrough);
    pcl::copyPointCloud(*forward_cloud_after_PassThrough, *mRPCs[1]);
    pcl::copyPointCloud(*forward_cloud_after_PassThrough, *mSortAxisZMeas[1]);

    std::cout << "forward_cloud_after_PassThrough->size(): " << forward_cloud_after_PassThrough->size() << std::endl;

    // 向后组 
    pcl::PointCloud<PointType>::Ptr backward_cloud_after_PassThrough(new pcl::PointCloud<PointType>);
    passthrough.setFilterLimits(min_th, min_mid_th);//设置直通滤波器操作范围
    passthrough.filter(*backward_cloud_after_PassThrough);
    pcl::copyPointCloud(*backward_cloud_after_PassThrough, *mRPCs[2]);
    pcl::copyPointCloud(*backward_cloud_after_PassThrough, *mSortAxisZMeas[2]);

    std::cout << "backward_cloud_after_PassThrough->size(): " << backward_cloud_after_PassThrough->size() << std::endl;

}

void GroundDetector2::DetectGroundBySegment(pcl::PointCloud<PointType>::Ptr rpc, pcl::PointCloud<PointType>::Ptr sazm,
                            Eigen::Vector4d &plane, pcl::PointCloud<PointType>::Ptr gm)
{
    
    TicToc t_whole;
    // 排序
    std::sort(sazm->points.begin(),sazm->points.end(),point_cmp);

    mirror_reflection_(sazm);

    extract_initial_seeds_(sazm, gm);

    plane_fitter_(rpc, gm, plane);

}

// 下面的函数改成向量相关
void GroundDetector2::mirror_reflection_(pcl::PointCloud<PointType>::Ptr pc)
{

    pcl::PointCloud<PointType>::iterator it = pc->points.begin();
    for(size_t i=0;i<pc->points.size();i++)
    {
        if(pc->points[i].z < -1.5*sensor_height_)
        {
            it++;
        }
        else
            break;
    }
    pc->points.erase(pc->points.begin(),it);
}

// estimate plane 
void GroundDetector2::estimate_plane_(pcl::PointCloud<PointType>::Ptr gm, Eigen::Vector4d &plane)
{
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*gm, cov, pc_mean);
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    Eigen::Vector3f normal = (svd.matrixU().col(2));
    plane(0) = normal.x();
    plane(1) = normal.y();
    plane(2) = normal.z();
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    double d = -(normal.transpose()*seeds_mean)(0,0); // 平面参数
    plane(3) = d; 

}

void GroundDetector2::extract_initial_seeds_(pcl::PointCloud<PointType>::Ptr sazm, pcl::PointCloud<PointType>::Ptr gm)
{
    gm->clear();

    double sum = 0;
    int cnt = 0;
    for(size_t i=0;i<sazm->points.size() && cnt<num_lpr_;i++)
    {
        sum += sazm->points[i].z;
        cnt++;
    }
    
    double lpr_height = cnt!=0?sum/cnt:0;
    for(size_t i=0;i<sazm->points.size();i++)
    {
        if(sazm->points[i].z < lpr_height + th_seeds_)
            gm->points.push_back(sazm->points[i]);
    }
}

// loop for org points segment
void GroundDetector2::plane_fitter_(pcl::PointCloud<PointType>::Ptr rpc, pcl::PointCloud<PointType>::Ptr gm, Eigen::Vector4d &plane)
{
    for(int i=0;i<num_iter_;i++)
    {
        estimate_plane_(gm ,plane);
        gm->clear();

        //pointcloud to matrix
        MatrixXf points(rpc->points.size(),3);
        int j =0;
        for(auto p:rpc->points)
            points.row(j++)<<p.x,p.y,p.z;
        
        // ground plane model
        VectorXf ones(rpc->points.size()); ones.setOnes();
        Eigen::Vector3f normal(plane(0), plane(1), plane(2));
        VectorXf result = points*normal + plane(3)*ones;
        
        // threshold filter
        for(int r=0;r<result.rows();r++)
        {
            if(std::fabs(result[r])<th_dist_)
            {
                // rpc->points[r].intensity = 1.0;
                gm->points.push_back(rpc->points[r]);
            }
            else
            {
                // rpc->points[r].intensity = 0.0;
            }
        }
    }
}

pcl::PointCloud<PointType>::Ptr GroundDetector2::getMergedRawPointCloud()
{
    return mMergedRawPointCloud;
}

std::vector<pcl::PointCloud<PointType>::Ptr> GroundDetector2::getDetect3dGround()
{
    return mGroundMeas;
}