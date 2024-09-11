
#include "ground_detector/ground_detector.hpp"

/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool point_cmp(PointType a, PointType b){
    return a.z<b.z;
}

GroundDetector::GroundDetector(double sh, int ni, int nl, double ts, double td): sensor_height_(sh), 
num_iter_(ni), num_lpr_(nl), th_seeds_(ts), th_dist_(td)
{
    mRawPointCloud.reset(new pcl::PointCloud<PointType>());
    mSortAxisZMea.reset(new pcl::PointCloud<PointType>());
    mGroundMea.reset(new pcl::PointCloud<PointType>());
}

void GroundDetector::DetectGround(pcl::PointCloud<PointType>::Ptr RawPointCloud)
{

    mRawPointCloud->clear();
    mSortAxisZMea->clear();
    mGroundMea->clear();

    pcl::copyPointCloud(*RawPointCloud, *mRawPointCloud);
    pcl::copyPointCloud(*RawPointCloud, *mSortAxisZMea);
    
    std::sort(mSortAxisZMea->points.begin(),mSortAxisZMea->points.end(),point_cmp);
    
    // Error point removal
    mirror_reflection_();
    
    // Extract init ground seeds
    extract_initial_seeds_();
    
    // Ground plane fitter mainloop
    plane_fitter_();
}

void GroundDetector::mirror_reflection_()
{
    pcl::PointCloud<PointType>::iterator it = mSortAxisZMea->points.begin();
    for(size_t i=0;i<mSortAxisZMea->points.size();i++)
    {
        if(mSortAxisZMea->points[i].z < -1.5*sensor_height_)
            it++;
        else
            break;
    }
    mSortAxisZMea->points.erase(mSortAxisZMea->points.begin(),it);
}

// estimate plane 
void GroundDetector::estimate_plane_(void)
{
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*mGroundMea, cov, pc_mean);
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    normal_ = (svd.matrixU().col(2));
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    d_ = -(normal_.transpose()*seeds_mean)(0,0);
    th_dist_d_ = th_dist_ - d_;
}

// Extract initial seeds of the given pointcloud sorted segment
void GroundDetector::extract_initial_seeds_()
{
    double sum = 0;
    int cnt = 0;
    for(size_t i=0;i<mSortAxisZMea->points.size() && cnt<num_lpr_;i++)
    {
        sum += mSortAxisZMea->points[i].z;
        cnt++;
    }
    
    double lpr_height = cnt!=0?sum/cnt:0;
    for(size_t i=0;i<mSortAxisZMea->points.size();i++)
    {
        if(mSortAxisZMea->points[i].z < lpr_height + th_seeds_)
            mGroundMea->points.push_back(mSortAxisZMea->points[i]);
    }
}

// loop for org points segment
void GroundDetector::plane_fitter_()
{
    for(int i=0;i<num_iter_;i++)
    {
        estimate_plane_();
        mGroundMea->clear();

        //pointcloud to matrix
        MatrixXf points(mRawPointCloud->points.size(),3);
        int j =0;
        for(auto p:mRawPointCloud->points)
            points.row(j++)<<p.x,p.y,p.z;
        
        // ground plane model
        VectorXf result = points*normal_;
        
        // threshold filter
        for(int r=0;r<result.rows();r++)
        {
            if(result[r]<th_dist_d_)
            {
                mRawPointCloud->points[r].intensity = 1.0;
                mGroundMea->points.push_back(mRawPointCloud->points[r]);
            }
            else
            {
                mRawPointCloud->points[r].intensity = 0.0;
            }
        }
    }

    std::cout << "mGroundMea.size(): " << mGroundMea->size() << std::endl;
}

pcl::PointCloud<PointType>::Ptr GroundDetector::getRawPointCloud()
{
    return mRawPointCloud;
}