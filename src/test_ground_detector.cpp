#include "ground_detector/ground_detector.hpp"
#include "tic_toc.h"

int main(int argc, char **argv) 
{
    if (argc < 2) {
        printf("Usage: %s <point cloud> [<delta> <epsilon> <gamma> <theta>]\n", argv[0]);
        return -1;
    }

    // 计算距离图像和索引格式的点云
    const std::string pcd_file(argv[1]);
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *point_cloud) == -1) {
        std::cout << "Couldn't read pcd file!\n";
        return -1;
    }

    // 创建地面检测器
    double dgdSensorHeight = 1.8;
    int ngdNumIter = 3;
    int ngdNumLpr = 20;
    double dgdSeedTH = 1.2;
    double dgdDistanceTH = 0.2;
    GroundDetector gd(dgdSensorHeight, ngdNumIter, ngdNumLpr, dgdSeedTH, dgdDistanceTH);

    TicToc t_whole;

    gd.DetectGround(point_cloud);

    printf("whole detection time %f ms \n \n", t_whole.toc());

    // 显示容器
    pcl::PointXYZRGB rgb_point;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pointcloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr detect_ground(
      new pcl::PointCloud<pcl::PointXYZI>);
    
    detect_ground = gd.getRawPointCloud();

    // std::cout << "detect_ground->size(): " << detect_ground->size() << std::endl;

    pcl::PointXYZI point;
    for(int i=0; i<detect_ground->points.size(); i++)
    {   
        point = detect_ground->points[i];
        rgb_point.x = point.x;
        rgb_point.y = point.y;
        rgb_point.z = point.z;
        if(int(point.intensity)==1)
        {
            rgb_point.r = 255;
            rgb_point.g = 0;
            rgb_point.b = 0; 
        }
        else
        {
            rgb_point.r = 255;
            rgb_point.g = 255;
            rgb_point.b = 255; 
        }

        rgb_pointcloud->push_back(rgb_point);
    }

    // pcl 显示所有检测到的平面
    pcl::visualization::PCLVisualizer::Ptr visualizer(
        new pcl::visualization::PCLVisualizer("Ground Detection Visualizer"));
    visualizer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
        rgb_color_handler(rgb_pointcloud);
    visualizer->addPointCloud<pcl::PointXYZRGB>(rgb_pointcloud, rgb_color_handler,
                                                "RGB PointCloud");
    visualizer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "RGB PointCloud");
    visualizer->addCoordinateSystem(5.0);                                                                  

    while (!visualizer->wasStopped()) {
        visualizer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}