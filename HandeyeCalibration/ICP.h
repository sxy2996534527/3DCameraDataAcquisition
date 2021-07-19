#ifndef __ICP_H__
#define __ICP_H__

#include <memory>
#include <fstream>

//点云需要的头文件
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//矩阵变换需要的头文件
#include <math.h>
#include <Eigen/Dense>

//#include <librealsense2/rs.hpp>
#define PI 3.14159265

// isinf()
#include <stdlib.h>

// segmentation needs
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/io/io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vtkAutoInit.h>

/** @brief Alias for a 3D point */
using Point = pcl::PointXYZ;

class ICP_progress
{
    public:
        ICP_progress();
        ~ICP_progress();
        int init();
    
    public:
        int readPointCloud(std::string format, std::string path);
        //int readPointCloud_target(std::string format, std::string path);
        //int readPointCloud_source(std::string format, std::string path);
        int readPointCloud_target(const std::string& path);
        int readPointCloud_source(const std::string& path);
        pcl::PointCloud<pcl::PointXYZ>::Ptr returnPointCloud(std::string type);

        void setInput_target(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_tmp);
        void setInput_source(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_tmp);
        Eigen::Matrix4d getT_Source_to_Target();

        Eigen::MatrixXd cal_T(double x, double y, double z, double rx, double ry, double rz);
        std::vector<double> cal_pose(Eigen::MatrixXd T);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float size);

        void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outpath);
        void drawPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string titleName);
        void drawPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string titleName);
        void drawTransResult();
        void printMatrix(const Eigen::Matrix4d & matrix);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cut (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void quickSort(int left, int right, std::vector<double>& arr, std::vector<int>& arr_son);

    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointCloud_target;
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointCloud_source;

        //pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointCloud_trans;
        Eigen::Matrix4d m_transMatrix;
};

#endif