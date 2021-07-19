#include "ICP.h"

ICP_progress::ICP_progress()
{

}

ICP_progress::~ICP_progress()
{

}

int ICP_progress::init()
{
    return 1;
}

// 读取点云
int ICP_progress::readPointCloud(std::string format, std::string path)
{
    if (format == "ply")
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) 
        {       
            PCL_ERROR("Couldnot read file.\n");
            return 0;
        }
    }
    return 1;
}

// 读取点云，并将此点云作为目标点云
// int ICP_progress::readPointCloud_target(std::string format, std::string path)
// {
//     if (format == "ply")
//     {
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) 
//         {       
//             PCL_ERROR("Couldnot read file.\n");
//             return 0;
//         }
//         m_pointCloud_target = cloud;
//     }
//     std::cout<<"target pointcloud size: "<<m_pointCloud_target->width<<" * "<<m_pointCloud_target->height << std::endl; 
//     return 1;
// }

int ICP_progress::readPointCloud_target(const std::string& path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //auto cloud = ReadIn(path);
    std::ifstream file(path);

    float x, y, z;
    while (file >> x >> y >> z)
    {
        cloud->points.push_back(Point{ x, y, z });
    }
    m_pointCloud_target = cloud;
    std::cout << "target pointcloud size: " << int(m_pointCloud_target->points.size()) << std::endl; 
    return 1;
}

// 读取点云，并将此点云作为源点云
// int ICP_progress::readPointCloud_source(std::string format, std::string path)
// {
//     if (format == "ply")
//     {
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) 
//         {       
//             PCL_ERROR("Couldnot read file.\n");
//             return 0;
//         }
//         m_pointCloud_source = cloud;
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
//         if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) 
//         {       
//             PCL_ERROR("Couldnot read file.\n");
//             return 0;
//         }
//         m_pointCloud_trans = cloud1;
//     }
//     std::cout<<"source pointcloud size: "<<m_pointCloud_source->width<<" * "<<m_pointCloud_source->height << std::endl; 
//     return 1;
// }

int ICP_progress::readPointCloud_source(const std::string& path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //auto cloud = ReadIn(path);
    std::ifstream file(path);

    float x, y, z;
    while (file >> x >> y >> z)
    {
        cloud->points.push_back(Point{ x, y, z });
    }
    m_pointCloud_source = cloud;
    std::cout << "source pointcloud size: " << int(m_pointCloud_source->points.size()) << std::endl; 
    return 1;
}

// 返回想要的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_progress::returnPointCloud(std::string type)
{
    if (type == "target")
    {
        return m_pointCloud_target;
    }
    if (type == "source")
    {
        return m_pointCloud_source;
    }
    // if (type == "trans")
    // {
    //     return m_pointCloud_trans;
    // }
}

// 设置输入点云为目标点云
void ICP_progress::setInput_target(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_tmp)
{
    m_pointCloud_target = pointCloud_tmp;
}

// 设置输入点云为源点云
void ICP_progress::setInput_source(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_tmp)
{
    m_pointCloud_source = pointCloud_tmp;
}

// ICP配准算法
Eigen::Matrix4d ICP_progress::getT_Source_to_Target()
{
    int iterations = 200; // 设置迭代次数
    pcl::console::TicToc time;
    std::cerr << "Target PointCloud has : " << m_pointCloud_target->width * m_pointCloud_target->height << " data points before DownSampling." << std::endl;
    std::cerr << "source PointCloud has : " << m_pointCloud_source->width * m_pointCloud_source->height << " data points before DownSampling." << std::endl;
    m_pointCloud_target = downSampling(m_pointCloud_target, 0.2f);
    m_pointCloud_source = downSampling(m_pointCloud_source, 0.2f);
    std::cerr << "Target PointCloud has : " << m_pointCloud_target->width * m_pointCloud_target->height << " data points after DownSampling." << std::endl;
    std::cerr << "Source PointCloud has : " << m_pointCloud_source->width * m_pointCloud_source->height << " data points after DownSampling." << std::endl;

    time.tic ();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (m_pointCloud_source);
    icp.setInputTarget (m_pointCloud_target);

    icp.align (*m_pointCloud_source); // 这句话会使用ICP匹配结果将源点云进行变换，变换后的点云重新再赋给源点云
    
    icp.setMaximumIterations (1);
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;

        transformation_matrix = icp.getFinalTransformation ().cast<double>(); // ICP匹配的结果
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        std::cerr << "FFFFFFFFFFFFFFFuck!!!!!!!!!!" << std::endl;
    }

    m_transMatrix = transformation_matrix;

    return transformation_matrix;
}

// 降采样
pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_progress::downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float size)
{
    std::cout << "Downsampling with leaf size: " << size << "..." << std::endl; 
    pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2), cloud_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::toPCLPointCloud2(*cloud, *cloud_in);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(size, size, size);
    sor.filter(*cloud_blob);

    pcl::fromPCLPointCloud2(*cloud_blob, *cloud_out);

    return cloud_out;
}

// 保存点云
void ICP_progress::savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outpath)
{
    std::cerr << "save path is :"<< outpath<< endl;
    //将string保存路径转为char*
    char *path = new char[outpath.size() +1];
    strcpy(path , outpath.c_str());
    std::cerr << "Path is : " << path << " ." << std::endl;
	
    //写出点云图
    pcl::PLYWriter writer;
    writer.write(path, *cloud, true);
    std::cerr << "PointCloud has : " << cloud->width * cloud->height << " data points." << std::endl;
}

// 画出点云
void ICP_progress::drawPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string titleName)
{
    pcl::visualization::PCLVisualizer viewer (titleName);
    int v (0);

    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, v);

    viewer.addCoordinateSystem(0.5);

    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h (cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud, cloud_in_color_h, "cloud_in_v1", v);

    viewer.addText (titleName, 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v);

    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v);

    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

// 两片点云画在一起
void ICP_progress::drawPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string titleName)
{
    pcl::visualization::PCLVisualizer viewer (titleName);
    int v (0);

    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, v);

    viewer.addCoordinateSystem(0.5);

    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl= 1.0 - bckgr_gray_level;

    // 第一个点云画成红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h1 (cloud1, (int) 255 * 1, (int) 255 * 0, (int) 255 * 0);
    viewer.addPointCloud (cloud1, cloud_in_color_h1, "cloud_in_v1", v); // 字符串"cloud_in_v1"代表

    // 第一个点云画成蓝色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h2 (cloud2, (int) 255 * 0, (int) 255 * 0, (int) 255 * 1);
    viewer.addPointCloud (cloud2, cloud_in_color_h2, "cloud_in_v2", v);

    // 增加文本
    viewer.addText (titleName, 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v);
    // 设置背景颜色为黑
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v);

    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

// 六维pose转变换矩阵
Eigen::MatrixXd ICP_progress::cal_T(double x, double y, double z, double rx, double ry, double rz)
{
    Eigen::MatrixXd Rx(3, 3);
    Eigen::MatrixXd Ry(3, 3);
    Eigen::MatrixXd Rz(3, 3);
    
    Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
    Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
    Rz << cos(rz), - sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;

    Eigen::MatrixXd R(3, 3);
    R = Rz * Ry * Rx;

    Eigen::MatrixXd P(3, 1);
    P << x, y, z;

    Eigen::MatrixXd T(4,4);
    T << R, P, Eigen::MatrixXd::Zero(1, 3), Eigen::MatrixXd::Identity(1,1);

    return T;
}

// 变换矩阵转六维pose
std::vector<double> ICP_progress::cal_pose(Eigen::MatrixXd T)
{
    double x = T(0, 3);
    double y = T(1, 3);
    double z = T(2, 3);
    double rx = atan2(T(2, 1), T(2, 2));
    double ry = asin(-T(2, 0));
    double rz = atan2(T(1, 0), T(0, 0));

    std::vector<double> pose;
    pose.push_back(x);
    pose.push_back(y);
    pose.push_back(z);
    pose.push_back(rx);
    pose.push_back(ry);
    pose.push_back(rz);

    return pose;
}

// 矩阵打印
void ICP_progress::printMatrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
  printf ("Transform matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
  printf ("T = | %6.3f %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", matrix (3, 0), matrix (3, 1), matrix (3, 2), matrix (3, 3));
}

// 快排算法
// quickSort(数组id起始位置(0), 数组id最后位置(size-1), 想要排序的父数组，想根据父数组排序同时排序的子数组)
void ICP_progress::quickSort(int left, int right, std::vector<double>& arr, std::vector<int>& arr_son)
{
    if (left >= right)
    {
	    return;
    }
    int i, j, base, temp;
    i = left;
    j = right;
    base = arr[left];

    int base_son;
    base_son = arr_son[left];
    int temp_son;
    while( i < j)
    {
        while (arr[j] >= base && i < j)
        {
            j--;
        }
        while (arr[i] <= base && i < j)
        {
            i++;
        }
        if (i < j)
        {
            temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;

            temp_son = arr_son[i];
            arr_son[i] = arr_son[j];
            arr_son[j] = temp_son;
        }
    }
    arr[left] = arr[i];
    arr[i] = base;
    arr_son[left] = arr_son[i];
    arr_son[i] = base_son;
    quickSort(left, i-1, arr, arr_son);
    quickSort(i+1, right, arr, arr_son);
}

// 用区域生长的办法，分割出满足需要的点云簇
pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_progress::cut (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::cout<<"segmentation input pointcloud size: "<<cloud->points.size()<<std::endl;
    
    // 1. 进行滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(cloud);
    voxelSampler.setLeafSize(0.5f, 0.5f, 0.5f);
    voxelSampler.filter(*cloud);
    	
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
    statFilter.setInputCloud(cloud);
    statFilter.setMeanK(10);
    statFilter.setStddevMulThresh(0.2);
    statFilter.filter(*cloud);
    std::cout<<"[segmentation] voxel filter finished!"<<std::endl;
	
    // 2. 计算点云的法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // 设置搜索方式
    // ne.setRadiusSearch(10); // 设置欧式距离搜索 
    ne.setKSearch(10); // 设置为K近邻搜索，较好用
    ne.compute(*normals);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    // 设置滤波轴，我设置为z轴，也就是相机的景深方向
    pass.setFilterFieldName("z"); 
    // 设置滤波范围，我只取相机z轴的-1米到1米范围内的数据点(我的相机比较特殊，有z值小于0的情况)
    pass.setFilterLimits(-1000, 1000); 
    std::cout<<"[segmentation] compute normals finished!"<<std::endl;

    // 3. 进行区域生长
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(500);
    reg.setMaxClusterSize(10000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(500);
    reg.setInputCloud(cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(50.0/ 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    std::cout<<"[segmentation] grow finished!"<<std::endl;

    // 输出区域生长后，分割出的点云簇数
    std::cout<<"Number of cluster is equal to " << clusters.size()<<std::endl;
    // id为0的点云簇中，点云的个数
    std::cout<<"First cluster has "<<clusters[0].indices.size()<<" points."<<std::endl;

    // 由于点云簇的id没有规则，我要按照每簇点云数据在相机z轴上的平均值，进行排序
    std::vector<double> z_dist;
    std::vector<int> z_dist_id;
    for (int i = 0 ; i < clusters.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, clusters[i].indices, *cloud_part);
        double z_sum = 0;
        for (int j = 0 ; j < cloud_part->points.size(); j++)
        {
            z_sum = z_sum + cloud_part->points[i].z;
        }
        double z_mean = z_sum/ cloud_part->points.size();
    
        z_dist.push_back(z_mean);
        z_dist_id.push_back(i);
    }

    // std::cout<<"before sort:"<<std::endl;
    for (int i = 0 ; i < clusters.size(); i++)
    {
        // std::cout<<"segmentation cluster z value:"<<z_dist[i]<<" its id: "<<z_dist_id[i]<<std::endl;
    }
    quickSort(0, clusters.size(), z_dist, z_dist_id);
    // std::cout<<"after sort:"<<std::endl;
    for (int i = 0 ; i < clusters.size(); i++)
    {
        // std::cout<<"segmentation cluster z value:"<<z_dist[i]<<" its id: "<<z_dist_id[i]<<std::endl;
    }

    // 以自己的标准，来筛选满足要求的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objectonly(new pcl::PointCloud<pcl::PointXYZ>);
    int x = 0;
    while (true)
    {
        if (z_dist_id[clusters.size()-1-x] >=0 && z_dist_id[clusters.size()-1-x] < 10 
            && z_dist[clusters.size()-1-x] < 100 && z_dist[clusters.size()-1-x] > -100 )
        {
            pcl::copyPointCloud(*cloud, clusters[z_dist_id[clusters.size()-1-x]].indices, *cloud_objectonly);
            break;
        }
	    x++;	
    }
    
    int cluster_need_num = 2; // 在满足要求的点云中，最后保留的簇数
    int num = 0;
    for (int i = x+1 ; i < clusters.size(); i++)
    {
        if (z_dist_id[clusters.size()-1-i] >=0 && z_dist_id[clusters.size()-1-i] < 100 
            && z_dist[clusters.size()-1-i] < 0 && z_dist[clusters.size()-1-i] > -100 )
        {
            int id = z_dist_id[clusters.size()-1-i];
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud, clusters[id].indices, *cloud_tmp);

            std::cout<<"segmentation keep id: "<<id<<" cluster pointcloud"<<std::endl;
            *cloud_objectonly = (*cloud_objectonly) + (*cloud_tmp);
            num ++;
        }
        if (num > cluster_need_num)
        {
            break;
        }
    }

	std::cout<<"After segmentation pointcloud of object size: "<<cloud_objectonly->points.size()<<std::endl;
    return cloud_objectonly;
}