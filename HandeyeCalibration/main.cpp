//#include <librealsense2/rs.hpp>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <unistd.h>

#define SUCCESS 0

#include "ICP.h"
//#include "util.hpp"

int main(int argc, char** argv)
{
    ICP_progress icp_handler;

    std::string format = "ply";

    // 读取点云文件，作为目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr draw_camera_standard;
    std::string path_standard = "00.txt";    
    int isRead_target = icp_handler.readPointCloud_target(path_standard);
    //auto soucePC = ReadIn("00.txt");
    draw_camera_standard = icp_handler.returnPointCloud("target"); 

    // 读取点云数据，作为源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr draw_camera_source;
    std::string path_source = "01.txt";    
    int isRead_source = icp_handler.readPointCloud_source(path_source);
    //auto soucePC = ReadIn("00.txt");
    draw_camera_source = icp_handler.returnPointCloud("source"); 

    // 对目标点云和源点云，都进行区域生长分割
    // 可以选择性使用，不是必须的
	draw_camera_standard = icp_handler.cut(draw_camera_standard);
	draw_camera_source = icp_handler.cut(draw_camera_source);

    // 设置目标点云和源点云
	icp_handler.setInput_target(draw_camera_standard);
	icp_handler.setInput_source(draw_camera_source);

    // ICP算法得到转换矩阵
	Eigen::Matrix4d ICPtrans;
	ICPtrans = icp_handler.getT_Source_to_Target();

	icp_handler.printMatrix(ICPtrans);

    // 画出ICP的结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr draw_target;
    pcl::PointCloud<pcl::PointXYZ>::Ptr draw_source;
	draw_target = icp_handler.returnPointCloud("target"); 
	draw_source = icp_handler.returnPointCloud("source"); 
	icp_handler.drawPointCloud(draw_target, draw_source, "1");

	return 1;
}