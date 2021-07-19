/*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     HandEye.hpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2020 / 05 / 09
*
*****************************************************************************/
#ifndef __HANDEYE_HPP__
#define __HANDEYE_HPP__

#include <vector>
#include "hpp/HandEye/HandEyeInput.hpp"
#include "hpp/Camera.hpp"
#include "hpp/Frame.hpp"
#include "hpp/APIExport.hpp"

namespace cs
{
/**
* @~chinese
* \defgroup HandEye 手眼标定
* @brief 提供手眼标定数据采集，参数计算，并使用标定结果将眼坐标系下的点或者向量转换到手的坐标系下
* @{
* @~english
* \defgroup HandEye HandEye Calibration
* @brief Provide functions for eye-hand calibration
* @{
*/

/**
* @~chinese
* @brief 枚举: 基线方向，请参照例子设置
* @~english
* @brief enumeration: The baseline direction
**/
typedef enum
{
	BASELINE_HORIZONTAL  = 0x00,	/**< @~chinese 水平基线		@~english baseline is horizontal*/ 
	BASELINE_VERTICAL    = 0x01,	/**< @~chinese 垂直基线		@~english baseline is vertical*/ 
}BASELINE_TYPE;

/**
* @~chinese
* @brief         重建标定板坐标
* @param[in]     camera                      : 已连接成功的相机指针
* @param[in]     pairFrame                   : 格式是STREAM_FORMAT_PAIR的帧数据
* @param[out]    points                      : 重建的标定板角点输出
* @param[in]     chessboardHorizontalPoints  : 横向标定板角点数
* @param[in]     chessboardVerticalPoints    : 竖向标定板角点数
* @param[in]     type                        : 基线方向, @参见 BASELINE_TYPE
* @return     成功:SUCCESS, 失败:其他错误码
* @~english
* @brief         reconstruct chessboard points
* @param[in]     camera                      : camera pointer, @see ICameraPtr
* @param[in]     pairFrame                   : the frame which format is STREAM_FORMAT_PAIR
* @param[out]    points                      : output the reconstructed chessboard points
* @param[in]     chessboardHorizontalPoints	 : number of horizontal chessboard points
* @param[in]     chessboardVerticalPoints    : number of vertical chessboard points
* @param[in]     type                        : baseline type, @see BASELINE_TYPE
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE reconstructChessboardPoints(ICameraPtr camera, IFramePtr pairFrame, std::vector<Point3f> &points, int chessboardHorizontalPoints = 9, int chessboardVerticalPoints = 6, BASELINE_TYPE type = BASELINE_HORIZONTAL);

/**
* @~chinese
* @brief         重建标定板坐标
* @param[in]     camera                      : 已连接成功的相机指针
* @param[in]     imageLeft                   : 左相机图像
* @param[in]     imageRight                  : 右相机图像
* @param[in]     width	                     : 图像宽度
* @param[in]     height						 : 图像高度
* @param[out]    points                      : 重建的标定板角点输出
* @param[in]     chessboardHorizontalPoints  : 横向标定板角点数
* @param[in]     chessboardVerticalPoints    : 竖向标定板角点数
* @param[in]     type                        : 基线方向, @参见 BASELINE_TYPE
* @return     成功:SUCCESS, 失败:其他错误码
* @~english
* @brief         reconstruct chessboard points
* @param[in]     camera                      : camera pointer, @see ICameraPtr
* @param[in]     imageLeft                   : left image
* @param[in]     imageRight                  : right image
* @param[in]     width	                     : the width of image
* @param[in]     height						 : the height of image
* @param[out]    points                      : output the reconstructed chessboard points
* @param[in]     chessboardHorizontalPoints	 : number of horizontal chessboard points
* @param[in]     chessboardVerticalPoints    : number of vertical chessboard points
* @param[in]     type                        : baseline type, @see BASELINE_TYPE
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE reconstructChessboardPoints(ICameraPtr camera, const unsigned char* imageLeft, const unsigned char* imageRight, int width, int height, std::vector<cs::Point3f> &points, int chessboardHorizontalPoints = 9, int chessboardVerticalPoints = 6, BASELINE_TYPE type = BASELINE_HORIZONTAL);

/**
* @~chinese
* @brief         通过左右相机图片中提取的特征点坐标计算到三维点坐标
* @param[in]     camera                      : 已连接成功的相机指针
* @param[in]     leftPoints                  : 左相机图片中提取的特征点坐标
* @param[in]     rightPoints                 : 右相机图片中提取的特征点坐标
* @param[out]    points                      : 重建输出的三维点坐标
* @param[in]     type                        : 基线类型
* @return 成功:SUCCESS, 失败:其他错误码
* @~english
* @brief         reconstruct feature points
* @param[in]     camera                      : camera pointer, @see ICameraPtr
* @param[in]     leftPoints                  : left picture feature points
* @param[in]     rightPoints                 : right picture feature points
* @param[out]    points                      : reconstructed points
* @param[in]     type                        : baseline type, @see BASELINE_TYPE
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE reconstructPoints(ICameraPtr camera, std::vector<Point2f> leftPoints, std::vector<Point2f> rightPoints, std::vector<Point3f> &points, BASELINE_TYPE type = BASELINE_HORIZONTAL);

/**
* @~chinese
* @brief         相机在机器人外部时计算旋转平移矩阵
* @param[in]     input           : 相机在机器人外部时的标定输入
* @param[out]    matrix          : 输出的眼到手矩阵
* @param[out]    error           : 输出误差
* @return 成功:SUCCESS, 失败:其他错误码
* @~english
* @brief         calculate eye to hand matrix
* @param[in]     input           : eye to hand calibration input
* @param[out]    matrix          : eye to hand matrix
* @param[out]    error           : convergence error
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE calibrateEyeToHand(std::vector<HandEyeCalibrationInput>& input, HandEyeMatrix& matrix, double* error = NULL);

/**
* @~chinese
* @brief         相机固定在机器人上时计算旋转平移矩阵
* @param[in]     input           : 相机固定在机器人上时的标定输入
* @param[out]    matrix          : 输出的眼到手矩阵
* @param[out]    error           : 输出误差
* @return 成功:SUCCESS, 失败:其他错误码
* @~english
* @brief         calculate eye in hand matrix
* @param[in]     input           : eye in hand calibration input
* @param[out]    matrix          : eye to hand matrix
* @param[out]    error           : convergence error
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE calibrateEyeInHand(std::vector<HandEyeCalibrationInput>& input, HandEyeMatrix& matrix, double* error = NULL);

/**
* @~chinese
* @brief         将相机坐标系下的点转到机器人坐标系
* @param[in]     pose           : 机器人当前位姿
* @param[in]     inputPoint     : 相机坐标系下的点
* @param[in]     matrix         : 眼到手矩阵，calibrateEyeToHand/calibrateEyeInHand计算得到
* @param[out]    outPoint       : 输出到机器人坐标系下的点
* @return 成功:SUCCESS, 失败:其他错误码
* @~english
* @brief         transform point to the coordinate system of hand
* @param[in]     pose           : the pose of robot
* @param[in]     inputPoint     : the input point in eye
* @param[in]     matrix         : the matrix eye to hand, returned by calibrateEyeToHand/calibrateEyeInHand
* @param[out]    outPoint       : output the point in hand
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE transformPointFromEyeToHand(RobotPoseMatrix4f pose, Point3f inputPoint, HandEyeMatrix matrix, Point3f& outPoint);

/**
* @~chinese
* @brief         将相机坐标系下的向量转到机器人坐标系
* @param[in]     pose           : 机器人当前位姿
* @param[in]     inputVector    : 相机坐标系下的向量
* @param[in]     matrix         : 眼到手矩阵，calibrateEyeToHand/calibrateEyeInHand计算得到
* @param[out]    outVector      : 输出到机器人坐标系下的向量
* @return 成功:SUCCESS, 失败:其他错误码
* @~english
* @brief         transform vector to the coordinate system of hand
* @param[in]     pose           : the pose of robot
* @param[in]     inputVector    : the input vector in eye
* @param[in]     matrix         : the matrix eye to hand, returned by calibrateEyeToHand/calibrateEyeInHand
* @param[out]    outVector      : output the vector in hand
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE transformVectorFromEyeToHand(RobotPoseMatrix4f pose, Point3f inputVector, HandEyeMatrix matrix, Point3f& outVector);

/**
* @~chinese
* @brief         将相机坐标系下的位姿转到机器人坐标系
* @param[in]     toolPose       : 机器人当前位姿
* @param[in]     poseInEye      : 相机坐标系下的位姿
* @param[in]     matrix         : 眼到手矩阵，calibrateEyeToHand/calibrateEyeInHand计算得到
* @param[in]	 poseType		：输出位姿表示类型
* @param[in]	 poseUnit		: 输出姿态表示单位角度/弧度
* @param[out]    outPoseInHand  : 输出位姿
* @return 成功:SUCCESS, 失败:其他错误码
* @~english
* @brief         transform point to the coordinate system of hand
* @param[in]     toolPose       : the pose of robot
* @param[in]     poseInEye      : the input point in eye
* @param[in]     matrix         : the matrix eye to hand, returned by calibrateEyeToHand/calibrateEyeInHand
* @param[in]	 poseType		：type of output pose
* @param[in]	 poseUnit		: unit of output direction(radian/degree)
* @param[out]    outPoseInHand  : the output pose in hand
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE transformPoseFromEyeToHand(RobotPoseMatrix4f toolPose, RobotPoseMatrix4f poseInEye, HandEyeMatrix matrix,
											  RobotPoseType poseType, RobotPoseUnit poseUnit, RobotPose& outPoseInHand);

/**
* @~chinese
* @brief         将相机坐标系下的位姿转到机器人坐标系
* @param[in]     toolPose       : 机器人当前位姿
* @param[in]     poseInEye      : 相机坐标系下的位姿
* @param[in]     matrix         : 眼到手矩阵，calibrateEyeToHand/calibrateEyeInHand计算得到
* @param[out]    outMatrixInHand  : 输出位姿
* @return 成功:SUCCESS, 失败:其他错误码
* @~english
* @brief         transform point to the coordinate system of hand
* @param[in]     toolPose       : the pose of robot
* @param[in]     poseInEye      : the input point in eye
* @param[in]     matrix         : the matrix eye to hand, returned by calibrateEyeToHand/calibrateEyeInHand
* @param[out]    outMatrixInHand  : the output matrix in hand
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE transformPoseFromEyeToHand(RobotPoseMatrix4f toolPose, RobotPoseMatrix4f poseInEye, HandEyeMatrix matrix,
	RobotPoseMatrix4f outMatrixInHand);

/*@} */
}
#endif
