/*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     HandEyeInput.hpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2020 / 05 / 09
*
*****************************************************************************/
#ifndef __HANDEYEINPUT_HPP__
#define __HANDEYEINPUT_HPP__

#include <vector>
#include "hpp/APIExport.hpp"

namespace cs
{
/**
* \ingroup HandEye
* @{
*/

/**
* @~chinese
* @brief 枚举: 当前位姿方向的表示方法
* @~english
* @brief enumeration: The type of direction
**/
typedef enum RobotPoseType
{
	POSE_TYPE_EULER_ZYZ = 0,	/**< @~chinese 欧拉ZYZ		@~english EULER ZYZ*/ 
	POSE_TYPE_EULER_XYZ = 1,	/**< @~chinese 欧拉XYZ		@~english EULER XYZ*/ 
	POSE_TYPE_EULER_ZYX = 2,	/**< @~chinese 欧拉ZYX		@~english EULER ZYX*/ 
	POSE_TYPE_RPY		= 3,	/**< @~chinese RPY			@~english RPY*/ 
	POSE_TYPE_QUATERNION = 4,	/**< @~chinese 四元数表示法	@~english Quaternion*/
	POSE_TYPE_XYZAB		= 5,	/**< @~chinese 用于五轴机床	@~english used by 5-axis machine*/
}RobotPoseType;

/**
* @~chinese
* @brief 枚举: 当前位姿方向的表示单位
* @~english
* @brief enumeration: The unit of direction
**/
typedef enum RobotPoseUnit
{
	POSE_UNIT_RADIAN = 0,	/**<@~chinese 弧度	@~english in radian */
	POSE_UNIT_DEGREE		/**<@~chinese 角度	@~english in degree */
}RobotPoseUnit;

/**
* @~chinese
* @brief 机器人位姿
* @~english
* @brief pose in hand
**/
typedef struct RobotPose
{
	float x;        /**<@~chinese 单位毫米	@~english unit mm */
	float y;        /**<@~chinese 单位毫米	@~english unit mm */
	float z;        /**<@~chinese 单位毫米	@~english unit mm */
	float alfa;     /**<@~chinese 姿态表示元素1	@~english pose representation element 1 */
	float beta;     /**<@~chinese 姿态表示元素2	@~english pose representation element 2 */
	float gamma;    /**<@~chinese 姿态表示元素3	@~english pose representation element 3 */
	float theta;    /**<@~chinese 姿态表示元素4	@~english pose representation element 4 */
}RobotPose;

typedef struct Point3f {
	float x;
	float y;
	float z;
}Point3f;

typedef struct Point2f {
	float x;
	float y;
}Point2f;

/*!\class RobotPoseMatrix4f
* @~chinese
* @brief 机器人位姿的4x4矩阵表示
* @~english
* @brief pose of robot in matrix 4x4
**/
class CS_API RobotPoseMatrix4f
{
public:
	RobotPoseMatrix4f();
	/**
	* @~chinese
	* @brief      通过机器人当前的位姿及位姿表示类型构造4x4矩阵
	* @param[in]  pose				：输入当前位姿
	* @param[in]  type				：位姿表示类型
	* @param[in]  unit				: 位姿表示角度/弧度
	* @~english
	* @brief      Start stream and return frame by callback
	* @param[in]  pose				：input pose
	* @param[in]  type				：type of pose
	* @param[in]  unit				: unit of direction(radian/degree)
	**/
	RobotPoseMatrix4f(RobotPose pose, RobotPoseType type, RobotPoseUnit unit);
	float r00;
	float r01;
	float r02;
	float tx;
	float r10;
	float r11;
	float r12;
	float ty;
	float r20;
	float r21;
	float r22;
	float tz;
	float zero0;
	float zero1;
	float zero2;
	float one;
};

/**
* @~chinese
* @brief 手眼标定矩阵
* @~english
* @brief eye to hand matrix
**/
typedef struct HandEyeMatrix
{
	float r00;
	float r01;
	float r02;
	float tx;
	float r10;
	float r11;
	float r12;
	float ty;
	float r20;
	float r21;
	float r22;
	float tz;
	float zero0;
	float zero1;
	float zero2;
	float one;
}HandEyeMatrix;

/*!\class HandEyeCalibrationInput
* @~chinese
* @brief 手眼标定输入类
* @~english
* @brief calibration input
**/
class CS_API HandEyeCalibrationInput
{
public:
	RobotPoseMatrix4f m_pose;
	std::vector<Point3f> m_points;
public:
	HandEyeCalibrationInput(HandEyeCalibrationInput&& other);

	HandEyeCalibrationInput& operator=(HandEyeCalibrationInput other);

	HandEyeCalibrationInput(const HandEyeCalibrationInput& other);

	~HandEyeCalibrationInput();
	/**
	* @~chinese
	* @brief      通过机器人当前的位姿构造手眼标定输入
	* @param[in]  pose				: 4x4矩阵表示的当前位姿
	* @param[in]  points			: 相机坐标系下标定板角点坐标
	* @~english
	* @brief      construction HandEyeCalibrationInput from current hand pose and chessboard points in eye
	* @param[in]  pose				: current hand pose in 4x4 matrix
	* @param[in]  points			: chessboard points in eye
	**/
	HandEyeCalibrationInput(RobotPoseMatrix4f pose, std::vector<Point3f>& points);
};
/*@} */
}
#endif
