 /*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     System.hpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2019 / 08 / 17
*
*****************************************************************************/
#ifndef __SYSTEM_HPP__
#define __SYSTEM_HPP__

#include <vector>
#include <memory>
#include "Types.hpp"
#include "APIExport.hpp"

namespace cs
{
class ISystem;

/**
* @~chinese
* \defgroup System 相机检测
* @brief 发现相机、设置相机变动回调
* @{
* @~english
* \defgroup System Camera monitor
* @brief Discover camera, monitor camera change
* @{
*/

/** @brief camera state change callback */
typedef void (*CameraChangeCallback)(std::vector<CameraInfo>& addedCameras, std::vector<CameraInfo>& removedCameras, void * userData);

typedef std::shared_ptr<ISystem> ISystemPtr;

/*!\class ISystem
* @~chinese
* @brief 系统接口
* @~english 
* @brief System interface
**/
class CS_API ISystem
{
public:

	virtual ~ISystem() {};

	/**
	* @~chinese
	* @brief      获取当前可连接的相机列表
	* @param[out] cameras			：返回有效相机列表
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Query valid 3d cameras
	* @param[out] cameras		    ：return valid 3d cameras
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE queryCameras(std::vector<CameraInfo> &cameras) = 0;

	/**
	* @~chinese
	* @brief      设置相机连接状态变动回调
	* @param[in]  callback		：回调函数
	* @param[in]  userData		：用户数据指针
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Set camera state change callback
	* @param[in]  callback		：camera state change callback
	* @param[in]  userData		：pointer of user data
	* @return success:return SUCCESS, fail:other error code
	**/ 
    virtual ERROR_CODE setCameraChangeCallback(CameraChangeCallback callback, void *userData) = 0;

};

CS_API std::shared_ptr<ISystem> getSystemPtr();
/*@} */
}
#endif
