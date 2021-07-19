 /*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     Types.hpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2019 / 08 / 17
*
*****************************************************************************/
#ifndef __TYPES_HPP__
#define __TYPES_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#define FRAMERATE_ANY 0

/** 
* @~chinese
* 枚举: 返回的错误码
* @~english
* enumeration: returned error code
**/
typedef enum ERROR_CODE
{
	SUCCESS = 0,					/**< @~chinese 成功				@~english success*/ 
	ERROR_PARAM,					/**< @~chinese 参数输入错误		@~english param input error*/
	ERROR_DEVICE_NOT_FOUND,			/**< @~chinese 未找到设备		@~english device not found*/
	ERROR_DEVICE_NOT_CONNECT,		/**< @~chinese 设备未连接		@~english device not connected*/
	ERROR_DEVICE_BUSY,				/**< @~chinese 设备忙			@~english device busy*/
	ERROR_STREAM_NOT_START,			/**< @~chinese 流尚未打开		@~english stream not start*/
	ERROR_STREAM_BUSY,				/**< @~chinese 流已打开			@~english stream had started*/
	ERROR_FRAME_TIMEOUT,			/**< @~chinese 获取帧数据失败		@~english get frame failed*/
	ERROR_NOT_SUPPORT,				/**< @~chinese 尚不支持			@~english not support*/
	ERROR_PROPERTY_GET_FAILED,		/**< @~chinese 获取属性失败		@~english get property failed*/
	ERROR_PROPERTY_SET_FAILED		/**< @~chinese 设置属性失败		@~english set property failed*/
}ERROR_CODE;

/**
* @~chinese
* 枚举: 相机流类型
* @~english
* enumeration: stream type
**/
typedef enum STREAM_TYPE
{
	STREAM_TYPE_DEPTH	= 0, /**<@~chinese 深度流	@~english Depth camera stream */
    STREAM_TYPE_RGB		= 1, /**<@~chinese RGB流		@~english RGB camera stream */
	STREAM_TYPE_COUNT
}STREAM_TYPE;


/// \~chinese
/// \defgroup StreamFormat 数据流格式
/// \brief 深度流和RGB流所支持的所有格式
/// @{
/// \~english
/// \defgroup StreamFormat Stream format
/// \brief Format of depth stream and RGB stream
/// @{
/**
* @~chinese
* 枚举: 流数据格式
* @~english
* enumeration: stream format
**/
typedef enum STREAM_FORMAT
{
	STREAM_FORMAT_MJPG		= 0x00,		 /**< @~chinese RGB流的MJPG压缩的数据			
											  @~english MJPG compressed data*/ 
	STREAM_FORMAT_RGB8		= 0x01,		 /**< @~chinese RGB流的8位红,绿,蓝3通道数据			
											  @~english 8-bit red, green and blue channels*/ 
	STREAM_FORMAT_Z16		= 0x02,		 /**< @~chinese 深度流的深度图格式, 每一个深度值以unsigned short表示
											  @~english 16-bit unsigned short depth values. The depth in millimeters is equal to depth scale * pixel value. */ 
	STREAM_FORMAT_Z16Y8Y8	= 0x03,		 /**< @~chinese 深度流的深度图+红外图组合格式,	
														通过FRAME_DATA_FORMAT_Z16获得深度数据，
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图, 
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图
											  @~english output depth map and infrared, 
														get depth map by FRAME_DATA_FORMAT_Z16
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/ 
	STREAM_FORMAT_PAIR		= 0x04,		 /**< @~chinese 深度流的红外图格式，
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图, 
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图	
											  @~english output infrared，
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/ 
}STREAM_FORMAT;

/**
* @~chinese
* 枚举: 帧数据格式，用于获取复合流数据中的指定格式数据起始地址
* @~english
* enumeration: format of frame data, used for get specified data in a composite frame
**/
typedef enum FRAME_DATA_FORMAT
{
	FRAME_DATA_FORMAT_Z16				= 0x00,		/**< @~chinese 深度流的深度图格式, 每一个深度值以unsigned short表示
														 @~english 16-bit unsigned short depth values. The depth in millimeters is equal to depth scale * pixel value. */ 
	FRAME_DATA_FORMAT_IR_LEFT			= 0x01,		/**< @~chinese 左红外图数据， 8-bit unsigned char表示一个灰度值				
														 @~english 8-bit unsigned char gray level of left infrared*/
	FRAME_DATA_FORMAT_IR_RIGHT			= 0x02,		/**< @~chinese 右红外图数据， unsigned char表示一个灰度值				
														 @~english 8-bit unsigned char gray level of right infrared*/
}FRAME_DATA_FORMAT;
/// @}


/// \~chinese
/// \defgroup PropertyType 基础属性
/// \brief 列举所有可设置的基础属性
/// @{
/// \~english
/// \defgroup PropertyType Basic property
/// \brief List basic properties
/// @{

/**
* @~chinese
* 枚举: 相机的基本属性
* @~english
* enumeration: basic property of camera
**/
typedef enum PROPERTY_TYPE
{
	PROPERTY_GAIN						= 0x00,	/**<@~chinese 增益				@~english gain of depth camera or RGB camera*/
	PROPERTY_EXPOSURE					= 0x01,	/**<@~chinese 曝光值				@~english Controls exposure time of depth camera or RGB camera*/
	PROPERTY_FRAMETIME					= 0x02,	/**<@~chinese 帧时间				@~english Frame time of depth camera */
	PROPERTY_FOCUS						= 0x03,	/**<@~chinese 焦距				@~english Focus of RGB camera*/
	PROPERTY_ENABLE_AUTO_FOCUS			= 0x04,	/**<@~chinese 是否自动对焦		@~english Enable / disable auto-focus of RGB camera*/
	PROPERTY_ENABLE_AUTO_EXPOSURE		= 0x05, /**<@~chinese 是否自动曝光		@~english Enable / disable auto-exposure of RGB camera*/
	PROPERTY_ENABLE_AUTO_WHITEBALANCE	= 0x06, /**<@~chinese 是否自动白平衡		@~english White balance of RGB camera*/
	PROPERTY_WHITEBALANCE				= 0x07,	/**<@~chinese 白平衡值			@~english adjust white balance of RGB camera*/
	PROPERTY_WHITEBALANCE_R				= 0x08,	/**<@~chinese 白平衡R通道		@~english Channel r of RGB camera, adjust white balance*/
	PROPERTY_WHITEBALANCE_B				= 0x09,	/**<@~chinese 白平衡B通道		@~english Channel b of RGB camera, adjust white balance*/
	PROPERTY_WHITEBALANCE_G				= 0x10,	/**<@~chinese 白平衡G通道		@~english Channel g of RGB camera, adjust white balance*/
} PROPERTY_TYPE;
/// @}

/**
* @~chinese
* 枚举: 相机触发模式 
* @~english
* enumeration: trigger mode
**/
typedef enum TRIGGER_MODE
{
	TRIGGER_MODE_OFF		= 0, /**< @~chinese 关闭触发模式，持续输出深度流	
									  @~english output depth map continuously*/ 
	TRIGGER_MODE_HARDWAER	= 1, /**< @~chinese 外触发模式，需要在触发口输入硬件信号才能出图
									  @~english external trigger mode，you should input hardware pulse to get depth frame*/
	TRIGGER_MODE_SOFTWAER	= 2, /**< @~chinese 软触发模式，需要调用cs::ICamera::softTrigger才能出深度图
									  @~english software trigger mode，you should call cs::ICamera::softTrigger to get depth frame*/
}TRIGGER_MODE;

/**
* @~chinese
* 枚举: 高动态的模式
* @~english
* enumeration: mode of HDR
**/
typedef enum HDR_MODE
{
	HDR_MODE_OFF			= 0,	/**< @~chinese 关闭				@~english HDR off*/ 
	HDR_MODE_HIGH_RELECT	= 1,	/**< @~chinese 适用于测高反物体	@~english suitable for shiny object*/
	HDR_MODE_LOW_RELECT		= 2,	/**< @~chinese 适用于测深色物体	@~english suitable for dark object*/
	HDR_MODE_ALL_RELECT		= 3		/**< @~chinese 适用于测复合表面	@~english suitable for composite object*/
}HDR_MODE;

/**
* @~chinese
* @brief 枚举: 自动曝光模式
* @~english
* @brief enumeration: mode of auto exposure
**/
typedef enum AUTO_EXPOSURE_MODE
{
	AUTO_EXPOSURE_MODE_CLOSE = 0,			/**< @~chinese 关闭				@~english off*/
	AUTO_EXPOSURE_MODE_FIX_FRAMETIME = 1,	/**< @~chinese 在不改变帧率的前提下自动调节曝光时间
											@~english adjust exposure automatically and keep frame time unchanged*/
	AUTO_EXPOSURE_MODE_HIGH_QUALITY = 2		/**< @~chinese 自动调节曝光时间，会按需改变帧率
											@~english adjust exposure and frame time automatically*/
}AUTO_EXPOSURE_MODE;

/**
* @~chinese
* @brief 枚举: 网络传输压缩方式
* @~english
* @brief enumeration: mode of compress
**/
typedef enum NETWORK_COMPRESS_MODE
{
	NETWORK_COMPRESS_MODE_CLOSE = 0,		/**< @~chinese 关闭				@~english off*/
	NETWORK_COMPRESS_MODE_ZIP	= 1,		/**< @~chinese ZIP(默认设置)     @~english ZIP(Default)*/
}NETWORK_COMPRESS_MODE;


/**
* @~chinese
* @brief 测量深度范围，超出范围的值将被置零
* @~english
* @brief range of depth， value out of range will be set to zero
**/
typedef struct DepthRange
{
	int min;		/**< @~chinese 深度最小值		@~english minimum of depth*/ 
	int max;		/**< @~chinese 深度最大值		@~english maximum of depth*/ 
}DepthRange;

/**
* @~chinese
* @brief 网络连接时设备的IP设置，当autoEnable设置为true时，无需设置ipFourthByte
* @~english
* @brief IP setting，when autoEnable is true, there is no need to set ipFourthByte
**/
typedef struct IpSetting
{
	unsigned int autoEnable;	/**< @~chinese 是否开启DHCP		@~english enable/disable DHCP*/ 
	unsigned char ipFourthByte;	/**< @~chinese IP地址的第四位		@~english the fourth byte of ip*/ 
}IpSetting;

/**
* @~chinese
* @brief HDR自动模式时曝光级数及两级曝光之间的倍数设置
* @~english
* @brief exposure times and interstage scale of HDR
**/
typedef struct HdrScaleSetting
{
	unsigned int highReflectModeCount;	/**< @~chinese 高反模式曝光级数		@~english exposure times of high-reflective mode*/ 
	unsigned int highReflectModeScale;	/**< @~chinese 高反模式两级间倍数		@~english interstage scale of high-reflective mode*/ 
	unsigned int lowReflectModeCount;	/**< @~chinese 深色模式曝光级数		@~english exposure times of low-reflective mode*/ 
	unsigned int lowReflectModeScale;	/**< @~chinese 深色模式两级间倍数		@~english interstage scale of low-reflective mode*/ 
}HdrScaleSetting;

#pragma pack(push, 1)

/**
* @~chinese
* @brief HDR某一级曝光的参数
* @~english
* @brief exposure param of HDR
**/
typedef struct HdrExposureParam
{
	unsigned int  exposure;	/**< @~chinese 曝光时间			@~english exposure time*/ 
	unsigned char gain;		/**< @~chinese 增益				@~english gain*/ 
}HdrExposureParam;

/**
* @~chinese
* @brief HDR曝光参数
* @~english
* @brief all exposure params of HDR
**/
typedef struct HdrExposureSetting
{
	unsigned char count;			/**< @~chinese 总曝光级数		@~english total exposure times of HDR*/ 
	HdrExposureParam param[11];		/**< @~chinese 各级曝光参数		@~english all params of HDR*/ 
}HdrExposureSetting;

#pragma pack(pop)


/// \~chinese
/// \defgroup PropertyExtensionType 扩展属性
/// \brief 列举所有可设置的扩展属性
/// @{
/// \~english
/// \defgroup PropertyExtensionType Extensional property
/// \brief List extensional properties
/// @{

/**
* @~chinese
* @brief 枚举: 扩展属性
* @~english
* @brief enumeration: extensional of property
**/
typedef enum PROPERTY_TYPE_EXTENSION
{
	PROPERTY_EXT_IP_SETTING				= 0x4,	 /**< @~chinese IP设置				@~english IP setting*/
	PROPERTY_EXT_DEPTH_RANGE			= 0x707, /**< @~chinese 深度范围				@~english depth range of camera*/
	PROPERTY_EXT_HDR_MODE				= 0x914, /**< @~chinese HDR模式				@~english HDR mode*/
	PROPERTY_EXT_NETWORK_COMPRESS		= 0x5,	 /**< @~chinese 网络传输时流是否压缩
													  @~english whether the stream compresses when transmited by network*/

	PROPERTY_EXT_HDR_SCALE_SETTING		= 0x915, /**< @~chinese HDR自动模式的配置		@~english setting of auto-HDR*/
	PROPERTY_EXT_HDR_EXPOSURE			= 0x916, /**< @~chinese HDR各级参数			@~english all params of HDR*/
	PROPERTY_EXT_AUTO_EXPOSURE_MODE		= 0x912, /**< @~chinese 深度相机自动曝光模式	@~english auto exposure mode of depth camera*/
	PROPERTY_EXT_DEPTH_SCALE			= 0x0,	 /**< @~chinese 深度值缩放系数		@~english depth unit for real distance */
	PROPERTY_EXT_TRIGGER_MODE 			= 0x1,	 /**< @~chinese 触发模式				@~english/PROPERTY_EXT_TRIGGER_OUT_MODE set trigger mode ,normal or trigge mode, value 1 stands for software trigger mode, value 2 stands for hardware trigger mode, other stands for trigger off(default)*/
	PROPERTY_EXT_CONTRAST_MIN			= 0x705, /**< @~chinese 对比度阈值			@~english remove where fringe contrast below this value*/
	PROPERTY_EXT_LED_ON_OFF			    = 0xb00, /**< @~chinese 是否打开LED灯		@~english turn on/off led*/
} PROPERTY_TYPE_EXTENSION;
/// @}

/**
* @~chinese
* @brief 扩展属性值，联合体表示，设置和获取时只取指定属性对应的字段即可
* @~english
* @brief union of extensional property
**/
typedef union PropertyExtension
{
	float depthScale;							/**< @~chinese 对应PROPERTY_EXT_DEPTH_SCALE			@~english corresponding PROPERTY_EXT_DEPTH_SCALE			*/
	TRIGGER_MODE triggerMode;					/**< @~chinese 对应PROPERTY_EXT_TRIGGER_MODE			@~english corresponding PROPERTY_EXT_TRIGGER_MODE			*/
	int algorithmContrast;						/**< @~chinese 对应PROPERTY_EXT_CONTRAST_MIN			@~english corresponding PROPERTY_EXT_CONTRAST_MIN			*/
	AUTO_EXPOSURE_MODE autoExposureMode;		/**< @~chinese 对应PROPERTY_EXT_AUTO_EXPOSURE_MODE	@~english corresponding PROPERTY_EXT_AUTO_EXPOSURE_MODE	*/
	HdrScaleSetting hdrScaleSetting;			/**< @~chinese 对应PROPERTY_EXT_HDR_SCALE_SETTING	@~english corresponding PROPERTY_EXT_HDR_SCALE_SETTING	*/
	HdrExposureSetting hdrExposureSetting;		/**< @~chinese 对应PROPERTY_EXT_HDR_EXPOSURE			@~english corresponding PROPERTY_EXT_HDR_EXPOSURE			*/
	int ledOnOff;								/**< @~chinese 对应PROPERTY_EXT_LED_ON_OFF			@~english corresponding PROPERTY_EXT_LED_ON_OFF			*/
	HDR_MODE hdrMode;							/**< @~chinese 对应PROPERTY_EXT_HDR_MODE				@~english corresponding PROPERTY_EXT_HDR_MODE				*/
	DepthRange depthRange;						/**< @~chinese 对应PROPERTY_EXT_DEPTH_RANGE			@~english corresponding PROPERTY_EXT_DEPTH_RANGE			*/
	IpSetting ipSetting;						/**< @~chinese 对应对应PROPERTY_EXT_IP_SETTING		  @~english corresponding PROPERTY_EXT_IP_SETTING*/
	NETWORK_COMPRESS_MODE networkCompressMode;	/**< @~chinese 对应PROPERTY_EXT_NETWORK_COMPRESS		@~english corresponding PROPERTY_EXT_NETWORK_COMPRESS	*/
	int reserved[15];							/**< @~chinese 预留									@~english reserved */
}PropertyExtension;								

/**
* @~chinese
* @brief 流信息组合，用于打开流时使用，可通过ICamera::getStreamInfos获得
* @~english
* @brief stream information, returned by ICamera::getStreamInfos
**/
typedef struct StreamInfo
{
	STREAM_FORMAT format;	/**< @~chinese 流信息		@~english stream format*/ 
	int width;				/**< @~chinese 宽度			@~english stream width*/
	int height;				/**< @~chinese 高度			@~english stream height*/
	float fps;				/**< @~chinese 帧率			@~english stream framerate*/
}StreamInfo;

/**
* @~chinese
* @brief 相机信息，可通过ICamera::getInfo或ISystem::queryCameras获得
* @~english
* @brief camera informations, returned by ICamera::getStreamInfos or ISystem::queryCameras
**/
typedef struct CameraInfo
{
	char name[32];					/**< @~chinese 相机类型			@~english type of camera*/ 
	char serial[32];				/**< @~chinese 序列号			@~english serial number of camera*/
	char uniqueId[32];				/**< @~chinese 相机标识			@~english unique Id of camera*/
	char firmwareVersion[32];		/**< @~chinese 固件版本			@~english version of firmware*/
	char algorithmVersion[32];		/**< @~chinese 算法版本			@~english version of algorithm*/
}CameraInfo;

/**
* @~chinese
* @brief 相机内参
* @~english
* @brief Intrinsics of depth camera or RGB camera
**/
typedef struct Intrinsics
{
	short width;	/**< @~chinese 标定分辨率-宽度		@~english calibration resolution-width*/ 
	short height;	/**< @~chinese 标定分辨率-高度		@~english calibration resolution-height*/ 
	float fx;
	float zero01;
	float cx;
	float zeor10;
	float fy;
	float cy;
	float zeor20;
	float zero21;
	float one22;
}Intrinsics;

/**
* @~chinese
* @brief 深度相机到RGB相机间的旋转平移信息
* @~english
* @brief Rotation and translation offrom depth camera to RGB camera
**/
typedef struct Extrinsics
{
	float rotation[9];                           /**<@~chinese 3x3旋转矩阵		@~english column-major 3x3 rotation matrix */
	float translation[3];                        /**<@~chinese 3元素的平移矩阵	@~english three-element translation vector */
}Extrinsics;

/**
* @~chinese
* @brief 深度相机或RGB相机畸变参数
* @~english
* @brief Distort of depth camera or RGB camera
**/
typedef struct Distort
{
	float k1;
	float k2;
	float k3;
	float k4;
	float k5;
}Distort;

#ifdef __cplusplus
}
#endif

#endif
