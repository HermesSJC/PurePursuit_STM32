/**
	******************************************************************************
	* File Name          : stracking.h
	* Description        : 这个文件包括了自动驾驶所需的一切定义和函数申明
	******************************************************************************
	*
	* COPYRIGHT(c) 2019-2020 东南大学-仪器科学与工程学院-石佳晨 (QQ:369348508)
	*
	* api说明文档
	*
	* ----------------------------------------------------------------------------
	*
	* 01-配置初始路径的动态内存分配
	* 函数原型 bool configureInitPathMemory(void);	
	* 返回true  初始化成功
	* 返回false 初始化失败
	*
	* ----------------------------------------------------------------------------
	*
	* 02-配置参考路径的动态内存分配
	* 函数原型 bool configureReferencePathMemory(void)
	* 返回true  初始化成功
	* 返回false 初始化失败
	*
	* ----------------------------------------------------------------------------
	*
	* 03-清除参考路径
	* 函数原型 void clearReferencePath(void);
	*
	* ----------------------------------------------------------------------------
	*
	* 04-清除初始路径
	* 函数原型 void clearInitPath(void);		
	*
	* ----------------------------------------------------------------------------
	*
	* 05-添加第一次人工行走的参考点
	* 函数原型 bool addInitPoint(point3d p);
	* 输入参数 [point3d] 东北天坐标系的点
	* 返回true  添加成功
	* 返回false 添加失败
	*
	* ----------------------------------------------------------------------------
	*
	* 06-转换初始航向角
	* 函数原型 float changeCourseAngle(float angle);
	* 输入参数 原始航向角
	* 返回参数 转换以后的航向角
	*
	* ----------------------------------------------------------------------------
	*
	* 07-转换坐标系
	* 函数原型 void JWG2ENU(double j,double w,double g, point3d *p);
	* 输入参数 double类型 依次 经度 维度 高度（度，度，米）
	* 输出参数 转换完毕以后 point类型点的地址 坐标系 东 北 天（米，米，米）
	*
	* ----------------------------------------------------------------------------
	* 
	* 08-自动运行
	* 函数原型 int autoRunPurePursuit(point3d p, float speed, float courseAngle, command *info, trackStatus *status);
	* 输入参数 point类型的东北天坐标系的点（米，米，米），速度（米/秒），转换后的航向角（度）
	* 输出参数 command类型的 info的地址
	*           command::nDirection  |- 方向
	*						|- DIRECTION_MID      中间
	*						|- DIRECTION_LEFT			左转
	*						|- DIRECTION_RIGHT		右转
	*						command::nSwerveAngle |- 期望转向角（0.1度）
	*					 trackStatus类型的 status的地址
	*	 				  trackStatus::fLateral 横向误差
	*           trackStatus::fheadingAngleError 航向误差
	* 返回参数  |------TRACKING_STATUS_NORMAL               正常运行
	*           |------TRACKING_STATUS_NO_ENOUGH_POINTS     没有足够点
	*           |------TRACKING_STATUS_NO_MEMORY						没有内存 gg
	*						|------TRACKING_STATUS_TO_THE_END						走到底
	*
	* ----------------------------------------------------------------------------
	*
	* 09-设置是否需要进行平移路径
	* 函数原型 void setFirstMove(bool status);
  * 输入true  下一个点会成为平移预设路径的参考点 
	*           ++ 警告：会重置之前的预设路径和参考路径和参考点
	* 输入false 下一个点取消成为平移预设路径的参考点 
	*
	* ----------------------------------------------------------------------------
	*
	* 10-设置是否要把下一个点设置为东北天坐标系的原点
	* 函数原型 void setFirstPoint(bool status);
	* 输入true  下一个点会成为东北天坐标系的原点 
	*           ++ 警告：会重置之前的坐标系的原点
	* 输入false 下一个点取消成为东北天坐标系的原点
	*
	* ----------------------------------------------------------------------------
	* 
	* 11-设置采样周期和参考点个数
	* 函数原型 void setSamplingPeriod(uint8_t samplingPeriod);
	* 输入参数 采样周期 单位：赫兹（hz）
	*
	* ----------------------------------------------------------------------------
	* 
	* 12-设置前视距离系数
	* 函数原型 void setSightDistanceFactor(uint8_t factor);
  * 输入参数 前视距离系数 单位：无量纲
	* 
	* ----------------------------------------------------------------------------
	* 
	* 13-设置直线或者线段追踪
	* 函数原型 void setLineArithmetic(bool status);
	* 输入true  直线追踪算法 
	* 输入false 线段追踪算法
	* 
	* ----------------------------------------------------------------------------
	*
	* 14-设置速度影响系数	
	* 函数原型 void setSpeedAffactedFactor(uint8_t factor);
  * 输入参数 速度影响系数	 单位：无量纲
	*
	* ----------------------------------------------------------------------------
	*
	* 15-设置速度固定系数
	* 函数原型 void setSpeedFixedFactor(uint8_t factor);
  * 输入参数 速度固定系数 单位：无量纲
	* ----------------------------------------------------------------------------
	*
	* ###### WARNING ######
	* 请不要调用除了上述API以外的API
	*
  ******************************************************************************
*/

#ifndef __STRACKING_H
#define __STRACKING_H

/* Private Include ------------------------------------------------------------ */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#if (__FPU_PRESENT == 1)			
#include <arm_math.h>			//如果使能了硬件浮点功能，就包括硬件浮点数学库
#else
#include <math.h>					//如果没使能硬件浮点功能，就包括普通数学库
#endif

/* Private Define  ------------------------------------------------------------ */

#define FE_WGS84	0.00335281066474748049		//WGS84偏心率
#define RE_WGS84	6378137.0									//WGS84地球半径
#define e					0.006694379990141316461		//转换常数

#define Degree2Rad	0.01745329251994327813	//角度转弧度
#define Rad2Degree	57.29577951308237971		//弧度转角度

#define CarWheelBearingDistance 3.71f				//轴距

#define MaxSwerveAngle 30.0f								//最大转向角

/* Private Typedef ------------------------------------------------------------ */

typedef struct SPoint3d 			point3d;
typedef struct SInitNode 			initNode;
typedef struct SInitList 			initList;
typedef struct SReferenceList	referenceList;
typedef struct SCommmand			command;
typedef struct SLineInfo			lineInfo;
typedef struct STrackStatus		trackStatus;

/* Private Struct ------------------------------------------------------------- */

/**
* @brief 定义一个三维的点
*/
struct SPoint3d
{
	float x;
	float y;
	float z;
};

/**
* @brief 定义一个双向链表的结点 作为初始化路径的点
*/
struct SInitNode
{
	struct SInitNode *prev;			//前一个结点的地址
	struct SPoint3d point;	//路径点
	struct SInitNode *next;			//后一个结点的地址
};

/**
* @brief 定义一个包含了双向链表信息的结构体 作为初始化路径
*/
struct SInitList
{
	struct SInitNode *head;			//头结点
	struct SInitNode *tail;			//尾结点
	uint16_t nSize;					//长度
};

/**
* @brief 定义一个简单的可变数组的结构体 作为参考路径
*/
struct SReferenceList
{
	struct SPoint3d *point;	//路径点
	uint16_t nSize;					//长度
};

/**
* @brief 定义命令
*/
struct SCommmand
{
	int nDirection;
	int nWheelSteeringAngle;
};

/**
* @brief 定义路径追踪的部分状态
*/
struct STrackStatus
{
	float fLateralError;
	float fHeadingAngleError;
};

/**
* @brief 直线的信息
*/
struct SLineInfo
{
	float A;
	float B;
	float C;
};

/* Private Enum -------------------------------------------------------------- */

/**
* @brief 定义方向
*/
enum DIRECTION_STATUS
{
	DIRECTION_STATUS_MID = 0x00,		//直行
	DIRECTION_STATUS_LEFT,					//左转
	DIRECTION_STATUS_RIGHT					//右转
};

/**
* @brief 定义追踪状态
*/
enum TRACKING_STATUS
{
	TRACKING_STATUS_NORMAL = 0x00,			//正常
	TRACKING_STATUS_NO_ENOUGH_POINTS,
	TRACKING_STATUS_NO_MEMORY,
	TRACKING_STATUS_TO_THE_END
};

/**
* @brief 命令指令状态
*/

enum COMMAND_STATUS
{
	COMMAND_STATUS_NO_COMMAND = 0x00,						//没有指令

	COMMAND_STATUS_IS_RUN,											//强制开始或者强制停止
	COMMAND_STATUS_IS_RECORD,                   //是否记录数据 如果是的话 下一个数据会被认为坐标原点
	COMMAND_STATUS_IS_AUTORUN,                  //是否自动运行 如果是的话 下一个点会被作为平移路径的参考点
	COMMAND_STATUS_IS_INIT,                     //是否初始化路径 如果是的话 开始初始化路径，并覆盖掉之前的坐标点
	COMMAND_STATUS_IS_ADJUST,                   //是否左右矫正
	COMMAND_STATUS_IS_LINE,											//是否直线
	COMMAND_STATUS_IS_TURN,											//是否转向
	COMMAND_STATUS_IS_SLAVE,										//是否下位机计算

	COMMAND_STATUS_SET_SAMPLINGPERIOD,					//设置采样周期
	COMMAND_STATUS_SET_SIGHTDISTANCE_FACTOR,		//设置前视距离系数
	COMMAND_STATUS_SET_KP,                     	//发送PID参数P
  COMMAND_STATUS_SET_KI,                     	//发送PID参数I
  COMMAND_STATUS_SET_KD,                     	//发送PID参数D
	
	COMMAND_STATUS_AUTOSET_MIDANGLE,            //自动设置中位角
	COMMAND_STATUS_AUTOSET_PWMPID,              //自动设置角度跟随的pid值
	
	COMMAND_STATUS_AFFECTSPEED,									//设置速度影响因子
	COMMAND_STATUS_FIXEDSPEED,									//设置速度固定因子

	COMMAND_STATUS_AUTORUN_STATUS               //自动运行的状态
};

/* Private Function ---------------------------------------------------------- */
//基本算数运算

//求两个数的算数平方根
float GetArithmeticSquareRoot(float n1, float n2);
//求第一个点到第二个点的角度
float GetPointAngle(point3d p1, point3d p2);
//通过两点计算直线参数
void CountLinePara(point3d p1, point3d p2, lineInfo *l);
//计算点到直线的距离
float CountPointLineDistance(point3d p, lineInfo l);

//对路径的基本操作

//初始化初始路径的地址分配
bool configureInitPathMemory(void);				
//初始化参考路径的地址分配
bool configureReferencePathMemory(void);		
//添加初始路径
bool addInitPoint(point3d p);
//添加参考路径
void AddReferencePoint(float x, float y, float z);
//清除参考路径
void clearReferencePath(void);					
//清除初始路径
void clearInitPath(void);		

//姿态参数设置

//转换初始航向角
float changeCourseAngle(float angle);
//导航坐标系的转换
void JWG2ENU(double j, double w, double g, point3d *enu);

//生成参考路径
uint8_t GenerateReferencePath(point3d p, float headingAngle);
//purepursuit模型自动追踪
uint8_t autoRunPurePursuit(point3d p, float speed, float headingAngle, command* info, trackStatus* status);

//追踪算法相关参数设置

//设置直线or线段追踪
void setLineArithmetic(bool arithmetic);
//设置是否第一个点 用来初始化坐标原点
void setFirstPoint(bool flag);
//设置是否平移路径
void setFirstMove(bool status);
//设置采样频率
void setSamplingPeriod(uint8_t period);
//设置速度影响系数	
void setSpeedAffactedFactor(uint8_t factor);
//设置速度固定系数
void setSpeedFixedFactor(uint8_t factor);
//设置前视距离系数
void setLookAheadDistanceFactor(uint8_t factor);


#endif	// END OF FILE
