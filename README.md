# PurePursuit_STM32
基于STM32的PurePursuit算法的实现

# 开发环境
windows:x64 东南大学教育版<br>
mdk:5.26.2<br>
mdk-package:stm32f7-2.11.0<br>
java:1.8.0_191<br>
cubemx:4.27.0<br>
cubemx-package:hal_stm32f7_1.12.0<br>

# 命令指南
COMMAND_STATUS<br>
{<br>
	COMMAND_STATUS_NO_COMMAND = 0x00,		//没有指令<br><br>
	COMMAND_STATUS_IS_RUN,				/强制开始或者强制停止<br>
	COMMAND_STATUS_IS_RECORD,                   	//是否记录数据 如果是的话 下一个数据会被认为坐标原点<br>
	COMMAND_STATUS_IS_AUTORUN,                  	//是否自动运行 如果是的话 下一个点会被作为平移路径的参考点<br>
	COMMAND_STATUS_IS_INIT,                     	//是否初始化路径 如果是的话 开始初始化路径，并覆盖掉之前的坐标点<br>
	COMMAND_STATUS_IS_ADJUST,                   	//是否左右矫正<br>
	COMMAND_STATUS_IS_LINE,				//是否直线<br>
	COMMAND_STATUS_IS_TURN,				//是否转向<br>
	COMMAND_STATUS_IS_SLAVE,			//是否下位机计算<br><br>
	COMMAND_STATUS_SET_SAMPLINGPERIOD,		//设置采样周期<br>
	COMMAND_STATUS_SET_SIGHTDISTANCE_FACTOR,	//设置前视距离系数<br>
	COMMAND_STATUS_SET_KP,                     	//发送PID参数P<br>
  	COMMAND_STATUS_SET_KI,                     	//发送PID参数I<br>
  	COMMAND_STATUS_SET_KD,                     	//发送PID参数D<br><br>
	COMMAND_STATUS_AUTOSET_MIDANGLE,            	//自动设置中位角<br>
	COMMAND_STATUS_AUTOSET_PWMPID,              	//自动设置角度跟随的pid值<br><br>
	COMMAND_STATUS_AFFECTSPEED,			//设置速度影响因子<br>
	COMMAND_STATUS_FIXEDSPEED,			//设置速度固定因子<br>
	COMMAND_STATUS_AUTORUN_STATUS,              	//自动运行的状态<br><br>
	COMMAND_STATUS_ERROR              		//出现错误<br>
};


# api说明
## 01-配置初始路径的动态内存分配
函数原型 bool configureInitPathMemory(void);<br>
返回true  初始化成功<br>
返回false 初始化失败<br><br>

## 02-配置参考路径的动态内存分配
函数原型 bool configureReferencePathMemory(void);<br>
返回true  初始化成功<br>
返回false 初始化失败<br>

## 03-清除参考路径
函数原型 void clearReferencePath(void);<br>

## 04-清除初始路径
函数原型 void clearInitPath(void);<br>

## 05-添加第一次人工行走的参考点
函数原型 bool addInitPoint(point3d p);<br>
输入参数 [point3d] 东北天坐标系的点<br>
返回true  添加成功<br>
返回false 添加失败<br>

# 修改历史
## 时间：2018-08-30  版本：1.0.0
1-完成了下位机移植的初版<br>

## 时间：2018-09-20  版本：2.0.0
1-完成了算法的改进，缩短时间<br>

## 时间：2018-09-21  版本：2.0.1
1-修复了转换航向角错误的bug<br>

## 时间：2018-09-25  版本：2.0.2
1-修改了平移路径的算法<br>
2-增加了修改采样周期和采样点数的api<br>
3-增加了设置前视距离系数的api<br>

## 时间：2018-09-26  版本：2.0.3
1-增加了一些enum的定义<br>

## 时间：2018-10-15  版本：2.0.4
1-修复了直线追踪算法中的bug<br>
2-优化了平移路径的算法<br>
3-增加了直线追踪算法<br>
4-增加了设置为直线追踪算法的api<br>

## 时间：2019-01-11  版本：2.0.5
1-增加了设置两个时间影响系数的api<br>
2-优化了纯追踪算法,使其根据速度修改延迟控制的功能<br>

## 时间：2019-03-11  版本：3.0.0
1-使用链表重写路径算法<br>

## 时间：2019-06-04  版本：3.0.1
1-修复了链表清空可能会死机的bug<br>

## 时间：2019-06-14  版本：3.0.2
1-增加了离线初始化的功能<br>
2-优化了链表清空的算法<br>

## 时间：2019-06-15  版本：3.0.3
1-修复了离线初始化以后 再在线初始化 数据没有清空的bug<br>
