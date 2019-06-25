# PurePursuit_STM32
基于STM32的PurePursuit算法的实现

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
	01-配置初始路径的动态内存分配
	函数原型 bool configureInitPathMemory(void);	
	返回true  初始化成功
	返回false 初始化失败

# 修改历史

