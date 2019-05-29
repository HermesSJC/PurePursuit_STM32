/**
	******************************************************************************
	* File Name          : smpc.c
	* Description        : 这个文件包括了自动驾驶的函数实现
	******************************************************************************
	*
	* COPYRIGHT(c) 2019-2020 东南大学-仪器科学与工程学院-石佳晨 (QQ:369348508)
	*
	* define 需增加 ARM_MATH_CM7 __CC_ARM 和 __FPU_PRESENT=1 才可以使用硬件FPU进行快速计算
	*
	* 跟新日志 -------------------------------------------------------------------
	*
	* 时间：2018-08-30  版本：1.0.0
	* 1-完成了下位机移植的初版
	*
	* 时间：2018-09-20  版本：2.0.0
	* 1-完成了算法的改进，缩短时间
	* 
	* 时间：2018-09-21	版本：2.0.1
	* 1-修复了转换航向角错误的bug
	*
	* 时间：2018-09-25  版本：2.0.2
	* 1-修改了平移路径的算法
	* 2-增加了修改采样周期和采样点数的api
	* 3-增加了设置前视距离系数的api
	*
	* 时间：2018-09-26  版本：2.0.3
	* 1-增加了一些enum的定义
	*
	* 时间：2018-10-15  版本：2.0.4
	* 1-修复了直线追踪算法中的bug
	* 2-优化了平移路径的算法
	* 3-增加了直线追踪算法
	* 4-增加了设置为直线追踪算法的api
	*
	* 时间：2019-01-11  版本：2.0.5
	* 1-增加了设置两个时间影响系数的api
	* 2-优化了纯追踪算法,使其根据速度修改延迟控制的功能 
	*
	* 时间：2019-03-11  版本：3.0.0
	* 1-使用链表重写路径算法
	*
	* 时间：2019-05-14  版本：3.0.1
	* 1-修复了转向方向判断错误的bug
	*
	* 时间：2019-05-16  版本：3.0.2
	* 1-修复了删除链表时候卡死的bug
	*
	* 时间：2019-05-17  版本：3.0.3
	* 1-修复了平移路径角度单位出错导致平移错误的bug
	*
	* 时间：2019-05-22  版本：3.1.0
	* 1-修复了使用线段算法时,平移路径可能会卡死的bug
	* 2-修复了删除链表时,剩下一个点没有删除的bug
	* 3-添加了侧滑预测算法,并兼容之前的算法
  ******************************************************************************
*/

/* Include ------------------------------------------------------------------ */

#include "stracking.h"

#include "usart.h"
#include "main.h"

/* Private Variables ---------------------------------------------------------*/

initList initPath;						//第一次行驶的路径
referenceList referencePath;	//平移后的参考路径

uint16_t nReferencePointIndex = 0;	//参考路径下标的序号

uint8_t nSamplePointNum = 2;		//采样点的个数
float fSamplePeriod = 0.5f;			//采样时间(秒)

float fSpeedAffactedFactor = 0.15f;			//速度影响系数	
float fSpeedFixedFactor = 0.05f;				//速度固定系数

bool isLineArithmeticFlag = false;			//是否直线追踪
bool isFirstPointFlag = false;					//是否第一个点
bool isFirstMoveFlag = false;						//是否平移路径

float fLookAheadDistanceFactor = 4.0f;	//前视距离系数

/* Private Functions ---------------------------------------------------------*/

/**
* @name:				GetArithmeticSquareRoot
* @brief:				计算两个数的算术平方根
* @in:					n1,n2[float] 输入的两个数
* @out:					无
* @retval:			[float]两个数的算术平方根
* @reviseTime:	2018-09-20
*/
float GetArithmeticSquareRoot(float n1, float n2)
{
//如果使能了硬件浮点库
#if (__FPU_PRESENT == 1)
	
	float fSqrtOut = 0.0f;
	float fPowerIn[2] = {0.0f, 0.0f};
	float fPowerOut = 0.0f;
	
	//两个数字赋初值
	fPowerIn[0] = n1;
	fPowerIn[1] = n2;
	
	//求平方和
	arm_power_f32(fPowerIn, 2, &fPowerOut);
	
	//计算成功 返回平方根
	if( arm_sqrt_f32(fPowerOut, &fSqrtOut) == ARM_MATH_SUCCESS)
	{
		return fSqrtOut;
	}
	//计算失败 返回-1
	else
	{
		return -1.0f;
	}
	
//如果没有使用硬件浮点库
#else
	
	return sqrtf(powf(n1,2) + powf(n2,2));
	
#endif
}

/**
* @name:				getPointAngle
* @brief:				计算第一个点到第二个点的的角度
* @in:					p1,p2[point3d] 点的坐标
* @out:					无
* @retval:			[float]角度[-180,180)
* @reviseTime:	2018-08-29
*/
inline float GetPointAngle(point3d p1, point3d p2)
{
	return atan2f(p2.y - p1.y, p2.x - p1.x) * (float)Rad2Degree ;
}

/**
* @name:					countLinePara
* @brief:					通过两点计算直线的Ax+By+C=0的参数
* @in:						p1,p2[point3d] 两个参考点
* @out:						l[lineInfo]	直线的参数
* @retval:				无
* @reviseTime:		2019-04-19
*/
void CountLinePara(point3d p1, point3d p2, lineInfo *l)
{
	
	//如果斜率不存在
	// p1.x == p2.x
	if( -1.0e-5f < p1.x-p2.x && p1.x-p2.x < 1.0e-5f )
	{
		l->A = 1.0f;
		l->B = 0.0f;
		l->C = -p1.x;
	}
	else
	{
		float k = (p2.y - p1.y) / (p2.x - p1.x);
		l->A = k;
		l->B = -1.0f;
		l->C = p2.y - k * p2.x;
	}
}

/**
* @name:				countPointLineDistance
* @brief:				计算点到直线的距离
* @in:					p[point3d] 点的坐标 | l[lineInfo]	直线的参数
* @out:					无
* @retval:			[float]点到直线的距离
* @reviseTime:	2018-08-21
*/
inline float CountPointLineDistance(point3d p, lineInfo l)
{
		return (l.A*p.x + l.B*p.y + l.C) / GetArithmeticSquareRoot(l.A, l.B);
}

/**
* @name:			configureInitPathMemory
* @brief:			为初始路径分配内存地址
* @in:				无
* @out:				无
* @retval:			[bool]是否分配成功
* @reviseTime:		2019-03-11
*/
bool configureInitPathMemory(void)
{
	//申请头尾结点的地址
	initPath.head = malloc(sizeof(initNode));
	initPath.tail = malloc(sizeof(initNode));

	//分配失败报错返回 成功则继续
	if (initPath.head == NULL || initPath.tail == NULL)
	{
		return false;
	}

	//设置头结点 head-> <-tail
	initPath.head->prev = NULL;
	initPath.head->point.x = 0.0f;
	initPath.head->point.y = 0.0f;
	initPath.head->point.z = 0.0f;
	initPath.head->next = initPath.tail;

	//设置尾结点 head-> <-tail
	initPath.tail->prev = initPath.head;
	initPath.tail->point.x = 0.0f;
	initPath.tail->point.y = 0.0f;
	initPath.tail->point.z = 0.0f;
	initPath.tail->next = NULL;

	//设置点的个数
	initPath.nSize = 0;

	return true;
}

/**
* @name:				configureReferencePathMemory
* @brief:				为参考路径分配内存地址
* @in:					无
* @out:					无
* @retval:			[bool]是否分配成功
* @reviseTime:	2019-03-11
*/
bool configureReferencePathMemory(void)
{
	referencePath.point = malloc(sizeof(point3d) * 2);

	nReferencePointIndex = 0;

	if (referencePath.point == NULL)
	{
		return false;
	}

	referencePath.nSize = 0;

	return true;
}

/**
* @name:			addInitPoint
* @brief:			添加初始化路径
* @in:				无
* @out:				无
* @retval:			[bool]是否添加成功
* @reviseTime:		2019-03-11
*/
bool addInitPoint(point3d p)
{
	initNode *temp;										//需要添加的结点
	static initNode *lastPosition;		//记录上次添加的位置

	/*第一个数据 放在首结点*/
	if (initPath.nSize == 0 && initPath.head->next == initPath.tail && initPath.tail->prev == initPath.head)
	{
		//坐标赋值
		initPath.head->point.x = p.x;
		initPath.head->point.y = p.y;
		initPath.head->point.z = p.z;

		//数据成为1个
		initPath.nSize = 1;
	}
	/* 第二个数据 放在尾结点 */
	else if (initPath.nSize == 1 && initPath.head->next == initPath.tail && initPath.tail->prev == initPath.head)
	{
		//坐标赋值
		initPath.tail->point.x = p.x;
		initPath.tail->point.y = p.y;
		initPath.tail->point.z = p.z;

		//数据成为2个
		initPath.nSize = 2;
		
		//上一个位置的指针指向头节点
		lastPosition = initPath.head;
	}
	/* 第三个以及以后的数据 */
	else
	{
		//为数据分配内存 如果分配失败则报错返回 成功则继续
		temp = malloc(sizeof(initNode));
		if (temp == NULL)
		{
			return false;
		}		
		
		//增加一个新的结点
		lastPosition->next = temp;
		
		//尾结点的数据放在倒数第二个结点
		temp->prev = lastPosition;
		temp->point.x = initPath.tail->point.x;
		temp->point.y = initPath.tail->point.y;
		temp->point.z = initPath.tail->point.z;
		temp->next = initPath.tail;
		
		//坐标赋值
		initPath.tail->point.x = p.x;
		initPath.tail->point.y = p.y;
		initPath.tail->point.z = p.z;
		initPath.tail->prev = temp;
		
		//数据数量自增一次
		initPath.nSize++;
		
		//上一个位置的指针指向上一个位置
		lastPosition = temp;
		
	}
	return true;
}

/**
* @name:				AddReferencePoint
* @brief:				添加参考路径
* @in:					无
* @out:					无
* @retval:			无
* @reviseTime:	2019-03-11
*/
void AddReferencePoint(float x, float y, float z)
{
	/* 对应位置添加点的信息 */
	(referencePath.point + referencePath.nSize)->x = x;
	(referencePath.point + referencePath.nSize)->y = y;
	(referencePath.point + referencePath.nSize)->z = z;

	/* 点的个数自增一次 */
	referencePath.nSize++;
}

/**
* @name:				clearReferencePath
* @brief:				清除参考路径
* @in:					无
* @out:					无
* @retval:			无
* @reviseTime:	2019-03-11
*/
void clearReferencePath(void)
{
	free(referencePath.point);
	referencePath.point = NULL;

	nReferencePointIndex = 0;

	referencePath.nSize = 0;
}

/**
* @name:			clearInitPath
* @brief:			清除初始路径
* @in:				无
* @out:				无
* @retval:			[bool]是否添加成功
* @reviseTime:		2019-03-11
*/
void clearInitPath(void)
{
	initNode *head;
	initNode *temp;
	
	head = initPath.head->next;
	temp = head->next;
	
	do
	{
		head->prev = NULL;
		head->next = NULL;
		free(head);
		head = temp;
		temp = head->next;
	}while(temp->next != initPath.tail);

	//设置头结点
	initPath.head->prev = NULL;
	initPath.head->point.x = 0.0f;
	initPath.head->point.y = 0.0f;
	initPath.head->point.z = 0.0f;
	initPath.head->next = initPath.tail;

	//设置尾结点
	initPath.tail->prev = initPath.head;
	initPath.tail->point.x = 0.0f;
	initPath.tail->point.y = 0.0f;
	initPath.tail->point.z = 0.0f;
	initPath.tail->next = NULL;

	//设置点的个数
	initPath.nSize = 0;
}

/**
* @name:				debugPrintInitPath
* @brief:				输出初始路径
* @in:					无
* @out:					无
* @retval:			无
* @reviseTime:	2018-04-22
*/
void debugPrintInitPath(void)
{
	initNode * head;
	head = initPath.head;
	
	uint16_t i = 1;
	printf("init path:\r\n");
	printf("from head to tail:\r\n");
	do
	{	
		printf("point %3d is (%.4f, %.4f, %.4f)\r\n",i, head->point.x, head->point.y, head->point.z);
		i++;
		head = head->next;
		
	}while(head->next != NULL);
	
	initNode *tail;
	tail = initPath.tail;
	
	i = 1;
	printf("\r\nfrom tail to head:\r\n");
	do
	{
		printf("point %3d is (%.4f, %.4f, %.4f)\r\n",i, tail->point.x, tail->point.y, tail->point.z);
		i++;
		tail = tail->prev;
		
	}while(tail->prev != NULL);
}

/**
* @name:				debugPrintInitPath
* @brief:				输出参考路径
* @in:					无
* @out:					无
* @retval:			无
* @reviseTime:	2018-04-22
*/
void debugPrintReferencePath(void)
{
	for(uint16_t i = 0; i < referencePath.nSize; i++)
	{
		printf("point %3d is (%.4f, %.4f, %.4f)\r\n",i, (referencePath.point + i)->x, (referencePath.point + i)->y, (referencePath.point + i)->z );
	}
}


/**
* @name:				changeCourseAngle
* @brief:				转换航向角
* @in:					angle[float] 转换前航向角
* @out:					无
* @retval:			[float] 转换后航向角[0,360)
* @reviseTime:	2018-09-21
*/
__inline float changeCourseAngle(float angle)
{
	/* 角度转换 */
	angle = 450.0f - angle;
	
	/* 角度转换到[0,360)之间 */
	angle = (angle >= 360.0f) ? angle - 360.0f : ( angle < 0.0f ) ? angle + 360.0f : angle ;
	
	return angle;
}


/**
* @name:				JWG2ENU
* @brief:				经纬高坐标系转换到东北天坐标系
* @in:					j,w,g[double] 经纬高坐标
* @out:					p[point] 东北天坐标
* @retval:			无
* @reviseTime:	2018-09-20
*/
void JWG2ENU(double j, double w, double g, point3d *p)
{
	//坐标转换的系数矩阵
	static double dTransforMatrix[3][3] = {{0.0}};
	//ecef0的坐标系的数值
	static double ecefx0 = 0.0, ecefy0 = 0.0, ecefz0 = 0.0;

	//角度的系数
	double sinp = 0, cosp = 0.0, sinl = 0.0, cosl = 0.0;
	//坐标转换系数
	double v = 0.0;
	//ecef的坐标系的数值
	double ecefx = 0.0, ecefy = 0.0, ecefz = 0.0;

	//角度转弧度
	j = j * Degree2Rad;
	w = w * Degree2Rad;

	//带入sin和cos
	sinp = sin(w);
	cosp = cos(w);
	sinl = sin(j);
	cosl = cos(j);

	//设置第一个点的参数
	if (isFirstPointFlag == true)
	{
		//只设置一次
		isFirstPointFlag = false;

		//设置转换矩阵的系数
		dTransforMatrix[0][0] = -sinl;					dTransforMatrix[0][1] = cosl;						dTransforMatrix[0][2] = 0.0;
		dTransforMatrix[1][0] = -sinp * cosl;   dTransforMatrix[1][1] = -sinp * sinl;   dTransforMatrix[1][2] = cosp;
		dTransforMatrix[2][0] = cosp * cosl;    dTransforMatrix[2][1] = cosp * sinl;    dTransforMatrix[2][2] = sinp;

		//计算ecef0的值 单位从米转换为米 类型从double转doubel
		v = RE_WGS84 / sqrt(1.0 - e * sinp*sinp);
		ecefx0 = (v + g)*cosp*cosl;
		ecefy0 = (v + g)*cosp*sinl;
		ecefz0 = (v*(1.0 - e) + g)*sinp;

		//第一个坐标的点肯定是(0,0,0)
		p->x = 0.0f;
		p->y = 0.0f;
		p->z = 0.0f;
	}
	else
	{
		//jwg转ecef 单位从米转换为米 类型从double转doubel
		v = RE_WGS84 / sqrt(1.0 - e * sinp*sinp);
		ecefx = (v + g)*cosp*cosl - ecefx0;
		ecefy = (v + g)*cosp*sinl - ecefy0;
		ecefz = (v*(1.0 - e) + g)*sinp - ecefz0;

		//ecef转enu 单位从米转换为米，类型从double转float
		p->x = (float)(dTransforMatrix[0][0] * ecefx + dTransforMatrix[0][1] * ecefy + dTransforMatrix[0][2] * ecefz);
		p->y = (float)(dTransforMatrix[1][0] * ecefx + dTransforMatrix[1][1] * ecefy + dTransforMatrix[1][2] * ecefz);
		p->z = (float)(dTransforMatrix[2][0] * ecefx + dTransforMatrix[2][1] * ecefy + dTransforMatrix[2][2] * ecefz);
	}
}

/**
* @name:				setSamplingPeriod
* @brief:				设置采样周期和采样点的个数
* @in:					period[uint8_t] 采样周期（单位：hz）
* @out:					无
* @retval:			无
* @reviseTime:	2018-09-25
*/
__inline void setSamplingPeriod(uint8_t period)
{
	nSamplePointNum = period;
	fSamplePeriod = (float) (1.0f / period);
}

/**
* @name:				setLineArithmetic
* @brief:				设置是否使用直线追踪算法
* @in:					status[bool] 是否设置
* @out:					无
* @retval:			无
* @reviseTime:	2018-10-15
*/
void setLineArithmetic(bool status)
{
	isLineArithmeticFlag = status;
}

/**
* @name:				setFirstPoint
* @brief:				设置是否第一个点的函数
* @in:					status[bool] 是否设置
* @out:					无
* @retval:			无
* @reviseTime:	2018-09-20
*/
__inline void setFirstPoint(bool status)
{
	isFirstPointFlag = status;
}

/**
* @name:					setFirstMove
* @brief:					设置是否第一次平移路径的函数
* @in:						status[bool] 是否设置
* @out:						无
* @retval:				无
* @reviseTime:		2018-09-20
*/
__inline void setFirstMove(bool status)
{
	isFirstMoveFlag = status;
}

/**
* @name:				setSpeedAffactedFactor
* @brief:				设置速度影响系数
* @in:					[uint8_t] 影响系数
* @out:					无
* @retval:			无
* @reviseTime:	2019-04-19
*/
__inline void setSpeedAffactedFactor(uint8_t factor)
{
	fSpeedAffactedFactor = (float) (factor / 100.0f);
}

/**
* @name:				setSpeedFixedFactor
* @brief:				设置速度固定系数
* @in:					[uint8_t] 影响系数
* @out:					无
* @retval:			无
* @reviseTime:	2019-04-19
*/
__inline void setSpeedFixedFactor(uint8_t factor)
{
	fSpeedFixedFactor = (float) (factor / 100.0f);
}

/**
* @name:					setLookAheadDistanceFactor
* @brief:					设置前视距离系数
* @in:						factor[uint8_t] 前视距离系数
* @out:						无
* @retval:				无
* @reviseTime:		2018-09-25
*/
__inline void setLookAheadDistanceFactor(uint8_t factor)
{
	fLookAheadDistanceFactor = (float) ( factor / 10.0f );
}

/**
* @name:			generateReferencePath
* @brief:			生成参考路径
* @in:				无
* @out:				无
* @retval:			[bool]是否生成成功
* @reviseTime:		2019-03-12
*/
uint8_t GenerateReferencePath(point3d p, float headingAngle)
{
	float x00 = 0.0f, y00 = 0.0f, z00 = 0.0f;
	float fHeadingAngleError = 0.0f;
	float fReferenceHeadingAngle = 0.0;

	// 0 | 判断有没有足够的点
	//线段
	if (isLineArithmeticFlag == false)
	{
		if (initPath.nSize < 2 * nSamplePointNum)
		{
			return TRACKING_STATUS_NO_ENOUGH_POINTS;
		}
	}
	//直线
	else
	{
		if (initPath.nSize < 2)
		{
			return TRACKING_STATUS_NO_ENOUGH_POINTS;
		}
	}

	// 1 | 清除以前的信息
	clearReferencePath();

	// 2 | 根据初始路径重新分配地址
	//根据追踪方法分配空间大小
	uint16_t nReferencePointNum = (isLineArithmeticFlag == false )? initPath.nSize/nSamplePointNum : 2 ;
	//根据个数分配内存 如果分配失败则报错 成功则继续
	referencePath.point = malloc(nReferencePointNum * sizeof(point3d));
	if (referencePath.point == NULL)
	{
		return TRACKING_STATUS_NO_MEMORY;
	}

	// 3 | 计算角度之间的差距
	//参考角度 (-180, 180]
	fReferenceHeadingAngle = atan2f(initPath.tail->point.y - initPath.head->point.y, initPath.tail->point.x - initPath.head->point.x)*(float)Rad2Degree;
	//航向角 (0, 360]
	fHeadingAngleError = headingAngle - fReferenceHeadingAngle;
	//航向误差 转换到(-180, 180]
	fHeadingAngleError = (fHeadingAngleError >= 180.0f) ? fHeadingAngleError - 360.0f : ((fHeadingAngleError < -180.0f) ? fHeadingAngleError + 360.0f : fHeadingAngleError);

	// 4 | 平移路径
	//线段
	if (isLineArithmeticFlag == false)
	{
		uint16_t i = 0;
		//航向角差距不大 按照第一个平移
		if ( -90.0f <= fHeadingAngleError && fHeadingAngleError < 90.0f)
		{
			//获取第一个点的位置
			initNode * head = initPath.head;
			x00 = p.x - head->point.x;
			y00 = p.y - head->point.y;
			z00 = p.z - head->point.z;
			
			do
			{
				//每隔nSamplePointNum 取点
				if (i % nSamplePointNum == 0)
				{
					AddReferencePoint(head->point.x + x00, head->point.y + y00, head->point.z + z00);
				}
				//到下一个点
				head = head->next;
			}while(head->next != NULL);
		}
		//差距比较大则按照最后一个平移
		else
		{
			//获取最后一个点的位置
			initNode * tail = initPath.tail;
			x00 = p.x - tail->point.x;
			y00 = p.y - tail->point.y;
			z00 = p.z - tail->point.z;
	
			do
			{
				//每隔nSamplePointNum 取点
				if (i % nSamplePointNum == 0)
				{
					AddReferencePoint(tail->point.x + x00, tail->point.y + y00, tail->point.z + z00);
				}
				//到前一个点
				tail = tail->prev;
			}while(tail->prev != NULL);
		}
	}
	//直线
	else
	{
		//航向角差距不大 按照第一个平移
		if (-90.0f <= fHeadingAngleError && fHeadingAngleError < 90.0f)
		{
			x00 = p.x - initPath.head->point.x;
			y00 = p.y - initPath.head->point.y;
			z00 = p.z - initPath.head->point.z;

			AddReferencePoint(initPath.head->point.x + x00, initPath.head->point.y + y00, initPath.head->point.z + z00);
			AddReferencePoint(initPath.tail->point.x + x00, initPath.tail->point.y + y00, initPath.tail->point.z + z00);
		}
		//差距比较大则按照最后一个平移
		else
		{
			x00 = p.x - initPath.tail->point.x;
			y00 = p.y - initPath.tail->point.y;
			z00 = p.z - initPath.tail->point.z;

			AddReferencePoint(initPath.tail->point.x + x00, initPath.tail->point.y + y00, initPath.tail->point.z + z00);
			AddReferencePoint(initPath.head->point.x + x00, initPath.head->point.y + y00, initPath.head->point.z + z00);
		}
	}

	return TRACKING_STATUS_NORMAL;
}

/**
* @name:				autoRunPurePursuit
* @brief:				模型预测控制追踪路径
* @in:					当前点p[point3d] | 速度speed[float] | 航向角headingAngle[float] | 侧滑角slipingAngle[float] | 当前转向弧度presentWheelSteeringRadian[float] 
* @out:					控制命令[command] | 追踪状态[trackStatus]
* @retval:			追踪情况[uint8_t]
* @reviseTime:	2019-05-29
*/
uint8_t autoRunPurePursuit(point3d p, float speed, float headingAngle, float slipingAngle, float presentWheelSteeringRadian, command* info, trackStatus* status)
{
	//定义转向半径  ##在k时刻需要k-1时刻的转向半径
	static float fTurningRadius;
#ifdef SLIP_ENABLE
	//定义上一个时刻的航向角
	static float fPrevTimeHeadingAngle;
	//定义上一个时刻的x坐标和y坐标
	static float fPrevTimeX, fPrevTimeY;
#endif
	
	
	/*  0 | 判断是否移动路径 */
	if(isFirstMoveFlag == true)
	{
		isFirstMoveFlag = false;
		uint8_t res = GenerateReferencePath(p, headingAngle);
		if(res != TRACKING_STATUS_NORMAL)
		{
			status->fHeadingAngleError = 0.0f;
			status->fLateralError = 0.0f;
			info->nDirection = DIRECTION_STATUS_MID;
			info->nWheelSteeringAngle = 0;
			return res;
		}
		//k-1时刻的转向半径定义为很大 当作直线行驶
		fTurningRadius = 1.0e5f;
#ifdef SLIP_ENABLE
		//用当前时刻的点当作k-1时刻的点 否则第一次一定会被判定产生侧滑
		fPrevTimeX = p.x;
		fPrevTimeY = p.y;
#endif
	}
	
	/*  1 | 确定当前要追踪的两个点的数据 以及是否到底 */
	//定义当前追踪路径的两个端点
	point3d prevPoint, nextPoint;
	//直线追踪
	if(isLineArithmeticFlag == true)
	{
		//直线追踪只需要2个点
		//第一个点
		prevPoint.x = (referencePath.point + 0)->x;
		prevPoint.y = (referencePath.point + 0)->y;
		prevPoint.z = (referencePath.point + 0)->z;
		
		//第二个点
		nextPoint.x = (referencePath.point + (referencePath.nSize - 1))->x;
		nextPoint.y = (referencePath.point + (referencePath.nSize - 1))->y;
		nextPoint.z = (referencePath.point + (referencePath.nSize - 1))->z;
		
		//计算参考路线的长度
		float fReferenceLineLength = GetArithmeticSquareRoot(prevPoint.x - nextPoint.x, prevPoint.y - nextPoint.y);
		//计算和前一个端点的距离
		float fPrevDistance = GetArithmeticSquareRoot(p.x - prevPoint.x, p.y - prevPoint.y);
		
		//利用三角形的边长定律 当最长边被切换时 说明直线追踪到底了 返回结束
		if (fPrevDistance >= fReferenceLineLength)
		{
			status->fHeadingAngleError = 0.0f;
			status->fLateralError = 0.0f;
			info->nDirection = DIRECTION_STATUS_MID;
			info->nWheelSteeringAngle = 0;
			return TRACKING_STATUS_TO_THE_END;
		}
	}
	//线段追踪
	else
	{
		//下标没越界 说明还没到底 提取两个端点的值
		if (nReferencePointIndex + 1 <= referencePath.nSize - 1)
		{
			//前一个点
			prevPoint.x = (referencePath.point + nReferencePointIndex)->x;
			prevPoint.y = (referencePath.point + nReferencePointIndex)->y;
			prevPoint.z = (referencePath.point + nReferencePointIndex)->z;
			//到前一个点的距离
			float fPresDistance = GetArithmeticSquareRoot(p.x - prevPoint.x, p.y - prevPoint.y);

			//后一个点
			nextPoint.x = (referencePath.point + (nReferencePointIndex + 1))->x;
			nextPoint.y = (referencePath.point + (nReferencePointIndex + 1))->y;
			nextPoint.z = (referencePath.point + (nReferencePointIndex + 1))->z;
			//到后一个点的距离
			float fNextDistance = GetArithmeticSquareRoot(p.x - nextPoint.x, p.y - nextPoint.y);
			
			//如果离当前点的距离大于下个点，切换参考点
			if (fPresDistance > fNextDistance)
			{
				nReferencePointIndex++;
				
				//如果切换了越界,就说明倒底部了
				if (nReferencePointIndex + 1 == referencePath.nSize)
				{
					status->fHeadingAngleError = 0.0f;
					status->fLateralError = 0.0f;
					info->nDirection = DIRECTION_STATUS_MID;
					info->nWheelSteeringAngle = 0;
					return TRACKING_STATUS_TO_THE_END;
				}
				//没有的话 重新定位点
				prevPoint.x = (referencePath.point + nReferencePointIndex)->x;
				prevPoint.y = (referencePath.point + nReferencePointIndex)->y;
				prevPoint.z = (referencePath.point + nReferencePointIndex)->z;

				nextPoint.x = (referencePath.point + (nReferencePointIndex + 1))->x;
				nextPoint.y = (referencePath.point + (nReferencePointIndex + 1))->y;
				nextPoint.z = (referencePath.point + (nReferencePointIndex + 1))->z;				
			}
		}
	}
	
//如果硬件FPU可以使用 则使用硬件fpu计算一些信息
#if( __FPU_PRESENT == 1)
	float fSinValue, fCosValue;
	float fSqrtOut;
#endif
	
	/*  3 | 改进型纯追踪算法计算期望转向角信息 */
	/** -------------------------------- 滞后性预测及调整 -------------------------------- **/
#ifdef CONTROLDELAY_ENABLE
	// 根据速度计算延迟时间
	float fDelayTime = fSpeedFixedFactor + fSpeedAffactedFactor * speed;
	// 计算在延迟时间内的航向角的变化 这里是k-1时刻的转向半径
	float fDelayTimeChangeHeadingRadian = speed * fDelayTime / fTurningRadius;
#if ( __FPU_PRESENT==1 )
	//计算车体坐标系的值
	float fBodyX = fTurningRadius * (1 - arm_cos_f32(fDelayTimeChangeHeadingRadian));
	float fBodyY = fTurningRadius * arm_sin_f32(fDelayTimeChangeHeadingRadian);
	//计算车体坐标系下改变的值
	arm_sin_cos_f32(headingAngle, &fSinValue, &fCosValue);
	float fDelayTimeChangeX = fBodyX * fSinValue + fBodyY * fCosValue;
	float fDelayTimeChangeY = fBodyY * fSinValue - fBodyX * fCosValue;
#else
	//计算车体坐标系的值
	fBodyX = fTurningRadius * (1 - cosf(fDelayTimeChangeHeadingRadian));
	fBodyY = fTurningRadius * sinf(fDelayTimeChangeHeadingRadian);
	//计算车体坐标系下改变的值
	float fHeadingRadian = headingAngle * (float)Degree2Rad;
	float sinH = sinf(fHeadingRadian), cosh = cosf(fHeadingRadian);
	float fDelayTimeChangeX = fBodyX * sinH + fBodyY * cosH;
	float fDelayTimeChangeY = fBodyY * sinH - fBodyX * cosH;
#endif
	
#endif

	/** -------------------------------- 侧滑的预测及调整 -------------------------------- **/
#ifdef SLIP_ENABLE
	//预测的理论位置
#if ( __FPU_PRESENT==1 )
	arm_sin_cos_f32(fPrevTimeHeadingAngle, &fSinValue, &fCosValue);
	float fPredictedX = fPrevTimeX + speed*fSamplePeriod*fCosValue;
	float fPredictedY = fPrevTimeY + speed*fSamplePeriod*fSinValue;
#else
	float fPrevTimeHeadingRadian = fPrevTimeHeadingAngle*(float)Degree2Rad;
	float sinH = sinf(fPrevTimeHeadingRadian), cosH = cosf(fPrevTimeHeadingRadian)
	float fPredictedX = fPrevTimeX + speed*fSamplePeriod*cosH;
	float fPredictedY = fPrevTimeY + speed*fSamplePeriod*sinH;
#endif
	//期望等效侧滑角
	float fEquivalentSlipingAngle = 0.0f;
	//如果距离比较大 判定产生了侧滑 计算等效侧滑角 否则就还是0
	if(GetArithmeticSquareRoot(p.x - fPredictedX, p.y - fPredictedY) > 0.1f)
	{
		//预测的期望航向角
		float fPredictedHeadingAngle = fPrevTimeHeadingAngle + (speed*fSamplePeriod/fTurningRadius)*(float)(Rad2Degree);
		//后轮侧滑角
		float fRearWheelSlipingRadian = (headingAngle - slipingAngle)*(float)Degree2Rad;
		//前轮侧滑角
		float fFrontWheelSlipingRadian = (headingAngle - fPredictedHeadingAngle)*(float)Degree2Rad;
		//计算等效侧滑角
		fEquivalentSlipingAngle = (1.0f/(1.0f/presentWheelSteeringRadian + 1.0f/(presentWheelSteeringRadian+fRearWheelSlipingRadian) + 1.0f/fFrontWheelSlipingRadian) - presentWheelSteeringRadian)*(float)(Rad2Degree);
	}
	//跟新上一个时刻位置 和航向角
	fPrevTimeX = p.x;
	fPrevTimeY = p.y;
	fPrevTimeHeadingAngle = headingAngle;
#endif

	/** ----------------------------------- 纯追踪算法 ----------------------------------- **/
	//计算参考路径角度 计算瞄准的点的角度
	float fReferenceLineAngle = GetPointAngle(prevPoint, nextPoint);
	float fAimPointAngle = GetPointAngle(p, nextPoint);
	
	//计算航向角误差
	headingAngle = (headingAngle > 180.0f) ? headingAngle - 360.0f : ((headingAngle <= -180.0f) ? headingAngle + 360.0f : headingAngle);
	float fHeadingAngleError = fReferenceLineAngle - headingAngle;
	fHeadingAngleError = (fHeadingAngleError > 180.0f) ? fHeadingAngleError - 360.0f : ((fHeadingAngleError <= -180.0f) ? fHeadingAngleError + 360.0f : fHeadingAngleError);
	status->fHeadingAngleError = fHeadingAngleError;
	
	//计算横向误差
	//确定参考参考线段的信息
	lineInfo referenceLine;
	CountLinePara(prevPoint, nextPoint, &referenceLine);
	//计算点到直线的距离
	status->fLateralError = CountPointLineDistance(p, referenceLine);
	
#ifdef CONTROLDELAY_ENABLE
	// 预测未来时刻的位置
	p.x = p.x + fDelayTimeChangeX;
	p.y = p.y + fDelayTimeChangeY;
	//计算新的点到直线的距离
	float fLateralError = CountPointLineDistance(p, referenceLine);
#else
	// 确定横向误差的数值
	float fLateralError = status->fLateralError;
#endif
	//转换默认航向角到[0-360)
	fReferenceLineAngle = (fReferenceLineAngle < 0.0f) ? fReferenceLineAngle + 360.0f : fReferenceLineAngle;
	//确定横向误差的符号
	if (90.0f < fReferenceLineAngle  && fReferenceLineAngle <= 270.0f)
	{
		fLateralError = -fLateralError;
		status->fLateralError = -status->fHeadingAngleError;
	}
	
	//计算前视距离
	//float fLookAheadDistance = fLookAheadDistanceFactor * speed;
	float fLookAheadDistance = 4.0f;
	
	//计算转向角度
	//期望转向角
	float fWheelSteeringAngle;
	//当偏离的太厉害的时候
	if (fabsf(fLateralError) > fLookAheadDistance)
	{
		//计算k时刻的转弯半径
		fTurningRadius = CarWheelBearingDistance / tanf(MaxSwerveAngle*(float)Degree2Rad);
		// 计算转向角度
		if (fabsf(fAimPointAngle - headingAngle) < 180.0f)
		{
			fWheelSteeringAngle = (fAimPointAngle < headingAngle) ? -MaxSwerveAngle : MaxSwerveAngle;
		}
		else
		{
			fWheelSteeringAngle = (fAimPointAngle < headingAngle) ? MaxSwerveAngle : -MaxSwerveAngle;
		}
	}
	//偏转的不是很厉害
	else
	{
#if ( __FPU_PRESENT==1 )
		//计算前视距离和横向误差的平方差的根
		arm_sqrt_f32(fLookAheadDistance*fLookAheadDistance - fLateralError * fLateralError, &fSqrtOut);
		//计算误差角的sin和cos值
		arm_sin_cos_f32(fHeadingAngleError, &fSinValue, &fCosValue);
		//计算k时刻的转向半径
		fTurningRadius = 0.5f * (fLookAheadDistance*fLookAheadDistance) / (fLateralError*fCosValue + fSqrtOut*fSinValue);
#else
		fTurningRadius = 0.5f * (fLookAheadDistance*fLookAheadDistance) / (fLateralError*cosf(fHeadingAngleError*(float)Degree2Rad) + sqrtf(fLookAheadDistance*fLookAheadDistance - fLateralError*fLateralError)*sinf(HeadingAngleError(float)Degree2Rad));
#endif
		
		//计算转向角度
#ifdef SLIP_ENABLE	
		fWheelSteeringAngle = atanf(CarWheelBearingDistance/fTurningRadius) * (float)Rad2Degree + fEquivalentSlipingAngle;
#else
		fWheelSteeringAngle = atanf(CarWheelBearingDistance/fTurningRadius) * (float)Rad2Degree;
#endif
		//如果太大就纠正回去
		fWheelSteeringAngle = (fWheelSteeringAngle > MaxSwerveAngle) ? MaxSwerveAngle : ((fWheelSteeringAngle < -MaxSwerveAngle) ? -MaxSwerveAngle : fWheelSteeringAngle);
	}
	
	/*  4 | 将信息转化为控制信息并输出 */
	info->nDirection = (fabsf(fWheelSteeringAngle) < 2e-2f ) ? DIRECTION_STATUS_MID : (fWheelSteeringAngle > 0) ? DIRECTION_STATUS_LEFT : DIRECTION_STATUS_RIGHT;
	info->nWheelSteeringAngle = (uint16_t)(fabsf(fWheelSteeringAngle *10.0f));
	
	return TRACKING_STATUS_NORMAL;
}

/* END OF FILE */
