/**
	******************************************************************************
	* File Name          : smpc.c
	* Description        : ����ļ��������Զ���ʻ�ĺ���ʵ��
	******************************************************************************
	*
	* COPYRIGHT(c) 2019-2020 ���ϴ�ѧ-������ѧ�빤��ѧԺ-ʯ�ѳ� (QQ:369348508)
	*
	* define ������ ARM_MATH_CM7 __CC_ARM �� __FPU_PRESENT=1 �ſ���ʹ��Ӳ��FPU���п��ټ���
	*
	* ������־ -------------------------------------------------------------------
	*
	* ʱ�䣺2018-08-30  �汾��1.0.0
	* 1-�������λ����ֲ�ĳ���
	*
	* ʱ�䣺2018-09-20  �汾��2.0.0
	* 1-������㷨�ĸĽ�������ʱ��
	* 
	* ʱ�䣺2018-09-21	�汾��2.0.1
	* 1-�޸���ת������Ǵ����bug
	*
	* ʱ�䣺2018-09-25  �汾��2.0.2
	* 1-�޸���ƽ��·�����㷨
	* 2-�������޸Ĳ������ںͲ���������api
	* 3-����������ǰ�Ӿ���ϵ����api
	*
	* ʱ�䣺2018-09-26  �汾��2.0.3
	* 1-������һЩenum�Ķ���
	*
	* ʱ�䣺2018-10-15  �汾��2.0.4
	* 1-�޸���ֱ��׷���㷨�е�bug
	* 2-�Ż���ƽ��·�����㷨
	* 3-������ֱ��׷���㷨
	* 4-����������Ϊֱ��׷���㷨��api
	*
	* ʱ�䣺2019-01-11  �汾��2.0.5
	* 1-��������������ʱ��Ӱ��ϵ����api
	* 2-�Ż��˴�׷���㷨,ʹ������ٶ��޸��ӳٿ��ƵĹ��� 
	*
	* ʱ�䣺2019-03-11  �汾��3.0.0
	* 1-ʹ��������д·���㷨
	*
	* ʱ�䣺2019-05-14  �汾��3.0.1
	* 1-�޸���ת�����жϴ����bug
	*
	* ʱ�䣺2019-05-16  �汾��3.0.2
	* 1-�޸���ɾ������ʱ������bug
	*
	* ʱ�䣺2019-05-17  �汾��3.0.3
	* 1-�޸���ƽ��·���Ƕȵ�λ������ƽ�ƴ����bug
	*
	* ʱ�䣺2019-05-22  �汾��3.1.0
	* 1-�޸���ʹ���߶��㷨ʱ,ƽ��·�����ܻῨ����bug
	* 2-�޸���ɾ������ʱ,ʣ��һ����û��ɾ����bug
	* 3-����˲໬Ԥ���㷨,������֮ǰ���㷨
  ******************************************************************************
*/

/* Include ------------------------------------------------------------------ */

#include "stracking.h"

#include "usart.h"
#include "main.h"

/* Private Variables ---------------------------------------------------------*/

initList initPath;						//��һ����ʻ��·��
referenceList referencePath;	//ƽ�ƺ�Ĳο�·��

uint16_t nReferencePointIndex = 0;	//�ο�·���±�����

uint8_t nSamplePointNum = 2;		//������ĸ���
float fSamplePeriod = 0.5f;			//����ʱ��(��)

float fSpeedAffactedFactor = 0.15f;			//�ٶ�Ӱ��ϵ��	
float fSpeedFixedFactor = 0.05f;				//�ٶȹ̶�ϵ��

bool isLineArithmeticFlag = false;			//�Ƿ�ֱ��׷��
bool isFirstPointFlag = false;					//�Ƿ��һ����
bool isFirstMoveFlag = false;						//�Ƿ�ƽ��·��

float fLookAheadDistanceFactor = 4.0f;	//ǰ�Ӿ���ϵ��

/* Private Functions ---------------------------------------------------------*/

/**
* @name:				GetArithmeticSquareRoot
* @brief:				����������������ƽ����
* @in:					n1,n2[float] �����������
* @out:					��
* @retval:			[float]������������ƽ����
* @reviseTime:	2018-09-20
*/
float GetArithmeticSquareRoot(float n1, float n2)
{
//���ʹ����Ӳ�������
#if (__FPU_PRESENT == 1)
	
	float fSqrtOut = 0.0f;
	float fPowerIn[2] = {0.0f, 0.0f};
	float fPowerOut = 0.0f;
	
	//�������ָ���ֵ
	fPowerIn[0] = n1;
	fPowerIn[1] = n2;
	
	//��ƽ����
	arm_power_f32(fPowerIn, 2, &fPowerOut);
	
	//����ɹ� ����ƽ����
	if( arm_sqrt_f32(fPowerOut, &fSqrtOut) == ARM_MATH_SUCCESS)
	{
		return fSqrtOut;
	}
	//����ʧ�� ����-1
	else
	{
		return -1.0f;
	}
	
//���û��ʹ��Ӳ�������
#else
	
	return sqrtf(powf(n1,2) + powf(n2,2));
	
#endif
}

/**
* @name:				getPointAngle
* @brief:				�����һ���㵽�ڶ�����ĵĽǶ�
* @in:					p1,p2[point3d] �������
* @out:					��
* @retval:			[float]�Ƕ�[-180,180)
* @reviseTime:	2018-08-29
*/
inline float GetPointAngle(point3d p1, point3d p2)
{
	return atan2f(p2.y - p1.y, p2.x - p1.x) * (float)Rad2Degree ;
}

/**
* @name:					countLinePara
* @brief:					ͨ���������ֱ�ߵ�Ax+By+C=0�Ĳ���
* @in:						p1,p2[point3d] �����ο���
* @out:						l[lineInfo]	ֱ�ߵĲ���
* @retval:				��
* @reviseTime:		2019-04-19
*/
void CountLinePara(point3d p1, point3d p2, lineInfo *l)
{
	
	//���б�ʲ�����
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
* @brief:				����㵽ֱ�ߵľ���
* @in:					p[point3d] ������� | l[lineInfo]	ֱ�ߵĲ���
* @out:					��
* @retval:			[float]�㵽ֱ�ߵľ���
* @reviseTime:	2018-08-21
*/
inline float CountPointLineDistance(point3d p, lineInfo l)
{
		return (l.A*p.x + l.B*p.y + l.C) / GetArithmeticSquareRoot(l.A, l.B);
}

/**
* @name:			configureInitPathMemory
* @brief:			Ϊ��ʼ·�������ڴ��ַ
* @in:				��
* @out:				��
* @retval:			[bool]�Ƿ����ɹ�
* @reviseTime:		2019-03-11
*/
bool configureInitPathMemory(void)
{
	//����ͷβ���ĵ�ַ
	initPath.head = malloc(sizeof(initNode));
	initPath.tail = malloc(sizeof(initNode));

	//����ʧ�ܱ����� �ɹ������
	if (initPath.head == NULL || initPath.tail == NULL)
	{
		return false;
	}

	//����ͷ��� head-> <-tail
	initPath.head->prev = NULL;
	initPath.head->point.x = 0.0f;
	initPath.head->point.y = 0.0f;
	initPath.head->point.z = 0.0f;
	initPath.head->next = initPath.tail;

	//����β��� head-> <-tail
	initPath.tail->prev = initPath.head;
	initPath.tail->point.x = 0.0f;
	initPath.tail->point.y = 0.0f;
	initPath.tail->point.z = 0.0f;
	initPath.tail->next = NULL;

	//���õ�ĸ���
	initPath.nSize = 0;

	return true;
}

/**
* @name:				configureReferencePathMemory
* @brief:				Ϊ�ο�·�������ڴ��ַ
* @in:					��
* @out:					��
* @retval:			[bool]�Ƿ����ɹ�
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
* @brief:			��ӳ�ʼ��·��
* @in:				��
* @out:				��
* @retval:			[bool]�Ƿ���ӳɹ�
* @reviseTime:		2019-03-11
*/
bool addInitPoint(point3d p)
{
	initNode *temp;										//��Ҫ��ӵĽ��
	static initNode *lastPosition;		//��¼�ϴ���ӵ�λ��

	/*��һ������ �����׽��*/
	if (initPath.nSize == 0 && initPath.head->next == initPath.tail && initPath.tail->prev == initPath.head)
	{
		//���긳ֵ
		initPath.head->point.x = p.x;
		initPath.head->point.y = p.y;
		initPath.head->point.z = p.z;

		//���ݳ�Ϊ1��
		initPath.nSize = 1;
	}
	/* �ڶ������� ����β��� */
	else if (initPath.nSize == 1 && initPath.head->next == initPath.tail && initPath.tail->prev == initPath.head)
	{
		//���긳ֵ
		initPath.tail->point.x = p.x;
		initPath.tail->point.y = p.y;
		initPath.tail->point.z = p.z;

		//���ݳ�Ϊ2��
		initPath.nSize = 2;
		
		//��һ��λ�õ�ָ��ָ��ͷ�ڵ�
		lastPosition = initPath.head;
	}
	/* �������Լ��Ժ������ */
	else
	{
		//Ϊ���ݷ����ڴ� �������ʧ���򱨴��� �ɹ������
		temp = malloc(sizeof(initNode));
		if (temp == NULL)
		{
			return false;
		}		
		
		//����һ���µĽ��
		lastPosition->next = temp;
		
		//β�������ݷ��ڵ����ڶ������
		temp->prev = lastPosition;
		temp->point.x = initPath.tail->point.x;
		temp->point.y = initPath.tail->point.y;
		temp->point.z = initPath.tail->point.z;
		temp->next = initPath.tail;
		
		//���긳ֵ
		initPath.tail->point.x = p.x;
		initPath.tail->point.y = p.y;
		initPath.tail->point.z = p.z;
		initPath.tail->prev = temp;
		
		//������������һ��
		initPath.nSize++;
		
		//��һ��λ�õ�ָ��ָ����һ��λ��
		lastPosition = temp;
		
	}
	return true;
}

/**
* @name:				AddReferencePoint
* @brief:				��Ӳο�·��
* @in:					��
* @out:					��
* @retval:			��
* @reviseTime:	2019-03-11
*/
void AddReferencePoint(float x, float y, float z)
{
	/* ��Ӧλ����ӵ����Ϣ */
	(referencePath.point + referencePath.nSize)->x = x;
	(referencePath.point + referencePath.nSize)->y = y;
	(referencePath.point + referencePath.nSize)->z = z;

	/* ��ĸ�������һ�� */
	referencePath.nSize++;
}

/**
* @name:				clearReferencePath
* @brief:				����ο�·��
* @in:					��
* @out:					��
* @retval:			��
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
* @brief:			�����ʼ·��
* @in:				��
* @out:				��
* @retval:			[bool]�Ƿ���ӳɹ�
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

	//����ͷ���
	initPath.head->prev = NULL;
	initPath.head->point.x = 0.0f;
	initPath.head->point.y = 0.0f;
	initPath.head->point.z = 0.0f;
	initPath.head->next = initPath.tail;

	//����β���
	initPath.tail->prev = initPath.head;
	initPath.tail->point.x = 0.0f;
	initPath.tail->point.y = 0.0f;
	initPath.tail->point.z = 0.0f;
	initPath.tail->next = NULL;

	//���õ�ĸ���
	initPath.nSize = 0;
}

/**
* @name:				debugPrintInitPath
* @brief:				�����ʼ·��
* @in:					��
* @out:					��
* @retval:			��
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
* @brief:				����ο�·��
* @in:					��
* @out:					��
* @retval:			��
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
* @brief:				ת�������
* @in:					angle[float] ת��ǰ�����
* @out:					��
* @retval:			[float] ת�������[0,360)
* @reviseTime:	2018-09-21
*/
__inline float changeCourseAngle(float angle)
{
	/* �Ƕ�ת�� */
	angle = 450.0f - angle;
	
	/* �Ƕ�ת����[0,360)֮�� */
	angle = (angle >= 360.0f) ? angle - 360.0f : ( angle < 0.0f ) ? angle + 360.0f : angle ;
	
	return angle;
}


/**
* @name:				JWG2ENU
* @brief:				��γ������ϵת��������������ϵ
* @in:					j,w,g[double] ��γ������
* @out:					p[point] ����������
* @retval:			��
* @reviseTime:	2018-09-20
*/
void JWG2ENU(double j, double w, double g, point3d *p)
{
	//����ת����ϵ������
	static double dTransforMatrix[3][3] = {{0.0}};
	//ecef0������ϵ����ֵ
	static double ecefx0 = 0.0, ecefy0 = 0.0, ecefz0 = 0.0;

	//�Ƕȵ�ϵ��
	double sinp = 0, cosp = 0.0, sinl = 0.0, cosl = 0.0;
	//����ת��ϵ��
	double v = 0.0;
	//ecef������ϵ����ֵ
	double ecefx = 0.0, ecefy = 0.0, ecefz = 0.0;

	//�Ƕ�ת����
	j = j * Degree2Rad;
	w = w * Degree2Rad;

	//����sin��cos
	sinp = sin(w);
	cosp = cos(w);
	sinl = sin(j);
	cosl = cos(j);

	//���õ�һ����Ĳ���
	if (isFirstPointFlag == true)
	{
		//ֻ����һ��
		isFirstPointFlag = false;

		//����ת�������ϵ��
		dTransforMatrix[0][0] = -sinl;					dTransforMatrix[0][1] = cosl;						dTransforMatrix[0][2] = 0.0;
		dTransforMatrix[1][0] = -sinp * cosl;   dTransforMatrix[1][1] = -sinp * sinl;   dTransforMatrix[1][2] = cosp;
		dTransforMatrix[2][0] = cosp * cosl;    dTransforMatrix[2][1] = cosp * sinl;    dTransforMatrix[2][2] = sinp;

		//����ecef0��ֵ ��λ����ת��Ϊ�� ���ʹ�doubleתdoubel
		v = RE_WGS84 / sqrt(1.0 - e * sinp*sinp);
		ecefx0 = (v + g)*cosp*cosl;
		ecefy0 = (v + g)*cosp*sinl;
		ecefz0 = (v*(1.0 - e) + g)*sinp;

		//��һ������ĵ�϶���(0,0,0)
		p->x = 0.0f;
		p->y = 0.0f;
		p->z = 0.0f;
	}
	else
	{
		//jwgתecef ��λ����ת��Ϊ�� ���ʹ�doubleתdoubel
		v = RE_WGS84 / sqrt(1.0 - e * sinp*sinp);
		ecefx = (v + g)*cosp*cosl - ecefx0;
		ecefy = (v + g)*cosp*sinl - ecefy0;
		ecefz = (v*(1.0 - e) + g)*sinp - ecefz0;

		//ecefתenu ��λ����ת��Ϊ�ף����ʹ�doubleתfloat
		p->x = (float)(dTransforMatrix[0][0] * ecefx + dTransforMatrix[0][1] * ecefy + dTransforMatrix[0][2] * ecefz);
		p->y = (float)(dTransforMatrix[1][0] * ecefx + dTransforMatrix[1][1] * ecefy + dTransforMatrix[1][2] * ecefz);
		p->z = (float)(dTransforMatrix[2][0] * ecefx + dTransforMatrix[2][1] * ecefy + dTransforMatrix[2][2] * ecefz);
	}
}

/**
* @name:				setSamplingPeriod
* @brief:				���ò������ںͲ�����ĸ���
* @in:					period[uint8_t] �������ڣ���λ��hz��
* @out:					��
* @retval:			��
* @reviseTime:	2018-09-25
*/
__inline void setSamplingPeriod(uint8_t period)
{
	nSamplePointNum = period;
	fSamplePeriod = (float) (1.0f / period);
}

/**
* @name:				setLineArithmetic
* @brief:				�����Ƿ�ʹ��ֱ��׷���㷨
* @in:					status[bool] �Ƿ�����
* @out:					��
* @retval:			��
* @reviseTime:	2018-10-15
*/
void setLineArithmetic(bool status)
{
	isLineArithmeticFlag = status;
}

/**
* @name:				setFirstPoint
* @brief:				�����Ƿ��һ����ĺ���
* @in:					status[bool] �Ƿ�����
* @out:					��
* @retval:			��
* @reviseTime:	2018-09-20
*/
__inline void setFirstPoint(bool status)
{
	isFirstPointFlag = status;
}

/**
* @name:					setFirstMove
* @brief:					�����Ƿ��һ��ƽ��·���ĺ���
* @in:						status[bool] �Ƿ�����
* @out:						��
* @retval:				��
* @reviseTime:		2018-09-20
*/
__inline void setFirstMove(bool status)
{
	isFirstMoveFlag = status;
}

/**
* @name:				setSpeedAffactedFactor
* @brief:				�����ٶ�Ӱ��ϵ��
* @in:					[uint8_t] Ӱ��ϵ��
* @out:					��
* @retval:			��
* @reviseTime:	2019-04-19
*/
__inline void setSpeedAffactedFactor(uint8_t factor)
{
	fSpeedAffactedFactor = (float) (factor / 100.0f);
}

/**
* @name:				setSpeedFixedFactor
* @brief:				�����ٶȹ̶�ϵ��
* @in:					[uint8_t] Ӱ��ϵ��
* @out:					��
* @retval:			��
* @reviseTime:	2019-04-19
*/
__inline void setSpeedFixedFactor(uint8_t factor)
{
	fSpeedFixedFactor = (float) (factor / 100.0f);
}

/**
* @name:					setLookAheadDistanceFactor
* @brief:					����ǰ�Ӿ���ϵ��
* @in:						factor[uint8_t] ǰ�Ӿ���ϵ��
* @out:						��
* @retval:				��
* @reviseTime:		2018-09-25
*/
__inline void setLookAheadDistanceFactor(uint8_t factor)
{
	fLookAheadDistanceFactor = (float) ( factor / 10.0f );
}

/**
* @name:			generateReferencePath
* @brief:			���ɲο�·��
* @in:				��
* @out:				��
* @retval:			[bool]�Ƿ����ɳɹ�
* @reviseTime:		2019-03-12
*/
uint8_t GenerateReferencePath(point3d p, float headingAngle)
{
	float x00 = 0.0f, y00 = 0.0f, z00 = 0.0f;
	float fHeadingAngleError = 0.0f;
	float fReferenceHeadingAngle = 0.0;

	// 0 | �ж���û���㹻�ĵ�
	//�߶�
	if (isLineArithmeticFlag == false)
	{
		if (initPath.nSize < 2 * nSamplePointNum)
		{
			return TRACKING_STATUS_NO_ENOUGH_POINTS;
		}
	}
	//ֱ��
	else
	{
		if (initPath.nSize < 2)
		{
			return TRACKING_STATUS_NO_ENOUGH_POINTS;
		}
	}

	// 1 | �����ǰ����Ϣ
	clearReferencePath();

	// 2 | ���ݳ�ʼ·�����·����ַ
	//����׷�ٷ�������ռ��С
	uint16_t nReferencePointNum = (isLineArithmeticFlag == false )? initPath.nSize/nSamplePointNum : 2 ;
	//���ݸ��������ڴ� �������ʧ���򱨴� �ɹ������
	referencePath.point = malloc(nReferencePointNum * sizeof(point3d));
	if (referencePath.point == NULL)
	{
		return TRACKING_STATUS_NO_MEMORY;
	}

	// 3 | ����Ƕ�֮��Ĳ��
	//�ο��Ƕ� (-180, 180]
	fReferenceHeadingAngle = atan2f(initPath.tail->point.y - initPath.head->point.y, initPath.tail->point.x - initPath.head->point.x)*(float)Rad2Degree;
	//����� (0, 360]
	fHeadingAngleError = headingAngle - fReferenceHeadingAngle;
	//������� ת����(-180, 180]
	fHeadingAngleError = (fHeadingAngleError >= 180.0f) ? fHeadingAngleError - 360.0f : ((fHeadingAngleError < -180.0f) ? fHeadingAngleError + 360.0f : fHeadingAngleError);

	// 4 | ƽ��·��
	//�߶�
	if (isLineArithmeticFlag == false)
	{
		uint16_t i = 0;
		//����ǲ�಻�� ���յ�һ��ƽ��
		if ( -90.0f <= fHeadingAngleError && fHeadingAngleError < 90.0f)
		{
			//��ȡ��һ�����λ��
			initNode * head = initPath.head;
			x00 = p.x - head->point.x;
			y00 = p.y - head->point.y;
			z00 = p.z - head->point.z;
			
			do
			{
				//ÿ��nSamplePointNum ȡ��
				if (i % nSamplePointNum == 0)
				{
					AddReferencePoint(head->point.x + x00, head->point.y + y00, head->point.z + z00);
				}
				//����һ����
				head = head->next;
			}while(head->next != NULL);
		}
		//���Ƚϴ��������һ��ƽ��
		else
		{
			//��ȡ���һ�����λ��
			initNode * tail = initPath.tail;
			x00 = p.x - tail->point.x;
			y00 = p.y - tail->point.y;
			z00 = p.z - tail->point.z;
	
			do
			{
				//ÿ��nSamplePointNum ȡ��
				if (i % nSamplePointNum == 0)
				{
					AddReferencePoint(tail->point.x + x00, tail->point.y + y00, tail->point.z + z00);
				}
				//��ǰһ����
				tail = tail->prev;
			}while(tail->prev != NULL);
		}
	}
	//ֱ��
	else
	{
		//����ǲ�಻�� ���յ�һ��ƽ��
		if (-90.0f <= fHeadingAngleError && fHeadingAngleError < 90.0f)
		{
			x00 = p.x - initPath.head->point.x;
			y00 = p.y - initPath.head->point.y;
			z00 = p.z - initPath.head->point.z;

			AddReferencePoint(initPath.head->point.x + x00, initPath.head->point.y + y00, initPath.head->point.z + z00);
			AddReferencePoint(initPath.tail->point.x + x00, initPath.tail->point.y + y00, initPath.tail->point.z + z00);
		}
		//���Ƚϴ��������һ��ƽ��
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
* @brief:				ģ��Ԥ�����׷��·��
* @in:					��ǰ��p[point3d] | �ٶ�speed[float] | �����headingAngle[float] | �໬��slipingAngle[float] | ��ǰת�򻡶�presentWheelSteeringRadian[float] 
* @out:					��������[command] | ׷��״̬[trackStatus]
* @retval:			׷�����[uint8_t]
* @reviseTime:	2019-05-29
*/
uint8_t autoRunPurePursuit(point3d p, float speed, float headingAngle, float slipingAngle, float presentWheelSteeringRadian, command* info, trackStatus* status)
{
	//����ת��뾶  ##��kʱ����Ҫk-1ʱ�̵�ת��뾶
	static float fTurningRadius;
#ifdef SLIP_ENABLE
	//������һ��ʱ�̵ĺ����
	static float fPrevTimeHeadingAngle;
	//������һ��ʱ�̵�x�����y����
	static float fPrevTimeX, fPrevTimeY;
#endif
	
	
	/*  0 | �ж��Ƿ��ƶ�·�� */
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
		//k-1ʱ�̵�ת��뾶����Ϊ�ܴ� ����ֱ����ʻ
		fTurningRadius = 1.0e5f;
#ifdef SLIP_ENABLE
		//�õ�ǰʱ�̵ĵ㵱��k-1ʱ�̵ĵ� �����һ��һ���ᱻ�ж������໬
		fPrevTimeX = p.x;
		fPrevTimeY = p.y;
#endif
	}
	
	/*  1 | ȷ����ǰҪ׷�ٵ������������ �Լ��Ƿ񵽵� */
	//���嵱ǰ׷��·���������˵�
	point3d prevPoint, nextPoint;
	//ֱ��׷��
	if(isLineArithmeticFlag == true)
	{
		//ֱ��׷��ֻ��Ҫ2����
		//��һ����
		prevPoint.x = (referencePath.point + 0)->x;
		prevPoint.y = (referencePath.point + 0)->y;
		prevPoint.z = (referencePath.point + 0)->z;
		
		//�ڶ�����
		nextPoint.x = (referencePath.point + (referencePath.nSize - 1))->x;
		nextPoint.y = (referencePath.point + (referencePath.nSize - 1))->y;
		nextPoint.z = (referencePath.point + (referencePath.nSize - 1))->z;
		
		//����ο�·�ߵĳ���
		float fReferenceLineLength = GetArithmeticSquareRoot(prevPoint.x - nextPoint.x, prevPoint.y - nextPoint.y);
		//�����ǰһ���˵�ľ���
		float fPrevDistance = GetArithmeticSquareRoot(p.x - prevPoint.x, p.y - prevPoint.y);
		
		//���������εı߳����� ����߱��л�ʱ ˵��ֱ��׷�ٵ����� ���ؽ���
		if (fPrevDistance >= fReferenceLineLength)
		{
			status->fHeadingAngleError = 0.0f;
			status->fLateralError = 0.0f;
			info->nDirection = DIRECTION_STATUS_MID;
			info->nWheelSteeringAngle = 0;
			return TRACKING_STATUS_TO_THE_END;
		}
	}
	//�߶�׷��
	else
	{
		//�±�ûԽ�� ˵����û���� ��ȡ�����˵��ֵ
		if (nReferencePointIndex + 1 <= referencePath.nSize - 1)
		{
			//ǰһ����
			prevPoint.x = (referencePath.point + nReferencePointIndex)->x;
			prevPoint.y = (referencePath.point + nReferencePointIndex)->y;
			prevPoint.z = (referencePath.point + nReferencePointIndex)->z;
			//��ǰһ����ľ���
			float fPresDistance = GetArithmeticSquareRoot(p.x - prevPoint.x, p.y - prevPoint.y);

			//��һ����
			nextPoint.x = (referencePath.point + (nReferencePointIndex + 1))->x;
			nextPoint.y = (referencePath.point + (nReferencePointIndex + 1))->y;
			nextPoint.z = (referencePath.point + (nReferencePointIndex + 1))->z;
			//����һ����ľ���
			float fNextDistance = GetArithmeticSquareRoot(p.x - nextPoint.x, p.y - nextPoint.y);
			
			//����뵱ǰ��ľ�������¸��㣬�л��ο���
			if (fPresDistance > fNextDistance)
			{
				nReferencePointIndex++;
				
				//����л���Խ��,��˵�����ײ���
				if (nReferencePointIndex + 1 == referencePath.nSize)
				{
					status->fHeadingAngleError = 0.0f;
					status->fLateralError = 0.0f;
					info->nDirection = DIRECTION_STATUS_MID;
					info->nWheelSteeringAngle = 0;
					return TRACKING_STATUS_TO_THE_END;
				}
				//û�еĻ� ���¶�λ��
				prevPoint.x = (referencePath.point + nReferencePointIndex)->x;
				prevPoint.y = (referencePath.point + nReferencePointIndex)->y;
				prevPoint.z = (referencePath.point + nReferencePointIndex)->z;

				nextPoint.x = (referencePath.point + (nReferencePointIndex + 1))->x;
				nextPoint.y = (referencePath.point + (nReferencePointIndex + 1))->y;
				nextPoint.z = (referencePath.point + (nReferencePointIndex + 1))->z;				
			}
		}
	}
	
//���Ӳ��FPU����ʹ�� ��ʹ��Ӳ��fpu����һЩ��Ϣ
#if( __FPU_PRESENT == 1)
	float fSinValue, fCosValue;
	float fSqrtOut;
#endif
	
	/*  3 | �Ľ��ʹ�׷���㷨��������ת�����Ϣ */
	/** -------------------------------- �ͺ���Ԥ�⼰���� -------------------------------- **/
#ifdef CONTROLDELAY_ENABLE
	// �����ٶȼ����ӳ�ʱ��
	float fDelayTime = fSpeedFixedFactor + fSpeedAffactedFactor * speed;
	// �������ӳ�ʱ���ڵĺ���ǵı仯 ������k-1ʱ�̵�ת��뾶
	float fDelayTimeChangeHeadingRadian = speed * fDelayTime / fTurningRadius;
#if ( __FPU_PRESENT==1 )
	//���㳵������ϵ��ֵ
	float fBodyX = fTurningRadius * (1 - arm_cos_f32(fDelayTimeChangeHeadingRadian));
	float fBodyY = fTurningRadius * arm_sin_f32(fDelayTimeChangeHeadingRadian);
	//���㳵������ϵ�¸ı��ֵ
	arm_sin_cos_f32(headingAngle, &fSinValue, &fCosValue);
	float fDelayTimeChangeX = fBodyX * fSinValue + fBodyY * fCosValue;
	float fDelayTimeChangeY = fBodyY * fSinValue - fBodyX * fCosValue;
#else
	//���㳵������ϵ��ֵ
	fBodyX = fTurningRadius * (1 - cosf(fDelayTimeChangeHeadingRadian));
	fBodyY = fTurningRadius * sinf(fDelayTimeChangeHeadingRadian);
	//���㳵������ϵ�¸ı��ֵ
	float fHeadingRadian = headingAngle * (float)Degree2Rad;
	float sinH = sinf(fHeadingRadian), cosh = cosf(fHeadingRadian);
	float fDelayTimeChangeX = fBodyX * sinH + fBodyY * cosH;
	float fDelayTimeChangeY = fBodyY * sinH - fBodyX * cosH;
#endif
	
#endif

	/** -------------------------------- �໬��Ԥ�⼰���� -------------------------------- **/
#ifdef SLIP_ENABLE
	//Ԥ�������λ��
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
	//������Ч�໬��
	float fEquivalentSlipingAngle = 0.0f;
	//�������Ƚϴ� �ж������˲໬ �����Ч�໬�� ����ͻ���0
	if(GetArithmeticSquareRoot(p.x - fPredictedX, p.y - fPredictedY) > 0.1f)
	{
		//Ԥ������������
		float fPredictedHeadingAngle = fPrevTimeHeadingAngle + (speed*fSamplePeriod/fTurningRadius)*(float)(Rad2Degree);
		//���ֲ໬��
		float fRearWheelSlipingRadian = (headingAngle - slipingAngle)*(float)Degree2Rad;
		//ǰ�ֲ໬��
		float fFrontWheelSlipingRadian = (headingAngle - fPredictedHeadingAngle)*(float)Degree2Rad;
		//�����Ч�໬��
		fEquivalentSlipingAngle = (1.0f/(1.0f/presentWheelSteeringRadian + 1.0f/(presentWheelSteeringRadian+fRearWheelSlipingRadian) + 1.0f/fFrontWheelSlipingRadian) - presentWheelSteeringRadian)*(float)(Rad2Degree);
	}
	//������һ��ʱ��λ�� �ͺ����
	fPrevTimeX = p.x;
	fPrevTimeY = p.y;
	fPrevTimeHeadingAngle = headingAngle;
#endif

	/** ----------------------------------- ��׷���㷨 ----------------------------------- **/
	//����ο�·���Ƕ� ������׼�ĵ�ĽǶ�
	float fReferenceLineAngle = GetPointAngle(prevPoint, nextPoint);
	float fAimPointAngle = GetPointAngle(p, nextPoint);
	
	//���㺽������
	headingAngle = (headingAngle > 180.0f) ? headingAngle - 360.0f : ((headingAngle <= -180.0f) ? headingAngle + 360.0f : headingAngle);
	float fHeadingAngleError = fReferenceLineAngle - headingAngle;
	fHeadingAngleError = (fHeadingAngleError > 180.0f) ? fHeadingAngleError - 360.0f : ((fHeadingAngleError <= -180.0f) ? fHeadingAngleError + 360.0f : fHeadingAngleError);
	status->fHeadingAngleError = fHeadingAngleError;
	
	//����������
	//ȷ���ο��ο��߶ε���Ϣ
	lineInfo referenceLine;
	CountLinePara(prevPoint, nextPoint, &referenceLine);
	//����㵽ֱ�ߵľ���
	status->fLateralError = CountPointLineDistance(p, referenceLine);
	
#ifdef CONTROLDELAY_ENABLE
	// Ԥ��δ��ʱ�̵�λ��
	p.x = p.x + fDelayTimeChangeX;
	p.y = p.y + fDelayTimeChangeY;
	//�����µĵ㵽ֱ�ߵľ���
	float fLateralError = CountPointLineDistance(p, referenceLine);
#else
	// ȷ������������ֵ
	float fLateralError = status->fLateralError;
#endif
	//ת��Ĭ�Ϻ���ǵ�[0-360)
	fReferenceLineAngle = (fReferenceLineAngle < 0.0f) ? fReferenceLineAngle + 360.0f : fReferenceLineAngle;
	//ȷ���������ķ���
	if (90.0f < fReferenceLineAngle  && fReferenceLineAngle <= 270.0f)
	{
		fLateralError = -fLateralError;
		status->fLateralError = -status->fHeadingAngleError;
	}
	
	//����ǰ�Ӿ���
	//float fLookAheadDistance = fLookAheadDistanceFactor * speed;
	float fLookAheadDistance = 4.0f;
	
	//����ת��Ƕ�
	//����ת���
	float fWheelSteeringAngle;
	//��ƫ���̫������ʱ��
	if (fabsf(fLateralError) > fLookAheadDistance)
	{
		//����kʱ�̵�ת��뾶
		fTurningRadius = CarWheelBearingDistance / tanf(MaxSwerveAngle*(float)Degree2Rad);
		// ����ת��Ƕ�
		if (fabsf(fAimPointAngle - headingAngle) < 180.0f)
		{
			fWheelSteeringAngle = (fAimPointAngle < headingAngle) ? -MaxSwerveAngle : MaxSwerveAngle;
		}
		else
		{
			fWheelSteeringAngle = (fAimPointAngle < headingAngle) ? MaxSwerveAngle : -MaxSwerveAngle;
		}
	}
	//ƫת�Ĳ��Ǻ�����
	else
	{
#if ( __FPU_PRESENT==1 )
		//����ǰ�Ӿ���ͺ�������ƽ����ĸ�
		arm_sqrt_f32(fLookAheadDistance*fLookAheadDistance - fLateralError * fLateralError, &fSqrtOut);
		//�������ǵ�sin��cosֵ
		arm_sin_cos_f32(fHeadingAngleError, &fSinValue, &fCosValue);
		//����kʱ�̵�ת��뾶
		fTurningRadius = 0.5f * (fLookAheadDistance*fLookAheadDistance) / (fLateralError*fCosValue + fSqrtOut*fSinValue);
#else
		fTurningRadius = 0.5f * (fLookAheadDistance*fLookAheadDistance) / (fLateralError*cosf(fHeadingAngleError*(float)Degree2Rad) + sqrtf(fLookAheadDistance*fLookAheadDistance - fLateralError*fLateralError)*sinf(HeadingAngleError(float)Degree2Rad));
#endif
		
		//����ת��Ƕ�
#ifdef SLIP_ENABLE	
		fWheelSteeringAngle = atanf(CarWheelBearingDistance/fTurningRadius) * (float)Rad2Degree + fEquivalentSlipingAngle;
#else
		fWheelSteeringAngle = atanf(CarWheelBearingDistance/fTurningRadius) * (float)Rad2Degree;
#endif
		//���̫��;�����ȥ
		fWheelSteeringAngle = (fWheelSteeringAngle > MaxSwerveAngle) ? MaxSwerveAngle : ((fWheelSteeringAngle < -MaxSwerveAngle) ? -MaxSwerveAngle : fWheelSteeringAngle);
	}
	
	/*  4 | ����Ϣת��Ϊ������Ϣ����� */
	info->nDirection = (fabsf(fWheelSteeringAngle) < 2e-2f ) ? DIRECTION_STATUS_MID : (fWheelSteeringAngle > 0) ? DIRECTION_STATUS_LEFT : DIRECTION_STATUS_RIGHT;
	info->nWheelSteeringAngle = (uint16_t)(fabsf(fWheelSteeringAngle *10.0f));
	
	return TRACKING_STATUS_NORMAL;
}

/* END OF FILE */
