/**
	******************************************************************************
	* File Name          : smpc.c
	* Description        : ����ļ��������Զ���ʻ�ĺ���ʵ��
	******************************************************************************
	*
	* COPYRIGHT(c) 2019-2020 ���ϴ�ѧ-������ѧ�빤��ѧԺ-ʯ�ѳ� (QQ:369348508)
	*
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
	* 2-���˫�������ԭ��
	* 3-�������ӳ�ʼ·������
	*
	*
  ******************************************************************************
*/

/* Include ------------------------------------------------------------------ */

#include "stracking.h"

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
	return atan2f(p2.y - p1.y, p2.x - p1.x) * 57.29577951308237971f ;
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
	}
	/* ���������� ����һ����� ͬʱβ���������ƶ� */
	else if (initPath.nSize == 2 && initPath.head->next == initPath.tail && initPath.tail->prev == initPath.head)
	{
		//Ϊ��¼����������ڴ�ռ� ����ʧ���򱨴�
		lastPosition = malloc(sizeof(initNode));
		if (lastPosition == NULL)
		{
			return false;
		}
		
		//Ϊ���ݷ����ڴ� �������ʧ���򱨴��� �ɹ������
		temp = malloc(sizeof(initNode));
		if (temp == NULL)
		{
			return false;
		}

		initPath.head->next = temp;

		temp->prev = initPath.head;
		temp->point.x = initPath.tail->point.x;
		temp->point.y = initPath.tail->point.y;
		temp->point.z = initPath.tail->point.z;
		temp->next = initPath.tail;

		initPath.tail->prev = temp;
		initPath.tail->point.x = p.x;
		initPath.tail->point.y = p.y;
		initPath.tail->point.z = p.z;
		
		//��¼�µ�ǰλ�õĵ�ַ
		lastPosition = temp;

		initPath.nSize = 3;
	}
	/* ���ĸ��Լ��Ժ������ */
	else
	{
		//Ϊ���ݷ����ڴ� �������ʧ���򱨴��� �ɹ������
		temp = malloc(sizeof(initNode));
		if (temp == NULL)
		{
			return false;
		}		
		
		//��ȡ��һ������λ��
		lastPosition->next = temp;

		//��������
		temp->prev = lastPosition;
		temp->point.x = initPath.tail->point.x;
		temp->point.y = initPath.tail->point.y;
		temp->point.z = initPath.tail->point.z;
		temp->next = initPath.tail;

		initPath.tail->prev = temp;
		initPath.tail->point.x = p.x;
		initPath.tail->point.y = p.y;
		initPath.tail->point.z = p.z;

		//��¼�µ�ǰλ�õĵ�ַ
		lastPosition = temp;

		//������������һ��
		initPath.nSize++;
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

	head = initPath.head;

	for (uint16_t i = 0; i < initPath.nSize; i++)
	{
		temp = head->next;
		head->prev = NULL;
		free(head);
		head = temp;
	}

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
	static double dTransforMatrix[3][3] = { 0.0 };
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
	uint8_t nReferencePointNum = 0;

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
	nReferencePointNum = (isLineArithmeticFlag == false )? initPath.nSize / nSamplePointNum : 2 ;
	//���ݸ��������ڴ� �������ʧ���򱨴� �ɹ������
	referencePath.point = malloc(nReferencePointNum * sizeof(point3d));
	if (referencePath.point == NULL)
	{
		return TRACKING_STATUS_NO_MEMORY;
	}

	// 3 | ����Ƕ�֮��Ĳ��
	fReferenceHeadingAngle = atan2f(initPath.tail->point.y - initPath.head->point.y, initPath.tail->point.x - initPath.head->point.x);
	fHeadingAngleError = headingAngle - fReferenceHeadingAngle;
	fHeadingAngleError = (fHeadingAngleError > 180.0f) ? fHeadingAngleError - 360.0f : ((fHeadingAngleError <= -180.0f) ? fHeadingAngleError + 360.0f : fHeadingAngleError);

	// 4 | ƽ��·��
	//�߶�
	if (isLineArithmeticFlag == false)
	{
		uint16_t i;
		//����ǲ�಻�� ���յ�һ��ƽ��
		if (-90.0f <= fHeadingAngleError && fHeadingAngleError < 90.0f)
		{
			//��ȡ��һ�����λ��
			initNode * head = initPath.head;
			x00 = p.x - head->point.x;
			y00 = p.y - head->point.y;
			z00 = p.z - head->point.z;
			for (i = 0; i < initPath.nSize; i++)
			{
				//ÿ��nSamplePointNum ȡ��
				if (i % nSamplePointNum == 0)
				{
					AddReferencePoint(head->point.x + x00, head->point.y + y00, head->point.z + z00);
				}
				//����һ����
				head = head->next;
			}
		}
		//���Ƚϴ��������һ��ƽ��
		else
		{
			//��ȡ���һ�����λ��
			initNode * tail = initPath.tail;
			x00 = p.x - tail->point.x;
			y00 = p.y - tail->point.y;
			z00 = p.z - tail->point.z;
			for (i = 0; i < initPath.nSize; i++)
			{
				//ÿ��nSamplePointNum ȡ��
				if (i % nSamplePointNum == 0)
				{
					AddReferencePoint(tail->point.x + x00, tail->point.y + y00, tail->point.z + z00);
				}
				//��ǰһ����
				tail = tail->prev;
			}
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
* @in:					��ǰ��p[point3d] | �ٶ�speed[float] | �����headingAngle[float]
* @out:					��������[command] | ׷��״̬[trackStatus]
* @retval:			׷�����[uint8_t]
* @reviseTime:	2019-03-13
*/
uint8_t autoRunPurePursuit(point3d p, float speed, float headingAngle, command* info, trackStatus* status)
{
	//���嵱ǰ�����һ����
	point3d presPoint, nextPoint;
	//���嵱ǰ�ο���ľ������һ���ο���ľ���
	float fPresDistance, fNextDistance;
	//����ο��߶�
	lineInfo referenceLine;
	//����������
	float fLateralError;
	//����Ƕ����
	float fHeadingAngleError;
	//����ο��߶εĺ����
	float fReferenceLineAngle, fReferenceLineLength = 0.0f;
	//������׼�ĽǶȺ���׼�߶εĳ���
	float fAimPointAngle;
	//����ǰ�Ӿ���
	float fLookAheadDistance;
	//��������ǰ��ת���
	float fWheelSteeringAngle;
	//����ת��뾶
	static float fTurningRadius = 37.5f;
	//�����ӳ�ʱ��
	float fDelayTime;
	//�����ӳ�ʱ��ĺ���Ǳ仯
	float fDelayTimeChangeHeadingRadius;
	//���峵������ϵ�µı仯��
	float fDelayTimeChangeX, fDelayTimeChangeY;
	//���峵������ϵ�µ�����ֵ
	float fBodyX, fBodyY;

#if ( __FPU_PRESENT==1 )
	static float fSqrtOut = 0.0f;
	static float fsinOut = 0.0f, fcosOut = 0.0f;
#endif

	/*  0 | �ƶ�·�� */
	if(isFirstMoveFlag == true)
	{
		isFirstMoveFlag = false;
		uint8_t res = GenerateReferencePath(p,headingAngle);
		if(res != TRACKING_STATUS_NORMAL)
		{
			return res;
		}
	}

	/*  1 | ȷ����ǰҪ׷�ٵ������������ �Լ��Ƿ񵽵�*/
	//�����ֱ��׷��
	if (isLineArithmeticFlag == true)
	{
		//ֱ��׷��ֻ��Ҫ2����
		//��һ����
		presPoint.x = (referencePath.point + 0)->x;
		presPoint.y = (referencePath.point + 0)->y;
		presPoint.z = (referencePath.point + 0)->z;
		fPresDistance = GetArithmeticSquareRoot(p.x - presPoint.x, p.y - presPoint.y);

		//�ڶ�����
		nextPoint.x = (referencePath.point + (referencePath.nSize - 1))->x;
		nextPoint.y = (referencePath.point + (referencePath.nSize - 1))->y;
		nextPoint.z = (referencePath.point + (referencePath.nSize - 1))->z;
		fNextDistance = GetArithmeticSquareRoot(p.x - nextPoint.x, p.y - nextPoint.y);

		//����·�߳���
		fReferenceLineLength = GetArithmeticSquareRoot(presPoint.x - nextPoint.x, presPoint.y - nextPoint.y);
		//�ж��Ƿ����
		if (fPresDistance > fReferenceLineLength)
		{
			return TRACKING_STATUS_TO_THE_END;
		}
	}
	//�߶�׷��
	else
	{
		// �±�ûԽ�� �ж��Ƿ���Ҫ�л��ο���
		if (nReferencePointIndex + 1 <= referencePath.nSize - 1)
		{
			//�߶�׷����Ҫ��ͣ�ı任��
			//ǰһ����
			presPoint.x = (referencePath.point + nReferencePointIndex)->x;
			presPoint.y = (referencePath.point + nReferencePointIndex)->y;
			presPoint.z = (referencePath.point + nReferencePointIndex)->z;
			//��ǰһ����ľ���
			fPresDistance = GetArithmeticSquareRoot(p.x - presPoint.x, p.y - presPoint.y);

			//��һ����
			nextPoint.x = (referencePath.point + (nReferencePointIndex + 1))->x;
			nextPoint.y = (referencePath.point + (nReferencePointIndex + 1))->y;
			nextPoint.z = (referencePath.point + (nReferencePointIndex + 1))->z;
			//����һ����ľ���
			fNextDistance = GetArithmeticSquareRoot(p.x - nextPoint.x, p.y - nextPoint.y);

			//����뵱ǰ��ľ�������¸��㣬�л��ο���
			if (fPresDistance > fNextDistance)
			{
				nReferencePointIndex++;

				//����л���Խ��,��˵�����ײ���
				if (nReferencePointIndex + 1 == referencePath.nSize)
				{
					return TRACKING_STATUS_TO_THE_END;
				}
				//û�еĻ� ���¶�λ��
				presPoint.x = (referencePath.point + nReferencePointIndex)->x;
				presPoint.y = (referencePath.point + nReferencePointIndex)->y;
				presPoint.z = (referencePath.point + nReferencePointIndex)->z;

				nextPoint.x = (referencePath.point + (nReferencePointIndex + 1))->x;
				nextPoint.y = (referencePath.point + (nReferencePointIndex + 1))->y;
				nextPoint.z = (referencePath.point + (nReferencePointIndex + 1))->z;
			}
		}
		//����±�Խ�� �϶��ǵ�����
		else
		{
			return TRACKING_STATUS_TO_THE_END;
		}
	}

	/* 2 | �Ľ��ʹ�׷���㷨����·��׷�� */
	/** -------------------------------- �ͺ���Ԥ�⼰���� -------------------------------- **/
	// 2-1-1| �����ٶȼ����ӳ�ʱ��
	fDelayTime = fSpeedFixedFactor + fSpeedAffactedFactor * speed;

	// 2-1-2| �����ӳ�ʱ���ڵ������λ�� ##ʹ������һ�ε�ת��뾶
	// ���㺽��ǵı仯
	fDelayTimeChangeHeadingRadius = speed * fDelayTime / fTurningRadius;
#if ( __FPU_PRESENT==1 )
	//���㳵������ϵ��ֵ
	fBodyX = fTurningRadius * (1 - arm_cos_f32(fDelayTimeChangeHeadingRadius));
	fBodyY = fTurningRadius * arm_sin_f32(fDelayTimeChangeHeadingRadius);

	//���㳵������ϵ�¸ı��ֵ
	arm_sin_cos_f32(headingAngle, &fsinOut, &fcosOut);
	fDelayTimeChangeX = fBodyX * fsinOut + fBodyY * fcosOut;
	fDelayTimeChangeY = fBodyY * fsinOut - fBodyX * fcosOut;
#else
	//���㳵������ϵ��ֵ
	fBodyX = fTurningRadius * (1 - cosf(fTurningRadius));
	fBodyY = fTurningRadius * sinf(fTurningRadius);

	//���㳵������ϵ�¸ı��ֵ
	fDelayTimeChangeX = fBodyX * sinf(headingAngle) + fBodyY * cosf(headingAngle);
	fDelayTimeChangeY = fBodyY * sinf(headingAngle) - fBodyX * cosf(headingAngle);
#endif

	/** ----------------------------------- ��׷���㷨 ---------------------------------- **/
	// 2-2 | ����ο�·���Ƕȣ�������׼�Ƕ�
	fReferenceLineAngle = GetPointAngle(presPoint, nextPoint);
	fAimPointAngle = GetPointAngle(p, nextPoint);

	// 2-3 | ����Ƕ����
	headingAngle = (headingAngle > 180.0f) ? headingAngle - 360.0f : ((headingAngle <= -180.0f) ? headingAngle + 360.0f : headingAngle);
	fHeadingAngleError = fReferenceLineAngle - headingAngle;

	//����ת����[-180,180)
	fHeadingAngleError = (fHeadingAngleError > 180.0f) ? fHeadingAngleError - 360.0f : ((fHeadingAngleError <= -180.0f) ? fHeadingAngleError + 360.0f : fHeadingAngleError);
	// 2-4	| ����������
	// 2-4-1| ͨ��2���ο������ο��߶β���
	CountLinePara(presPoint, nextPoint, &referenceLine);
	// 2-4-2| �����Ż��Ժ�ĵ��λ��
	p.x = p.x + fDelayTimeChangeX;
	p.y = p.y + fDelayTimeChangeY;
	// 2-4-3| �����µĺ������
	fLateralError = CountPointLineDistance(p, referenceLine);
	// 2-4-4| ת���ɶ�Ӧ������,���������������,�ڳ������Ź滮·�������Ҳ�Ϊ��
	//ת��Ĭ�Ϻ���ǵ�[0-360)
	fReferenceLineAngle = (fReferenceLineAngle < 0.0f) ? fReferenceLineAngle + 360.0f : fReferenceLineAngle;
	//����������
	if (90.0f < fReferenceLineAngle  && fReferenceLineAngle <= 270.0f)
	{
		fLateralError = -fLateralError;
	}

	// 2-5 | ����ǰ�Ӿ���
	//fLookAheadDistance = fLookAheadDistanceFactor * speed * fSamplePeriod * nSamplePointNum;
	fLookAheadDistance = 4.0f;

	// 2-6 | ����ת��Ƕ�
	//��ƫ���̫������ʱ��
	if (fabsf(fLateralError) > fLookAheadDistance)
	{
		// 2-61| ����ת��뾶 ( tan(fMaxSwerveRadius) = 1 ��ʱ������ )
		//fTurningRadius = CarWheelBearingDistance / tanf(fMaxSwerveRadius);
		fTurningRadius = CarWheelBearingDistance;
		// 2-62| ����ת��Ƕ�
		if (fabsf(fAimPointAngle - headingAngle) < 180.0f)
		{
			fWheelSteeringAngle = (fAimPointAngle < headingAngle) ? -MaxSwerveAngle : MaxSwerveAngle;
		}
		else
		{
			fWheelSteeringAngle = (fAimPointAngle < headingAngle) ? MaxSwerveAngle : -MaxSwerveAngle;
		}
	}
	else
	{
		//	2-61| ����ת��뾶
#if ( __FPU_PRESENT==1 )

		//����ǰ�Ӿ���ͺ�������ƽ����ĸ�
		arm_sqrt_f32(fLookAheadDistance*fLookAheadDistance - fLateralError * fLateralError, &fSqrtOut);

		//�������ǵ�sin��cosֵ
		arm_sin_cos_f32(fHeadingAngleError, &fsinOut, &fcosOut);

		//����ת��뾶
		fTurningRadius = 0.5f * (fLookAheadDistance * fLookAheadDistance) / (fLateralError*fcosOut + fSqrtOut * fsinOut);
#else
		fTurningRadius = 0.5f * powf(fLookAheadDistance, 2) / (fLateralError*cosf(Degree2Rad*fHeadingAngleError) + sqrtf(fLookAheadDistance*fLookAheadDistance - fLateralError * fLateralError)*sinf(Degree2Rad*fHeadingAngleError));
#endif
		//	2-72| ����ת��Ƕ�
		fWheelSteeringAngle = atanf(CarWheelBearingDistance / fTurningRadius) * 57.29577951308237971f;

		//	2-73| ���̫��;�����ȥ
		fWheelSteeringAngle = (fWheelSteeringAngle > MaxSwerveAngle) ? MaxSwerveAngle : ((fWheelSteeringAngle < -MaxSwerveAngle) ? -MaxSwerveAngle : fWheelSteeringAngle);
	
	}
	/* 3 | �ж�ת�䷽��ͽǶ� */
	info->nDirection = (fWheelSteeringAngle > 0) ? DIRECTION_STATUS_LEFT : DIRECTION_STATUS_RIGHT;
	info->nWheelSteeringAngle = (uint16_t)(fabsf(fWheelSteeringAngle * 10.0f));

	/* 4 | ���ر�Ҫ���� */
	status->fHeadingAngleError = fHeadingAngleError;
	status->fLateralError = fLateralError;
	
	return TRACKING_STATUS_NORMAL;
}

/* END OF FILE */
