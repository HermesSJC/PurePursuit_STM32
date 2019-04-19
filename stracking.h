/**
	******************************************************************************
	* File Name          : stracking.h
	* Description        : ����ļ��������Զ���ʻ�����һ�ж���ͺ�������
	******************************************************************************
	*
	* COPYRIGHT(c) 2019-2020 ���ϴ�ѧ-������ѧ�빤��ѧԺ-ʯ�ѳ� (QQ:369348508)
	*
	* api˵���ĵ�
	*
	* ----------------------------------------------------------------------------
	*
	* 01-���ó�ʼ·���Ķ�̬�ڴ����
	* ����ԭ�� bool configureInitPathMemory(void);	
	* ����true  ��ʼ���ɹ�
	* ����false ��ʼ��ʧ��
	*
	* ----------------------------------------------------------------------------
	*
	* 02-���òο�·���Ķ�̬�ڴ����
	* ����ԭ�� bool configureReferencePathMemory(void)
	* ����true  ��ʼ���ɹ�
	* ����false ��ʼ��ʧ��
	*
	* ----------------------------------------------------------------------------
	*
	* 03-����ο�·��
	* ����ԭ�� void clearReferencePath(void);
	*
	* ----------------------------------------------------------------------------
	*
	* 04-�����ʼ·��
	* ����ԭ�� void clearInitPath(void);		
	*
	* ----------------------------------------------------------------------------
	*
	* 05-��ӵ�һ���˹����ߵĲο���
	* ����ԭ�� bool addInitPoint(point3d p);
	* ������� [point3d] ����������ϵ�ĵ�
	* ����true  ��ӳɹ�
	* ����false ���ʧ��
	*
	* ----------------------------------------------------------------------------
	*
	* 06-ת����ʼ�����
	* ����ԭ�� float changeCourseAngle(float angle);
	* ������� ԭʼ�����
	* ���ز��� ת���Ժ�ĺ����
	*
	* ----------------------------------------------------------------------------
	*
	* 07-ת������ϵ
	* ����ԭ�� void JWG2ENU(double j,double w,double g, point3d *p);
	* ������� double���� ���� ���� ά�� �߶ȣ��ȣ��ȣ��ף�
	* ������� ת������Ժ� point���͵�ĵ�ַ ����ϵ �� �� �죨�ף��ף��ף�
	*
	* ----------------------------------------------------------------------------
	* 
	* 08-�Զ�����
	* ����ԭ�� int autoRunPurePursuit(point3d p, float speed, float courseAngle, command *info, trackStatus *status);
	* ������� point���͵Ķ���������ϵ�ĵ㣨�ף��ף��ף����ٶȣ���/�룩��ת����ĺ���ǣ��ȣ�
	* ������� command���͵� info�ĵ�ַ
	*           command::nDirection  |- ����
	*						|- DIRECTION_MID      �м�
	*						|- DIRECTION_LEFT			��ת
	*						|- DIRECTION_RIGHT		��ת
	*						command::nSwerveAngle |- ����ת��ǣ�0.1�ȣ�
	*					 trackStatus���͵� status�ĵ�ַ
	*	 				  trackStatus::fLateral �������
	*           trackStatus::fheadingAngleError �������
	* ���ز���  |------TRACKING_STATUS_NORMAL               ��������
	*           |------TRACKING_STATUS_NO_ENOUGH_POINTS     û���㹻��
	*           |------TRACKING_STATUS_NO_MEMORY						û���ڴ� gg
	*						|------TRACKING_STATUS_TO_THE_END						�ߵ���
	*
	* ----------------------------------------------------------------------------
	*
	* 09-�����Ƿ���Ҫ����ƽ��·��
	* ����ԭ�� void setFirstMove(bool status);
  * ����true  ��һ������Ϊƽ��Ԥ��·���Ĳο��� 
	*           ++ ���棺������֮ǰ��Ԥ��·���Ͳο�·���Ͳο���
	* ����false ��һ����ȡ����Ϊƽ��Ԥ��·���Ĳο��� 
	*
	* ----------------------------------------------------------------------------
	*
	* 10-�����Ƿ�Ҫ����һ��������Ϊ����������ϵ��ԭ��
	* ����ԭ�� void setFirstPoint(bool status);
	* ����true  ��һ������Ϊ����������ϵ��ԭ�� 
	*           ++ ���棺������֮ǰ������ϵ��ԭ��
	* ����false ��һ����ȡ����Ϊ����������ϵ��ԭ��
	*
	* ----------------------------------------------------------------------------
	* 
	* 11-���ò������ںͲο������
	* ����ԭ�� void setSamplingPeriod(uint8_t samplingPeriod);
	* ������� �������� ��λ�����ȣ�hz��
	*
	* ----------------------------------------------------------------------------
	* 
	* 12-����ǰ�Ӿ���ϵ��
	* ����ԭ�� void setSightDistanceFactor(uint8_t factor);
  * ������� ǰ�Ӿ���ϵ�� ��λ��������
	* 
	* ----------------------------------------------------------------------------
	* 
	* 13-����ֱ�߻����߶�׷��
	* ����ԭ�� void setLineArithmetic(bool status);
	* ����true  ֱ��׷���㷨 
	* ����false �߶�׷���㷨
	* 
	* ----------------------------------------------------------------------------
	*
	* 14-�����ٶ�Ӱ��ϵ��	
	* ����ԭ�� void setSpeedAffactedFactor(uint8_t factor);
  * ������� �ٶ�Ӱ��ϵ��	 ��λ��������
	*
	* ----------------------------------------------------------------------------
	*
	* 15-�����ٶȹ̶�ϵ��
	* ����ԭ�� void setSpeedFixedFactor(uint8_t factor);
  * ������� �ٶȹ̶�ϵ�� ��λ��������
	* ----------------------------------------------------------------------------
	*
	* ###### WARNING ######
	* �벻Ҫ���ó�������API�����API
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
#include <arm_math.h>			//���ʹ����Ӳ�����㹦�ܣ��Ͱ���Ӳ��������ѧ��
#else
#include <math.h>					//���ûʹ��Ӳ�����㹦�ܣ��Ͱ�����ͨ��ѧ��
#endif

/* Private Define  ------------------------------------------------------------ */

#define FE_WGS84	0.00335281066474748049		//WGS84ƫ����
#define RE_WGS84	6378137.0									//WGS84����뾶
#define e					0.006694379990141316461		//ת������

#define Degree2Rad	0.01745329251994327813	//�Ƕ�ת����
#define Rad2Degree	57.29577951308237971		//����ת�Ƕ�

#define CarWheelBearingDistance 3.71f				//���

#define MaxSwerveAngle 30.0f								//���ת���

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
* @brief ����һ����ά�ĵ�
*/
struct SPoint3d
{
	float x;
	float y;
	float z;
};

/**
* @brief ����һ��˫������Ľ�� ��Ϊ��ʼ��·���ĵ�
*/
struct SInitNode
{
	struct SInitNode *prev;			//ǰһ�����ĵ�ַ
	struct SPoint3d point;	//·����
	struct SInitNode *next;			//��һ�����ĵ�ַ
};

/**
* @brief ����һ��������˫��������Ϣ�Ľṹ�� ��Ϊ��ʼ��·��
*/
struct SInitList
{
	struct SInitNode *head;			//ͷ���
	struct SInitNode *tail;			//β���
	uint16_t nSize;					//����
};

/**
* @brief ����һ���򵥵Ŀɱ�����Ľṹ�� ��Ϊ�ο�·��
*/
struct SReferenceList
{
	struct SPoint3d *point;	//·����
	uint16_t nSize;					//����
};

/**
* @brief ��������
*/
struct SCommmand
{
	int nDirection;
	int nWheelSteeringAngle;
};

/**
* @brief ����·��׷�ٵĲ���״̬
*/
struct STrackStatus
{
	float fLateralError;
	float fHeadingAngleError;
};

/**
* @brief ֱ�ߵ���Ϣ
*/
struct SLineInfo
{
	float A;
	float B;
	float C;
};

/* Private Enum -------------------------------------------------------------- */

/**
* @brief ���巽��
*/
enum DIRECTION_STATUS
{
	DIRECTION_STATUS_MID = 0x00,		//ֱ��
	DIRECTION_STATUS_LEFT,					//��ת
	DIRECTION_STATUS_RIGHT					//��ת
};

/**
* @brief ����׷��״̬
*/
enum TRACKING_STATUS
{
	TRACKING_STATUS_NORMAL = 0x00,			//����
	TRACKING_STATUS_NO_ENOUGH_POINTS,
	TRACKING_STATUS_NO_MEMORY,
	TRACKING_STATUS_TO_THE_END
};

/**
* @brief ����ָ��״̬
*/

enum COMMAND_STATUS
{
	COMMAND_STATUS_NO_COMMAND = 0x00,						//û��ָ��

	COMMAND_STATUS_IS_RUN,											//ǿ�ƿ�ʼ����ǿ��ֹͣ
	COMMAND_STATUS_IS_RECORD,                   //�Ƿ��¼���� ����ǵĻ� ��һ�����ݻᱻ��Ϊ����ԭ��
	COMMAND_STATUS_IS_AUTORUN,                  //�Ƿ��Զ����� ����ǵĻ� ��һ����ᱻ��Ϊƽ��·���Ĳο���
	COMMAND_STATUS_IS_INIT,                     //�Ƿ��ʼ��·�� ����ǵĻ� ��ʼ��ʼ��·���������ǵ�֮ǰ�������
	COMMAND_STATUS_IS_ADJUST,                   //�Ƿ����ҽ���
	COMMAND_STATUS_IS_LINE,											//�Ƿ�ֱ��
	COMMAND_STATUS_IS_TURN,											//�Ƿ�ת��
	COMMAND_STATUS_IS_SLAVE,										//�Ƿ���λ������

	COMMAND_STATUS_SET_SAMPLINGPERIOD,					//���ò�������
	COMMAND_STATUS_SET_SIGHTDISTANCE_FACTOR,		//����ǰ�Ӿ���ϵ��
	COMMAND_STATUS_SET_KP,                     	//����PID����P
  COMMAND_STATUS_SET_KI,                     	//����PID����I
  COMMAND_STATUS_SET_KD,                     	//����PID����D
	
	COMMAND_STATUS_AUTOSET_MIDANGLE,            //�Զ�������λ��
	COMMAND_STATUS_AUTOSET_PWMPID,              //�Զ����ýǶȸ����pidֵ
	
	COMMAND_STATUS_AFFECTSPEED,									//�����ٶ�Ӱ������
	COMMAND_STATUS_FIXEDSPEED,									//�����ٶȹ̶�����

	COMMAND_STATUS_AUTORUN_STATUS               //�Զ����е�״̬
};

/* Private Function ---------------------------------------------------------- */
//������������

//��������������ƽ����
float GetArithmeticSquareRoot(float n1, float n2);
//���һ���㵽�ڶ�����ĽǶ�
float GetPointAngle(point3d p1, point3d p2);
//ͨ���������ֱ�߲���
void CountLinePara(point3d p1, point3d p2, lineInfo *l);
//����㵽ֱ�ߵľ���
float CountPointLineDistance(point3d p, lineInfo l);

//��·���Ļ�������

//��ʼ����ʼ·���ĵ�ַ����
bool configureInitPathMemory(void);				
//��ʼ���ο�·���ĵ�ַ����
bool configureReferencePathMemory(void);		
//��ӳ�ʼ·��
bool addInitPoint(point3d p);
//��Ӳο�·��
void AddReferencePoint(float x, float y, float z);
//����ο�·��
void clearReferencePath(void);					
//�����ʼ·��
void clearInitPath(void);		

//��̬��������

//ת����ʼ�����
float changeCourseAngle(float angle);
//��������ϵ��ת��
void JWG2ENU(double j, double w, double g, point3d *enu);

//���ɲο�·��
uint8_t GenerateReferencePath(point3d p, float headingAngle);
//purepursuitģ���Զ�׷��
uint8_t autoRunPurePursuit(point3d p, float speed, float headingAngle, command* info, trackStatus* status);

//׷���㷨��ز�������

//����ֱ��or�߶�׷��
void setLineArithmetic(bool arithmetic);
//�����Ƿ��һ���� ������ʼ������ԭ��
void setFirstPoint(bool flag);
//�����Ƿ�ƽ��·��
void setFirstMove(bool status);
//���ò���Ƶ��
void setSamplingPeriod(uint8_t period);
//�����ٶ�Ӱ��ϵ��	
void setSpeedAffactedFactor(uint8_t factor);
//�����ٶȹ̶�ϵ��
void setSpeedFixedFactor(uint8_t factor);
//����ǰ�Ӿ���ϵ��
void setLookAheadDistanceFactor(uint8_t factor);


#endif	// END OF FILE
