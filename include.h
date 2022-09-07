#ifndef __INCLUDE_H
#define __INCLUDE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#if (__FPU_PRESENT == 1)
//如果使能了硬件浮点功能，就包括硬件浮点数学库
#include <arm_math.h>
#else
//如果没使能硬件浮点功能，就包括普通数学库
#include <math.h>
#endif

// WGS84偏心率
#define FE_WGS84 0.00335281066474748049
// WGS84地球半径
#define RE_WGS84 6378137.0
// 转换常数
#define e 0.006694379990141316461
// 角度转弧度
#define Degree2Rad 0.01745329251994327813
#define Rad2Degree 57.29577951308237971

// 弧度转角度
#define CarWheelBearingDistance 3.71f //轴距
// 最大转向角
#define MaxSwerveAngle 30.0f

    enum STATUS
    {
        OK = 0x00,
        NULL_POINT_ERROR,

    };

    typedef struct SPoint3d point3d;
    typedef struct SPoint3df point3df;

    /**
     * @brief 定义一个三维的点 float类型
     */
    struct SPoint3df
    {
        float x;
        float y;
        float z;
    };

    /**
     * @brief 定义一个三维的点 double类型
     */
    struct SPoint3d
    {
        double x;
        double y;
        double z;
    };

#ifdef __cplusplus
}
#endif

#endif