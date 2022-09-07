#include "coordinate.h"
#include "properties.h"

//定义一个内部的函数指针 后续利用该指针实现多态
static uint8_t (*pChange)(point3d, point3d *);

//内部的转换函数
static uint8_t jgw2enu(point3d in, point3d *out)
{
    //坐标转换的系数矩阵
    static double_t dTransforMatrix[3][3] = {0.0};
    // ecef0的坐标系的数值
    static double_t ecefx0 = 0.0, ecefy0 = 0.0, ecefz0 = 0.0;

    //角度的系数
    double_t sinp = 0, cosp = 0.0, sinl = 0.0, cosl = 0.0;
    //坐标转换系数
    double_t v = 0.0;
    // ecef的坐标系的数值
    double_t ecefx = 0.0, ecefy = 0.0, ecefz = 0.0;
    //角度转弧度
    double_t j = in.x * Degree2Rad;
    double_t w = in.y * Degree2Rad;
    double_t g = in.z;
    //带入sin和cos
    sinp = sin(w);
    cosp = cos(w);
    sinl = sin(j);
    cosl = cos(j);
    if (isFirstPoint())
    {
        //只设置一次
        cancelFirstPoint();

        //设置转换矩阵的系数
        dTransforMatrix[0][0] = -sinl;
        dTransforMatrix[0][1] = cosl;
        dTransforMatrix[0][2] = 0.0;
        dTransforMatrix[1][0] = -sinp * cosl;
        dTransforMatrix[1][1] = -sinp * sinl;
        dTransforMatrix[1][2] = cosp;
        dTransforMatrix[2][0] = cosp * cosl;
        dTransforMatrix[2][1] = cosp * sinl;
        dTransforMatrix[2][2] = sinp;

        //计算ecef0的值 单位从米转换为米 类型从double转doubel
        v = RE_WGS84 / sqrt(1.0 - e * sinp * sinp);
        ecefx0 = (v + g) * cosp * cosl;
        ecefy0 = (v + g) * cosp * sinl;
        ecefz0 = (v * (1.0 - e) + g) * sinp;

        //第一个坐标的点肯定是(0,0,0)
        out->x = 0.0;
        out->y = 0.0;
        out->z = 0.0;
    }
    else
    {
        // jwg转ecef 单位从米转换为米 类型从double转doubel
        v = RE_WGS84 / sqrt(1.0 - e * sinp * sinp);
        ecefx = (v + g) * cosp * cosl - ecefx0;
        ecefy = (v + g) * cosp * sinl - ecefy0;
        ecefz = (v * (1.0 - e) + g) * sinp - ecefz0;

        // ecef转enu 单位从米转换为米，类型从double转float
        out->x = dTransforMatrix[0][0] * ecefx + dTransforMatrix[0][1] * ecefy + dTransforMatrix[0][2] * ecefz;
        out->y = dTransforMatrix[1][0] * ecefx + dTransforMatrix[1][1] * ecefy + dTransforMatrix[1][2] * ecefz;
        out->z = dTransforMatrix[2][0] * ecefx + dTransforMatrix[2][1] * ecefy + dTransforMatrix[2][2] * ecefz;
    }
    return OK;
}

uint8_t change(point3d in, point3d *out)
{
    // 如果没指向某个地址 就返回错误
    if (pChange == NULL)
    {
        return NULL_POINT_ERROR;
    }
    return pChange(in, out);
}

void setChangeMode(uint8_t mode)
{
    setChangeMode(mode);
    switch (mode)
    {
    case JGW2ENU:
        pChange = jgw2enu;
        break;

    case UNKNOWN:
    default:
        pChange = NULL;
        break;
    }
}

__inline point3df double2float(point3d in)
{
    point3df out;
    out.x = (float_t)(in.x);
    out.y = (float_t)(in.y);
    out.z = (float_t)(in.z);
    return out;
}

__inline point3d float2double(point3df in)
{
    point3d out;
    out.x = (double_t)(in.x);
    out.y = (double_t)(in.y);
    out.z = (double_t)(in.z);
    return out;
}