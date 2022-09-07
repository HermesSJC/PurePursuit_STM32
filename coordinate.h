#ifndef __COORDINATE_H
#define __COORDINATE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "include.h"

    /**
     * @brief 定义转换的模式
     */
    enum COORDINATE_CHANGE_ENUM
    {
        //未知的
        UNKNOWN = 0x00,
        //经纬高 到 东北天
        JGW2ENU = 0x01
    };

    uint8_t change(point3d, point3d *);

    void setChangeMode(uint8_t mode);

    point3df double2float(point3d);

    point3d float2double(point3df);

#ifdef __cplusplus
}
#endif

#endif // END OF FILE