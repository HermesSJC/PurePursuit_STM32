#ifndef __COORDINATE_H
#define __COORDINATE_H

#ifdef __cplusplus
extern "C"
{
#endif

    /* Private Include ------------------------------------------------------------ */
#include "include.h"

    /* Private Enum -------------------------------------------------------------- */
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

    /* Private Function ---------------------------------------------------------- */
    //转换
    int8_t change(point3d in, point3d *out);

    void setChangeMode(uint8_t mode);

    uint8_t getChangeMode(void);

#ifdef __cplusplus
}
#endif

#endif // END OF FILE