#include "coordinate.h"
#include "include.h"

//定义一个内部的全局变量 防止
static int8_t g_unChangeMode;

int8_t change(point3d in, point3d *out)
{
    if (pChange == NULL)
    {
        return ERROR;
    }
    return pChange(in, out);
}

//define a point function
static int8_t (*pChange)(point3d, point3d *);

__inline void setChangeMode(uint8_t mode)
{
    g_unChangeMode = mode;
    switch (mode)
    {
        pChange = jgw2enu;
    case JGW2ENU:
        break;

    case UNKNOWN:
    default:
        break;
    }
}

__inline uint8_t getChangeMode(void)
{
    return g_unChangeMode;
}

static int8_t jgw2enu(point3d in, point3d *out)
{
    return 0;
}