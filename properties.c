#include "properties.h"

static bool g_bFirstPoint;

static uint8_t g_unChangeMode;

__inline void setFirstPoint(void)
{
    g_bFirstPoint = true;
}

__inline void cancelFirstPoint(void)
{
    g_bFirstPoint = false;
}

__inline bool isFirstPoint(void)
{
    return g_bFirstPoint;
}

__inline void setChangeModel(uint8_t mode)
{
    g_unChangeMode = mode;
}

__inline uint8_t getChangeModel(void)
{
    return g_unChangeMode;
}