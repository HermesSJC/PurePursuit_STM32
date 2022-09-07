#ifndef __PROPERTIES_H
#define __PROPERTIES_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "include.h"

    void setFirstPoint(void);

    void cancelFirstPoint(void);

    bool isFirstPoint(void);

    void setChangeModel(uint8_t);

    uint8_t getChangeModel(void);

#ifdef __cplusplus
}
#endif

#endif