/*******************************************************************************
*
*  Filename:       mpm3808.h
*  Revised:        7/03/2019
*  Revision:       0.1
*  Programmer(s):  RI
*
*  Description:    MPM3808 Digital Output Pressure Transducer Driver
*
*******************************************************************************/

#ifndef MPM3808_H
#define MPM3808_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
*   INCLUDES
*******************************************************************************/

#include "stdbool.h"
#include "stdint.h"

/******************************************************************************
*   TYPE DEFINITIONS
******************************************************************************/

// I2C Address of device
#define MPM3808_DEFAULT_ADDRESS     0x28

typedef enum
{
    OUTPUT_10_90,   // 10% ~ 90% (1638 ~ 14746)
    OUTPUT_5_95     // 5% ~ 95% (819 ~ 15563)
} MPM3808_Output_t;

/*******************************************************************************
*   PUBLIC FUNCTION PROTOTYPES
*******************************************************************************/

bool mpm3808_readCounts(uint16_t *cPress, uint16_t *cTemp);
double mpm3808_convertTemperature(uint16_t cTemp);
double mpm3808_convertPressure(uint16_t cPress, MPM3808_Output_t opType, double rangeMin, double rangeMax);

/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MPM3808_H */

