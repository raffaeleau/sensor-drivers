/*******************************************************************************
*
*  Filename:        sht2x.h
*  Revised:         27/05/2016
*  Revision:        0.4
*  Programmer(s):   RI
*
*  Description:     Sensirion SHT2x sensor layer. Definitions of commands,
*                   registers, and functions for sensor access.
*
*******************************************************************************/

#ifndef SHT2X_H
#define SHT2X_H

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
*   INCLUDE FILES
******************************************************************************/

#include "stdbool.h"
#include "stdint.h"

/******************************************************************************
*   TYPE DEFINITIONS
******************************************************************************/

/* Sensor commands */
typedef enum SHT2xCommand_t
{
  TRIG_T_MEASUREMENT_HM     = 0xE3, /* Command trig. temp meas. hold master */
  TRIG_RH_MEASUREMENT_HM    = 0xE5, /* Command trig. humidity meas. hold master */
  TRIG_T_MEASUREMENT_POLL   = 0xF3, /* Command trig. temp meas. no hold master */
  TRIG_RH_MEASUREMENT_POLL  = 0xF5, /* Command trig. humidity meas. no hold master */
  USER_REG_W                = 0xE6, /* Command writing user register */
  USER_REG_R                = 0xE7, /* Command reading user register */
  SOFT_RESET                = 0xFE, /* Command soft reset */
  SERIAL_NUM_FMA_1          = 0xFA, /* Command serial no. firt memory access 1 */
  SERIAL_NUM_FMA_2          = 0x0F, /* Command serial no. firt memory access 2 */
  SERIAL_NUM_SMA_1          = 0xFC, /* Command serial no. second memory access 1 */
  SERIAL_NUM_SMA_2          = 0xC9  /* Command serial no. second memory access 2 */
} SHT2xCommand_t;

/* Measurement signal selection */
typedef enum SHT2xMeasureType_t
{
  HUMIDITY,
  TEMP
} SHT2xMeasureType_t;

/* Measurement resolution settings */
typedef enum SHT2xResolution_t
{
  SHT2x_RES_12_14BIT        = 0x00, /* RH=12bit, T=14bit */
  SHT2x_RES_8_12BIT         = 0x01, /* RH= 8bit, T=12bit */
  SHT2x_RES_10_13BIT        = 0x80, /* RH=10bit, T=13bit */
  SHT2x_RES_11_11BIT        = 0x81, /* RH=11bit, T=11bit */
  SHT2x_RES_MASK            = 0x81  /* Mask for res. bits (7,0) in user reg. */
} SHT2xResolution_t;

/******************************************************************************
*   MACROS
******************************************************************************/

/* SHT2x I2C address */
#define SHT25_I2C_ADDR  (128 >> 1)	/* 0x40, 64d */

/******************************************************************************
*   PUBLIC FUNCTION PROTOTYPES
******************************************************************************/

bool sht2x_softReset(void);
bool sht2x_measureHoldMaster(SHT2xMeasureType_t measureType, uint16_t *pMeasurand);
bool sht2x_measureRelativeHumidity(uint16_t *pMeasurand);
bool sht2x_measureTemperature(uint16_t *pMeasurand);
double sht2x_calcRelativeHumidity(uint16_t rawRH);
double sht2x_calcTemperatureC(uint16_t rawTemp);
bool sht2x_getTemperatureC(double *pTemp);
bool sht2x_getRelativeHumidity(double *pRelHum);
bool sht2x_configureResolution(SHT2xResolution_t resolution);
bool sht2x_getSerialNumber(uint8_t *pSerialNumber);

/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SHT2X_H */
