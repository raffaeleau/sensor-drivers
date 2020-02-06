/*******************************************************************************
*
*  Filename:       mcp342x.h
*  Revised:        6/03/2019
*  Revision:       0.1
*  Programmer(s):  RI
*
*  Description:    MCP342X Analog-to-Digital Converter Driver
*
*******************************************************************************/

#ifndef MCP342X_H
#define MCP342X_H

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
// MCP3421, MCP3425 & MCP3426 are factory programmed for any of 0x68 through 0x6F
#define MCP342X_DEFAULT_ADDRESS     0x68

#define MCP342X_RDY_BIT             0x80

typedef enum MCP342X_ConversionMode_t
{
    MODE_ONESHOT    = 0x00,
    MODE_CONTINUOUS = 0x10,
    MODE_MASK       = 0x30
} MCP342X_ConversionMode_t;

typedef enum MCP342X_Resolution_t
{
    RES_12_BIT      = 0x00,
    RES_14_BIT      = 0x04,
    RES_16_BIT      = 0x08,
    RES_18_BIT      = 0x0C,
    RES_MASK        = 0x0C
} MCP342X_Resolution_t;

typedef enum MCP342X_PGAGain_t
{
    PGA_GAIN_1X     = 0x00,
    PGA_GAIN_2X     = 0x01,
    PGA_GAIN_4X     = 0x02,
    PGA_GAIN_8X     = 0x03,
    PGA_GAIN_MASK   = 0x03
} MCP342X_PGAGain_t;

typedef enum MCP342X_Channel_t
{
    CHAN_1          = 0x00,
    CHAN_2          = 0x02,
    CHAN_3          = 0x04,
    CHAN_4          = 0x06,
    CHAN_MASK       = 0x06,
} MCP342X_Channel_t;

// MCP342x object
typedef struct
{
    struct
    {
        MCP342X_ConversionMode_t conversionMode;
        MCP342X_Resolution_t resolution;
        MCP342X_PGAGain_t gain;
        MCP342X_Channel_t channel;
    } config;

} MCP342x_t;

/*******************************************************************************
*   PUBLIC FUNCTION PROTOTYPES
*******************************************************************************/

bool mcp342x_deviceTest(uint8_t addr);
bool mcp342x_writeConfiguration(MCP342x_t *handle);
bool mcp342x_read(MCP342x_t *handle, long *value, uint8_t  *status);
bool mcp342x_readVolts(MCP342x_t *handle, double *volts, uint8_t  *status);

/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MCP342X_H */

