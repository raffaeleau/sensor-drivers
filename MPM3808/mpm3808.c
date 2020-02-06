/*******************************************************************************
*
*  Filename:       mpm3808.c
*  Revised:        7/03/2019
*  Revision:       0.1
*  Programmer(s):  RI
*
*  Description:    MPM3808 Digital Output Pressure Transducer Driver
*
*******************************************************************************/
/*******************************************************************************
*   INCLUDES
*******************************************************************************/

#include "mpm3808.h"
#include <Interface/bsp_i2c.h>

/*******************************************************************************
*   PRIVATE CONSTANTS
*******************************************************************************/

static const int16_t TempDegCMax = 150;
static const int16_t TempDegCMin = -50;
static const uint16_t TempCountMax = 2047;
static const uint16_t TempCountMin = 0;

/*******************************************************************************
*   PUBLIC FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
 * @fn      mpm3808_readCounts
 *
 * @brief
 *
 * @param
 *
 * @param
 *
 * @return
 */
bool mpm3808_readCounts(uint16_t *cPress, uint16_t *cTemp)
{
    uint8_t data[4] = "";

    bool result = bspI2cRead(data, 4);

    if (result)
    {
        // 14-bit pressure: data[0] {P[13:8]}, data[1] {P[7:0]}
        *cPress = 0x3FFF & ((uint16_t)data[0] << 8 | (uint16_t)data[1]);

        // 11-bit temperature: data[2] {T[10:3]}, data[3] {T[2:0],xxxxx}
        *cTemp = (uint16_t)data[2] << 3 | (uint16_t)data[2] >> 5;
    }

    return result;
}

/******************************************************************************
 * @fn      mpm3808_convertTemperature
 *
 * @brief   Convert raw temperature (digital output) counts to temperature in degC
 *
 * @param   raw temperature counts
 *
 * @return  temperature in degC
 */
double mpm3808_convertTemperature(uint16_t cTemp)
{
    double m = ((double)TempDegCMax - (double)TempDegCMin) / ((double)TempCountMax - (double)TempCountMin);
    return m * (double)cTemp + (double)TempDegCMin;
}

/******************************************************************************
 * @fn      mpm3808_convertPressure
 *
 * @brief   Convert raw pressure (digital output) counts to pressure in kPa
 *
 * @param   raw pressure counts
 *
 * @param   output signal option: (10% ~ 95%) or (5% ~ 95%)
 *
 * @param   minimum kPa range
 *
 * @param   maximum kPa range
 *
 * @return  pressure in kPa
 */
double mpm3808_convertPressure(uint16_t cPress, MPM3808_Output_t opType, double rangeMin, double rangeMax)
{
    double op_min = (opType == OUTPUT_10_90) ? 1638.0 : 819.0;
    double op_max = (opType == OUTPUT_10_90) ? 14746.0 : 15563.0;

    return (rangeMax - rangeMin) * (((double)cPress - op_min)/(op_max - op_min)) + rangeMin;
}
