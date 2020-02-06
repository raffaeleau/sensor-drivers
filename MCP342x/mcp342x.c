/*******************************************************************************
*
*  Filename:       mcp342x.c
*  Revised:        6/03/2019
*  Revision:       0.1
*  Programmer(s):  RI
*
*  Description:    MCP342X Analog-to-Digital Converter Driver
*                  Compatible with MCP3421/2/3/4
*
*******************************************************************************/
/*******************************************************************************
*   INCLUDES
*******************************************************************************/

#include "mcp342x.h"
#include "math.h"
#include <Interface/bsp_i2c.h>

/*******************************************************************************
*   PRIVATE FUNCTION PROTOTYPES
*******************************************************************************/

static bool readValue(long *value, uint8_t *status);
static long resolveValue(long outputCode, uint8_t config);
static MCP342X_Resolution_t getResolution(uint8_t config);
static uint32_t getSampleTimeMs(MCP342X_Resolution_t res);
static double getLsbVolts(MCP342X_Resolution_t res);

/*******************************************************************************
*   PRIVATE FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
 * @fn      readValue
 *
 * @brief
 *
 * @param
 *
 * @return
 */
static bool readValue(long *value, uint8_t *status)
{
    uint8_t data[4] = "";
    uint8_t bytes = 0;
    bool result;

    // Read data until the conversion is ready
    do
    {
        /*
         * 18-bit resolution: the 4th byte contains the config.
         * 12/14/16-bit resolution: the 3rd byte contains the config.
         */
        result = bspI2cRead(data, 4);
        bytes = (getResolution(data[3]) == RES_18_BIT) ? 3 : 2;
        *status = data[bytes];

    } while ((*status & MCP342X_RDY_BIT) && result);

    if (!result)
    {
        return false;
    }

    // Extract the output code
    uint8_t i;
    long output_code = 0;
    for (i = 0; i < bytes; i++)
    {
        output_code = (output_code << 8) | data[i];
    }

    // Resolve ADC value
    *value = resolveValue(output_code, *status);

    return result;
}

/******************************************************************************
 * @fn      resolveValue
 *
 * @brief
 *
 * @param
 *
 * @return
 */
static long resolveValue(long outputCode, uint8_t config)
{
    long msb_mask = ~(((long)1 << (12 + (getResolution(config) >> 1))) - 1);
    long sign_bit = (long)1 << (11 + (getResolution(config) >> 1));

    // Remove unwanted repeated MSB bits for negative values
    return (outputCode & sign_bit) ? outputCode |= msb_mask : outputCode;
}

/******************************************************************************
 * @fn      getResolution
 *
 * @brief
 *
 * @param
 *
 * @return
 */
static MCP342X_Resolution_t getResolution(uint8_t config)
{
    return (MCP342X_Resolution_t)(config & RES_MASK);
}

/******************************************************************************
 * @fn      getSampleTimeMs
 *
 * @brief
 *
 * @param
 *
 * @return
 */
static uint32_t getSampleTimeMs(MCP342X_Resolution_t res)
{
    /*
     *  Resolution      SPS
     *  12-bit          240 (~6 ms)
     *  14-bit          60 (~20 ms)
     *  16-bit          15 (~70 ms)
     *  18-bit          3.75 (~270 ms)
     */
    uint32_t sample_time_map[] = {6, 20, 70, 270};
    return sample_time_map[(res >> 2)];
}

/******************************************************************************
 * @fn      getLsbVolts
 *
 * @brief
 *
 * @param
 *
 * @return
 */
static double getLsbVolts(MCP342X_Resolution_t res)
{
    /*
     *  Resolution      LSB
     *  12-bit          1 mV
     *  14-bit          250 uV
     *  16-bit          62.5 uV
     *  18-bit          15.625 uV
     */
    double lsb_map[] = {0.001, 0.00025, 0.0000625, 0.000015625};
    return lsb_map[(res >> 2)];
}

/*******************************************************************************
*   PUBLIC FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
 * @fn      mcp342x_deviceTest
 *
 * @brief   Device communication test
 *
 * @param   Address of device on I2C bus
 *
 * @return  true on receiving an ACK response
 */
bool mcp342x_deviceTest(uint8_t addr)
{
    // Acquire I2C resource if locked
    if (!bspI2cSelect(addr))
    {
        bspI2cDeselect();   // release to select new address
        bspI2cSelect(addr);
    }

    // Perform a read and expect an ACK response
    uint8_t data;
    return bspI2cRead(&data, 1);
}

/******************************************************************************
 * @fn      mcp342x_writeConfiguration
 *
 * @brief
 *
 * @param
 *
 * @return  true if successful
 */
bool mcp342x_writeConfiguration(MCP342x_t *handle)
{
    // Setup the configuration byte
    uint8_t config_byte = 0;
    config_byte |= (MODE_MASK & handle->config.conversionMode);
    config_byte |= (RES_MASK & handle->config.resolution);
    config_byte |= (PGA_GAIN_MASK & handle->config.gain);
    config_byte |= (CHAN_MASK & handle->config.channel);

    // Begin conversion for one-shot mode, no effect for continuous mode
    config_byte |= MCP342X_RDY_BIT;

    // Write to the device
    return bspI2cWriteSingle(config_byte);
}

/******************************************************************************
 * @fn      mcp342x_read
 *
 * @brief
 *
 * @param
 *
 * @return  true if successful
 */
bool mcp342x_read(MCP342x_t *handle, long *value, uint8_t  *status)
{
    if (handle->config.conversionMode == MODE_ONESHOT)
    {
        // Write the RDY bit to make a conversion
        mcp342x_writeConfiguration(handle);

        // Wait the sample time
        bspI2cDelayMs(getSampleTimeMs(handle->config.resolution));
    }

    // Read the conversion
    return readValue(value, status);
}

/******************************************************************************
 * @fn      mcp342x_readVolts
 *
 * @brief
 *
 * @param
 *
 * @return  true if successful
 */
bool mcp342x_readVolts(MCP342x_t *handle, double *volts, uint8_t  *status)
{
    long value = 0;
    bool result = mcp342x_read(handle, &value, status);

    if (result)
    {
        double lsb = getLsbVolts(handle->config.resolution);
        double gain = pow(2.0, handle->config.gain);
        *volts = (double)value * (lsb / gain);
    }

    return result;
}

