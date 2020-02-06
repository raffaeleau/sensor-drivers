/******************************************************************************
*
*  Filename:        sht2x.c
*  Revised:         27/05/2016
*  Revision:        0.4
*  Programmer(s):   RI
*
*  Description:     Sensirion SHT2x sensor layer. Functions for sensor access
*
******************************************************************************/

/******************************************************************************
*   INCLUDE FILES
******************************************************************************/

#include "sht2x.h"
#include "bsp_i2c.h"    /* Board support package */

/******************************************************************************
*   PRIVATE CONSTANTS
******************************************************************************/

/* CRC Generator Polynomial: P(x)=x^8+x^5+x^4+1 = 100110001 */
static const uint16_t Polynomial = 0x0131;

/******************************************************************************
*   PRIVATE FUNCTION PROTOTYPES
******************************************************************************/

static bool sht2x_readUserRegister(uint8_t *pRegisterValue);
static bool sht2x_writeUserRegister(uint8_t *pRegisterValue);
static bool sht2x_checkCRC(uint8_t* pdata, uint8_t nbrOfBytes, uint8_t checksum);

/******************************************************************************
*   PRIVATE FUNCTION DEFINITIONS
******************************************************************************/

/*******************************************************************************
 * @fn          sht2x_readUserRegister
 *
 * @brief       Read the User Register
 *
 *              User Register contains reserved bits.
 *              A read should always preceed a write.
 *              Reserved bits must remain unchanged.
 *
 * @param       pRegisterValue
 *
 * @return      status  true if I2C read succeeded, false if read failed
 */
static bool sht2x_readUserRegister(uint8_t *pRegisterValue)
{
    uint8_t checksum;
    bool status = true;
    uint8_t tx[1] = {USER_REG_R};
    uint8_t rx[2] = "";
    
    /* Send the one data byte and read two data bytes */
    status &= bspI2cWriteRead(tx, 1, rx ,2);
    
    /* Data has been received */
    *pRegisterValue = rx[0];
    checksum = rx[1];
    
    /* Verify checksum */
    status &= sht2x_checkCRC(pRegisterValue, 1, checksum);
    
    return status;
}

/*******************************************************************************
 * @fn          sht2x_writeUserRegister
 *
 * @brief       Write the User Register
 *
 *              User Register contains reserved bits.
 *              A read should always preceed a write.
 *              Reserved bits must remain unchanged.
 *
 * @param       pRegisterValue
 *
 * @return      status  true if I2C read succeeded, false if read failed
 */
static bool sht2x_writeUserRegister(uint8_t *pRegisterValue)
{
    uint8_t tx[2] = {USER_REG_W, *pRegisterValue};
    bool status = false;
    
    /* Send the two data bytes */
    status &= bspI2cWrite(tx, 2);
    
    return status;
}

/*******************************************************************************
 * @fn          sht2x_checkCRC
 *
 * @brief       Check the CRC with the calculated checksum
 *
 * @param       data            data buffer to calculate the crc on
 * @param       nbrOfBytes      number of bytes in the data buffer
 * @param       checksum        checksum to compare crc to
 *
 * @return      true            if crc = checksum
 *              false           if crc mismatch
 */
static bool sht2x_checkCRC(uint8_t* pdata, uint8_t nbrOfBytes, uint8_t checksum)
{
    uint8_t crc = 0;
    uint8_t byteCtr;
    uint8_t bit;
    
    /* Calculates 8-Bit checksum with given polynomial */
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= (pdata[byteCtr]);
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ Polynomial;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    
    if (crc != checksum)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/******************************************************************************
*   PUBLIC FUNCTION DEFINITIONS
******************************************************************************/

/*******************************************************************************
 * @fn          sht2x_softReset
 *
 * @brief       Send the Soft Reset command
 *
 *              This command is used for rebooting the sensor system without
 *              switching the power on and off again.
 *
 *              The Soft Reset takes less than 15ms as per Section 5.5
 *              (page 9) in the SHT25 Datasheet
 *
 * @param       none
 *
 * @return      status  true if I2C read succeeded, false if read failed
 */
bool sht2x_softReset(void)
{
    uint8_t tx[1] = {SOFT_RESET};
    bool status = true;
    
    status &= bspI2cWrite(tx, 1);
    
    /* Allow 20ms delay after issuing the Soft Reset command */
    bspI2cDelayMs(20);
    
    return status;
}

/*******************************************************************************
 * @fn          sht2x_measureHoldMaster
 *
 * @brief       Measure the sensor in Hold Master mode
 *
 *              In Hold Master Mode, the SHT2x pulls downs the SCL line while
 *              measuring to force the I2C master into a wait state.
 *              By releasing the SCL line the sensor indicates that the
 *              internal processing is terminated and that the transmission
 *              may be continued.
 *
 * @param       measureType		type of measurement to perform
 * @param       *pMeasurand		stores raw sensor output
 *
 * @return      status  true if I2C read succeeded, false if read failed
 */
bool sht2x_measureHoldMaster(SHT2xMeasureType_t measureType, uint16_t *pMeasurand)
{
    uint8_t checksum;       /* Checksum */
    uint8_t tx[1];          /* Data array for transmitted data */
    uint8_t rx[3];          /* Data array for received data */
    bool status = true;
    
    /* Prepare measurement command */
    if (measureType == HUMIDITY)
    {
        tx[0] = TRIG_RH_MEASUREMENT_HM;
    }
    else if (measureType == TEMP)
    {
        tx[0] = TRIG_T_MEASUREMENT_HM;
    }
    
    /* Send one data byte and receive three data bytes */
    status &= bspI2cWriteRead(tx, 1, rx ,3);
    
    *pMeasurand = (uint16_t)(rx[0] << 8);
    *pMeasurand += (uint16_t)(rx[1] & 0x00FF);
    checksum = rx[2];
    
    /* Verify checksum */
    status &= sht2x_checkCRC(rx, 2, checksum);
    
    return status;
}

/*******************************************************************************
 * @fn          sht2x_measureRelativeHumidity
 *
 * @brief       Measure Relative Humidity
 *
 *              If successfull, sht2x_calcRelativeHumidity() can be called
 *
 * @param       *pMeasurand     stores raw sensor output
 *
 * @return      true            if I2C read succeeded
 *              false           if I2C read failed
 */
bool sht2x_measureRelativeHumidity(uint16_t *pMeasurand)
{
    return sht2x_measureHoldMaster(HUMIDITY, pMeasurand);
}

/*******************************************************************************
 * @fn          sht2x_measureTemperature
 *
 * @brief       Measure Temperature
 *
 *              If successfull, sht2x_calcTemperatureC() can be called
 *
 * @param       *pMeasurand     stores raw sensor output
 *
 * @return      true            if I2C read succeeded
 *              false           if I2C read failed
 */
bool sht2x_measureTemperature(uint16_t *pMeasurand)
{
    return sht2x_measureHoldMaster(TEMP, pMeasurand);
}

/*******************************************************************************
 * @fn          sht2x_calcRelativeHumidity
 *
 * @brief       Calculate Relative Humidity from the raw sensor output
 *
 *              The status bits, the last bits of the LSB must be set to '0'
 *              before calculating physical values
 *
 * @param       rawRH
 *
 * @return      humidity (%RH)
 */
double sht2x_calcRelativeHumidity(uint16_t rawRH)
{
    double humidity;
    
    rawRH &= ~0x0003;       /* Clear bits [1..0] (status bits) */
    
    /* Calculate relative humidity [%RH], RH = -6 + 125 * SRH/2^16 */
    humidity = -6.0 + ((125.0/65536.0) * (double)rawRH);
    
    /* At saturation, its possible for the sensor to read above 100% RH */
    if (humidity > 100.0)
    {
        humidity = 100.0;
    }
    
    return humidity;
}

/*******************************************************************************
 * @fn          sht2x_calcTemperatureC
 *
 * @brief       Calculate Temperature in degrees Celcius from the raw sensor
 *              output
 *
 *              The status bits, the last bits of the LSB must be set to '0'
 *              before calculating physical values
 *
 * @param       rawTemp
 *
 * @return      temperatureC (degC)
 */
double sht2x_calcTemperatureC(uint16_t rawTemp)
{
    double temperatureC;
    
    rawTemp &= ~0x0003;     /* Clear bits [1..0] (status bits) */
    
    /* Calculate temperature [�C], T = -46.85 + 175.72 * ST/2^16 */
    temperatureC= -46.85 + ((175.72/65536.0) * (double)rawTemp);
    return temperatureC;
}

/*******************************************************************************
 * @fn          sht2x_getRelativeHumidity
 *
 * @brief       Get Relative Humidity in %RH
 *
 *              Combines both the I2C measurement and conversion from raw
 *              RH counts to relative humidity in %RH
 *
 * @param       *pRelHum    store the calculated Relative Humdity
 *
 * @return      true        if I2C read succeeded
 *              false       if I2C read failed
 */
bool sht2x_getRelativeHumidity(double *pRelHum)
{
    uint16_t rawRHCounts = 0;
    
    if (sht2x_measureRelativeHumidity(&rawRHCounts))
    {
        *pRelHum = sht2x_calcRelativeHumidity(rawRHCounts);
        return true;
    }
    
    return false;
}

/*******************************************************************************
 * @fn          sht2x_getTemperatureC
 *
 * @brief       Get Temperature in degrees Celcius
 *
 *              Combines both the I2C measurement and conversion from raw
 *              Temp counts to temperature in degrees Celcius
 *
 * @param       *pTemp      store the calculated Temperature in degrees Celcius
 *
 * @return      true        if I2C read succeeded
 *				false       if I2C read failed
 */
bool sht2x_getTemperatureC(double *pTemp)
{
    uint16_t rawTempCounts = 0;
    
    if (sht2x_measureTemperature(&rawTempCounts))
    {
        *pTemp = sht2x_calcTemperatureC(rawTempCounts);
        return true;
    }
    
    return false;
}

/*******************************************************************************
 * @fn          sht2x_configureResolution
 *
 * @brief       Configure the sensor measurement resolution
 *              NOTE: Maximum resolution is 12-bit RH and 14-bit Temp
 *
 * @param       resolution
 *
 * @return      status  true if I2C read succeeded, false if read failed
 */
bool sht2x_configureResolution(SHT2xResolution_t resolution)
{
    uint8_t userRegVal;
    bool status = true;
    
    status &= sht2x_readUserRegister(&userRegVal);  /* Read SHT2x User Register */
    userRegVal |= (SHT2x_RES_MASK & resolution);    /* Set resolution */
    status &= sht2x_writeUserRegister(&userRegVal); /* Write SHT2x User Register */
    
    return status;
}

/*******************************************************************************
 * @fn          sht2x_getSerialNumber
 *
 * @brief       Get the device specific Serial Number
 *
 * @param       *pSerialNumber - buffer to write serial number to
 *              NOTE: must point to a buffer of 8 bytes or more!
 *
 * @return      status  true if I2C read succeeded, false if read failed
 */
bool sht2x_getSerialNumber(uint8_t *pSerialNumber)
{
    uint8_t tx[2] = "";
    uint8_t rx[8] = "";
    bool status = true;
    
    /* First Memory Access */
    tx[0] = SERIAL_NUM_FMA_1;
    tx[1] = SERIAL_NUM_FMA_2;
    status &= bspI2cWriteRead(tx, 2, rx, 8);
    
    /* Verify checksum(s) */
    status &= sht2x_checkCRC(&(rx[0]), 1, rx[1]);
    status &= sht2x_checkCRC(&(rx[2]), 1, rx[3]);
    status &= sht2x_checkCRC(&(rx[4]), 1, rx[5]);
    status &= sht2x_checkCRC(&(rx[6]), 1, rx[7]);
    
    /* Arrange first part in the Serial Number buffer */
    pSerialNumber[5] = rx[6];
    pSerialNumber[4] = rx[4];
    pSerialNumber[3] = rx[2];
    pSerialNumber[2] = rx[0];
    
    /* Second Memory Access */
    tx[0] = SERIAL_NUM_SMA_1;
    tx[1] = SERIAL_NUM_SMA_2;
    status &= bspI2cWriteRead(tx, 2, rx, 6);
    
    /* Verify checksum(s) */
    status &= sht2x_checkCRC(&(rx[0]), 2, rx[2]);
    status &= sht2x_checkCRC(&(rx[3]), 2, rx[5]);
    
    /* Arrange second part in the Serial Number buffer */
    pSerialNumber[7] = rx[1];
    pSerialNumber[6] = rx[0];
    pSerialNumber[1] = rx[4];
    pSerialNumber[0] = rx[3];
    
    return status;
}
