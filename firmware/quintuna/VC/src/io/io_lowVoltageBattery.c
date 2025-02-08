#include "io_lowVoltageBattery.h"
#include "hw_i2c.h"
#include "hw_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

const uint8_t  BQ76922_I2C_ADDR      = 0x10;   // 7-bit I2C address
const uint16_t CMD_DASTATUS6         = 0x0076; // Subcommand for accumulated charge
const uint8_t  COMMAND_ADDRESS       = 0x3E;
const uint8_t  REG_SUBCOMMAND_LSB    = 0x3E;
const uint8_t  REG_SUBCOMMAND_MSB    = 0x3F;
const uint8_t  REG_DATA_BUFFER       = 0x40;
const uint8_t  REG_CHECKSUM          = 0x60;
const uint8_t  REG_RESPONSE_LENGTH   = 0x61;
const uint16_t CELL0_VOLTAGE_COMMAND = 0x1514;
const uint16_t CELL1_VOLTAGE_COMMAND = 0x1716;
const uint16_t CELL2_VOLTAGE_COMMAND = 0x1B1A;
const uint16_t CELL4_VOLTAGE_COMMAND = 0x1D1C;
const uint16_t STACK_VOLTAGE_COMMAND = 0x3534;

#define R_SENSE                 2.0f     // Sense resistor in mΩ
#define Q_FULL                  11200.0f // Battery full charge capacity in mAh
#define SECONDS_PER_HOUR        3600.0f  // Convert charge from Coulombs to mAh
#define PERCENTAGE_FACTOR       100.0f   // Convert SOC to percentage
#define ADC_CALIBRATION_FACTOR  7.4768f  // Derived from ADC gain and scaling

extern I2C_HandleTypeDef hi2c1; // Declaration of i2c bus
static I2cInterface lvBatMon = { &hi2c1, BQ76922_I2C_ADDR, 100 };

osSemaphoreId_t bat_mtr_sem; // Declaration of interrupt handling semaphore

/**
 * Helper function to write commands to communicate with BQ76922 over i2c
 * 
 * @param subcommand in 2 byte hexadecimal
 * @return true if the command was sucessfully written and false otherwise
 */
bool io_lowVoltageBattery_writeSubcommand(uint16_t subcommand)
{
    if (!hw_i2c_isTargetReady(&lvBatMon)){
        return false;
    }
    
    uint8_t data[2] = { subcommand & 0xFF, (subcommand >> 8) & 0xFF };
    if (!hw_i2c_memWrite(&hi2c1, REG_SUBCOMMAND_LSB, data, 2))
    {
        return false;
    }

    return true;
}

/**
 * Helper function to read the length of a response from BQ76922 in bytes
 * 
 * @return the number of bytes contained within a response
 */
uint8_t io_lowVoltageBattery_readResponseLength()
{
    uint8_t responseLength;

    if (!hw_i2c_memRead(&lvBatMon, REG_RESPONSE_LENGTH, &responseLength, sizeof(responseLength)))
    {
        return -1;
    }

    return responseLength;
}

/**
 * Helper function to read the state-of-charge (SOC) of the LV battery from BQ76922
 * 
 * @return the SOC as a percentage of full charge
 */
float io_lowVoltageBattery_readSOC()
{
    uint32_t charge;
    uint16_t time;

    uint8_t responseLen = io_lowVoltageBattery_readResponseLength();

    if (responseLen != 6)
    {
        return -1;
    }

    uint8_t buffer[6];

    if (!hw_i2c_memRead(&hi2c1, REG_DATA_BUFFER, buffer, 6))
    {
        return -1;
    }

    uint8_t checksum;
    if (!hw_i2c_memRead(&hi2c1, REG_CHECKSUM, &checksum, 1))
    {
        return -1;
    }

    uint8_t calculated_checksum = (CMD_DASTATUS6 & 0xFF) + (CMD_DASTATUS6 >> 8) + responseLen;
    for (int i = 0; i < responseLen; i++)
    {
        calculated_checksum += buffer[i];
    }
    calculated_checksum = ~calculated_checksum; // Invert bits

    if (calculated_checksum != checksum)
    {
        return -1;
    }

    charge = (buffer[0] | (buffer[1] << 8) | (buffer[2] << 16));
    time   = (buffer[3] | (buffer[4] << 8));

    float CC_GAIN    = ADC_CALIBRATION_FACTOR / R_SENSE;
    float charge_mAh = (charge * CC_GAIN) / SECONDS_PER_HOUR;

    return (charge_mAh / Q_FULL) * PERCENTAGE_FACTOR;
}

/**
 * Initialization function to define semaphore
 * 
 * @return true if the semaphore was sucessfully created and false otherwise
 */
bool io_lowVoltageBattery_init()
{
    bat_mtr_sem = osSemaphoreNew(1, 0, NULL); 

    if (bat_mtr_sem == NULL) {
        return false;
    }

    return true;
}

/**
 * Main driver function to execute i2c communication protocol to get the state-of-charge (SOC)
 * 
 * @return the SOC as a percentage of full charge
 */
float io_lowVoltageBattery_getSOC()
{
    if (!io_lowVoltageBattery_writeSubcommand(CMD_DASTATUS6))
    {
        return -1;
    }

    osSemaphoreAcquire(bat_mtr_sem, osWaitForever);

    return io_lowVoltageBattery_readSOC();
}
