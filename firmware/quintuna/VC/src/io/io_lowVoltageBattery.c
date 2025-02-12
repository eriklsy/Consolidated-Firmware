#include "io_lowVoltageBattery.h"
#include "hw_i2c.h"
#include "hw_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Register Addresses */
static const uint8_t  BQ76922_I2C_ADDR        = 0x10; 
static const uint8_t  REG_SUBCOMMAND_LSB      = 0x3E;
static const uint8_t  REG_SUBCOMMAND_MSB      = 0x3F;
static const uint8_t  REG_DATA_BUFFER         = 0x40;
static const uint8_t  REG_CHECKSUM            = 0x60;
static const uint8_t  REG_RESPONSE_LENGTH     = 0x61;
static const uint8_t  ALERT_PIN_CONFIG        = 0x56; 
static const uint8_t  ALARM_ENABLE_REG        = 0x66;  
static const uint8_t  ALARM_STATUS_REG        = 0x62;  

/* Voltage Commands */
static const uint16_t CELL0_VOLTAGE_COMMAND   = 0x1514;
static const uint16_t CELL1_VOLTAGE_COMMAND   = 0x1716;
static const uint16_t CELL2_VOLTAGE_COMMAND   = 0x1B1A;
static const uint16_t CELL4_VOLTAGE_COMMAND   = 0x1D1C;
static const uint16_t STACK_VOLTAGE_COMMAND   = 0x3534;

/* Charge Command */

static const uint16_t ACCUMULATED_CHARGE_COMMAND  = 0x0076;   // Subcommand for accumulated charge

/* Hardware Configuration and Conversion Factors */
static const float    R_SENSE                 = 2.0f;     // Sense resistor in mΩ
static const float    Q_FULL                  = 11200.0f; // Battery full charge capacity in mAh
static const float    SECONDS_PER_HOUR        = 3600.0f;  // Seconds per hour (for charge conversion)
static const float    PERCENTAGE_FACTOR       = 100.0f;   // To convert SOC to percentage
static const float    ADC_CALIBRATION_FACTOR  = 7.4768f;  // Derived from ADC gain and scaling

/* Alarm and Alert Settings */
static const uint8_t  ALARM_CLEAR_CMD         = 0x01;
static const uint8_t  ALERT_ACTIVE_LOW_BIT    = 5;        // Bit position for active-low configuration
static const uint8_t  ALERT_PIN_INTERRUPT_CONFIG = 0x02;   // ALERT pin configured as an interrupt output
static const uint8_t  ALARM_ENABLE_VALUE      = 0x82;     // Enable ADC scan alerts

/* I2C Transaction Parameters */
static const uint32_t I2C_TIMEOUT_MS          = 100;      // Timeout for I2C operations

/* Response Sizes */
static const uint8_t  CMD_SUBCOMMAND_SIZE     = 2;        // Two bytes for subcommand
static const uint8_t  RESPONSE_LENGTH_SIZE    = 1;        // One byte for response length and checksum
static const uint8_t  SOC_RESPONSE_LENGTH     = 6;        // Expected response size for SOC command
static const uint8_t  VOLTAGE_RESPONSE_LENGTH = 2;        // Expected response size for voltage command

/* Bit Mask and Shifting Constants */
static const uint8_t  BYTE_MASK               = 0xFF;     // 8-bit mask for lower byte extraction
static const uint8_t  BYTE_SHIFT              = 8;        // Shift amount for upper byte extraction

/* Delay Constants (in OS ticks) */
static const TickType_t SUBCOMMAND_PROCESS_DELAY_MS = 1;  // Delay after issuing a subcommand
static const TickType_t POLL_DELAY_MS               = 1;  // Delay between polls

extern I2C_HandleTypeDef hi2c1;

static I2cInterface lvBatMon = { &hi2c1, BQ76922_I2C_ADDR, I2C_TIMEOUT_MS };

osSemaphoreId_t bat_mtr_sem;

/**
 * @brief Sends a subcommand to the BQ76922 and waits until the device’s
 *        subcommand register clears (indicating that the response is ready).
 *
 * @param cmd The 16-bit subcommand to send.
 * @return true if the subcommand was sent and the response is ready; false otherwise.
 */
static bool io_lowVoltageBattery_send_subcommand(uint16_t cmd)
{
    if (!hw_i2c_isTargetReady(&lvBatMon))
        return false;

    uint8_t data[CMD_SUBCOMMAND_SIZE] = { 
        (uint8_t)(cmd & BYTE_MASK), 
        (uint8_t)((cmd >> BYTE_SHIFT) & BYTE_MASK) 
    };

    if (!hw_i2c_memWrite(&lvBatMon, REG_SUBCOMMAND_LSB, data, CMD_SUBCOMMAND_SIZE))
        return false;

    /* Wait for the subcommand to be processed */
    osDelay(SUBCOMMAND_PROCESS_DELAY_MS);

    uint8_t low = 0, high = 0;
    do {
        osDelay(POLL_DELAY_MS);
        low  = hw_i2c_memRead(&lvBatMon, REG_SUBCOMMAND_LSB);
        high = hw_i2c_memRead(&lvBatMon, REG_SUBCOMMAND_MSB);
    } while ((low | (high << BYTE_SHIFT)) == cmd);

    return true;
}

/**
 * @brief Reads the response from the BQ76922 and validates the checksum.
 *
 * @param cmd         The subcommand that was sent.
 * @param expectedLen The expected number of data bytes.
 * @param buffer      Pointer to a buffer (of at least expectedLen bytes) where the data will be stored.
 *
 * @return true if the response was successfully read and the checksum is valid; false otherwise.
 */
static bool io_lowVoltageBattery_read_response(uint16_t cmd, uint8_t expectedLen, uint8_t *buffer)
{
    uint8_t respLen;
    if (!hw_i2c_memRead(&lvBatMon, REG_RESPONSE_LENGTH, &respLen, RESPONSE_LENGTH_SIZE))
        return false;
    if (respLen != expectedLen)
        return false;

    if (!hw_i2c_memRead(&lvBatMon, REG_DATA_BUFFER, buffer, expectedLen))
        return false;

    uint8_t checksum;
    if (!hw_i2c_memRead(&lvBatMon, REG_CHECKSUM, &checksum, RESPONSE_LENGTH_SIZE))
        return false;

    uint8_t calcChecksum = (uint8_t)(cmd & BYTE_MASK) + (uint8_t)((cmd >> BYTE_SHIFT) & BYTE_MASK) + respLen;
    for (uint8_t i = 0; i < expectedLen; i++)
        calcChecksum += buffer[i];
    calcChecksum = ~calcChecksum;  // Invert bits

    return (calcChecksum == checksum);
}

/**
 * @brief Initializes the low-voltage battery monitoring system.
 *
 * @return true if initialization was successful; false otherwise.
 */
bool io_lowVoltageBattery_init(void)
{
    bat_mtr_sem = osSemaphoreNew(1, 0, NULL);
    if (bat_mtr_sem == NULL)
        return false;

    uint8_t alert_config;
    if (!hw_i2c_memRead(&lvBatMon, ALERT_PIN_CONFIG, &alert_config, RESPONSE_LENGTH_SIZE))
        return false;

    /* Set the active-low bit while preserving other bits */
    alert_config |= (1 << ALERT_ACTIVE_LOW_BIT);
    if (!hw_i2c_memWrite(&lvBatMon, ALERT_PIN_CONFIG, &alert_config, RESPONSE_LENGTH_SIZE))
        return false;

    /* Configure ALERT pin as an interrupt output */
    if (!hw_i2c_memWrite(&lvBatMon, ALERT_PIN_CONFIG, (uint8_t[]){ ALERT_PIN_INTERRUPT_CONFIG }, RESPONSE_LENGTH_SIZE))
        return false;

    /* Enable ADC scan alerts */
    if (!hw_i2c_memWrite(&lvBatMon, ALARM_ENABLE_REG, (uint8_t[]){ ALARM_ENABLE_VALUE }, RESPONSE_LENGTH_SIZE))
        return false;

    return true;
}

/**
 * @brief Gets the battery state-of-charge (SOC) as a percentage.
 *
 * @return SOC percentage on success, or -1.0f on error.
 */
float io_lowVoltageBattery_get_SOC(void)
{
    osSemaphoreAcquire(bat_mtr_sem, osWaitForever);

    if (!send_subcommand(ACCUMULATED_CHARGE_COMMAND))
        return -1.0f;

    uint8_t buffer[SOC_RESPONSE_LENGTH];
    if (!read_response(ACCUMULATED_CHARGE_COMMAND, SOC_RESPONSE_LENGTH, buffer))
        return -1.0f;

    /* Parse the 3-byte charge value (buffer[0]-buffer[2]) */
    uint32_t charge = buffer[0] | (buffer[1] << BYTE_SHIFT) | (buffer[2] << (BYTE_SHIFT * 2));
    float CC_GAIN    = ADC_CALIBRATION_FACTOR / R_SENSE;
    float charge_mAh = (charge * CC_GAIN) / SECONDS_PER_HOUR;

    /* Clear any pending alert */
    uint8_t alarmClear = ALARM_CLEAR_CMD;
    hw_i2c_memWrite(&lvBatMon, ALARM_STATUS_REG, &alarmClear, RESPONSE_LENGTH_SIZE);

    return (charge_mAh / Q_FULL) * PERCENTAGE_FACTOR;
}

/**
 * @brief Gets the battery voltage.
 *
 * @param voltage_cmd The subcommand used to read the voltage.
 *
 * @return The battery voltage on success, or -1 on error.
 */
uint16_t io_lowVoltageBattery_get_voltage(uint16_t voltage_cmd)
{
    osSemaphoreAcquire(bat_mtr_sem, osWaitForever);

    if (!send_subcommand(voltage_cmd))
        return (uint16_t)-1;

    uint8_t buffer[VOLTAGE_RESPONSE_LENGTH];
    if (!read_response(voltage_cmd, VOLTAGE_RESPONSE_LENGTH, buffer))
        return (uint16_t)-1;

    uint16_t voltage = buffer[0] | (buffer[1] << BYTE_SHIFT);
    return voltage;
}
