#pragma once

#include "hw_gpio.h"
#include "cmsis_os2.h"

extern osSemaphoreId_t bat_mtr_sem;

/**
 * Initialize the LV battery monitor.
 * 
 * @return true if semaphore init is sucessful and false otherwise
 */
bool io_lowVoltageBattery_init();

/**
 * Gets state of charge (SOC) from low voltage battery
 * 
 * @return the SOC of the battery as a percentage
 */
float io_lowVoltageBattery_getSOC();
