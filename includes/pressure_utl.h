/******************************************************************************
 * @file pressure_utl.h
 * @copyright Copyright (c) 2020 Makers For Life
 * @author Makers For Life
 * @brief Pressure computing utility function
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

#include <stdint.h>

// FUNCTIONS ==================================================================

/**
 * Convert the analog value from sensor to a pressure value
 *
 * @param sensorValue Value read from the analog input connected to the sensor
 * @return The pressure in mmH2O
 */
int16_t convertSensor2Pressure(uint32_t sensorValue);

/**
 * Reset the value of void filteredRawPressure to 0
 *
 * @note Mainly for testing purpose
 */
void resetFilteredRawPressure();
