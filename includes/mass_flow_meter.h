/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file mass_flow_meter.h
 * @brief Mass flow meter management
 *****************************************************************************/

#pragma once

#include <Arduino.h>
/**
 * Initialize Mass Flow Meter
 *
 *  @return True if there is a Mass Flow Meter connected
 *  @warning If no Mass Flow Meter is detected, you will always read volume = 0 mL
 */
bool MFM_init(void);

/**
 * Returns the number of milliliters since last reset
 *
 * @param reset_after_read If true, performs the volume reset in the same atomic operation
 */
int32_t MFM_read_milliliters(bool reset_after_read);

/**
 * Reset the volume counter
 */
void MFM_reset(void);

/**
 * Calibrate the zero of the sensor
 *
 * @note This uses the mean of 10 samples
 */
void MFM_calibrateZero(void);

/**
 * Read instant air flow
 */
int32_t MFM_read_airflow(void);

#if MASS_FLOW_METER_SENSOR == MFM_SFM_3300D
#define MFM_SENSOR_I2C_ADDRESS 0x40
#endif

#if MASS_FLOW_METER_SENSOR == MFM_SDP703_02
#define MFM_SENSOR_I2C_ADDRESS 0x40
#endif

#if MASS_FLOW_METER_SENSOR == MFM_HONEYWELL_HAF
#define MFM_SENSOR_I2C_ADDRESS 0x49
#endif
