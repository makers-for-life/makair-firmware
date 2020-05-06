/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file mass_flow_meter.h
 * @brief Mass flow meter management
 *****************************************************************************/

#pragma once

#include <Arduino.h>

// Read the volume, in milliliters
int32_t MFM_read_liters(boolean reset_after_read);

// Calibration set the sensor offset. Flow must be zero during 500ms
void MFM_calibrate(void);

// Reset the volume
void MFM_reset(void);

// Initialize the flow timer
boolean MFM_init(void);

// This input must be refreshed as often as possible with analog input.
// Analog input cannot be read inside the timer, it is not an atomic operation.
extern int32_t MFM_last_value;

// Instant flow in l/min. Value could be MASS_FLOw_ERROR_VALUE if no communication with a digital
// sensor.
double MFM_read_flow(void);

#define MASS_FLOw_ERROR_VALUE 999999

#if MASS_FLOW_METER_SENSOR == MFM_SFM_3300D
#define MFM_SENSOR_I2C_ADDRESS 0x40
#endif

#if MASS_FLOW_METER_SENSOR == MFM_SDP703_02
#define MFM_SENSOR_I2C_ADDRESS 0x40
#endif