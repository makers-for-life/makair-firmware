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
 * Get the number of milliliters since last reset
 *
 * @param reset_after_read If true, performs the volume reset in the same atomic operation
 * @return Volume of air that went through sensor since last reset in mL
 */
int32_t MFM_read_milliliters(bool reset_after_read);

/**
 * Get the number of milliliters since last reset for expiratory sensor
 *
 * @param reset_after_read If true, performs the volume reset in the same atomic operation
 * @return Volume of air that went through sensor since last reset in mL
 */
int32_t MFM_expi_read_milliliters(bool reset_after_read);

/// Reset the volume counter
void MFM_reset(void);

/**
 * Calibrate the zero of the sensor
 *
 * @return One of:
 * - MFM_CALIBRATION_OK: all right
 * - MFM_CALIBRATION_IMPOSSIBLE: communication problem
 * - MFM_CALIBRATION_OUT_OF_RANGE: unbelievable 10SLPM sensor drift; time to change it?
 *
 * @note This uses the mean of 10 samples
 */
int8_t MFM_calibrateZero(void);

#define MFM_CALIBRATION_OK 0
#define MFM_CALIBRATION_IMPOSSIBLE 1
#define MFM_CALIBRATION_OUT_OF_RANGE 2

/**
 * Get massflow meter offset
 */
int32_t MFM_getOffset(void);

/**
 * Read instant air flow
 */
int32_t MFM_read_airflow(void);

/**
 * Read instant air flow
 */
int32_t MFM_expi_read_airflow(void);

/**
 * Get the serial number of the inspiratory flow meter
 *
 * @return The serial number, or `0` if before init or if init failed
 */
uint32_t MFM_read_serial_number(void);

/**
 * Get the serial number of the expiratory flow meter
 *
 * @return The serial number, or `0` if before init or if init failed
 */
uint32_t MFM_expi_read_serial_number(void);

#define MFM_SFM_3300D_I2C_ADDRESS 0x40
#define MFM_HONEYWELL_HAF_I2C_ADDRESS 0x49
#define MFM_SDP703_02_I2C_ADDRESS 0x40
#define MFM_SFM3019_I2C_ADDRESS 0x2E

