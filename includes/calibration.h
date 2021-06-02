/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file calibration.h
 * @brief Calibration related functions
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// External libraries
#include "Arduino.h"

///  Initialization of calibration process
void Calibration_Init(void);

/**
 * Block execution for a given duration
 *
 * @param ms  Duration of the blocking in millisecond
 */
void Calibration_Wait_Measure_Pressure(uint16_t ms);

///  Read keyboard duing calibration process (delayed)
void Calibration_Read_Keyboard_Delayed(void);

///  Restart calibration process
void Calibration_Restart(void);

/**
 * Check if calibration mode is started
 *
 * @return True if calibration mode is started
 */
bool Calibration_Started(void);
