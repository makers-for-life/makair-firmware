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