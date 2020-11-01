/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure.cpp
 * @brief Pressure sensor related functions
 *****************************************************************************/

// INCLUDES ==================================================================

// Associated header
#include "../includes/pressure.h"
#include "../includes/pressure_utl.h"

// External
#include "Arduino.h"
#include <algorithm>

// Internal
#include "../includes/parameters.h"

// PROGRAM =====================================================================

// Get the measured or simulated pressure for the feedback control (in mmH2O)
int16_t inspiratoryPressureSensorOffset = 0;

int16_t readPressureSensor(uint16_t tick) {
    (void)tick;
    int16_t withOffset = convertSensor2Pressure(analogRead(PIN_PRESSURE_SENSOR)) - inspiratoryPressureSensorOffset;
    return max(int16_t(0), withOffset);
}
