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

// INITIALISATION =============================================================

PressureSensor inspiratoryPressureSensor;

// FUNCTIONS ==================================================================

PressureSensor::PressureSensor() { m_PressureSensorOffset = 0; }

int32_t PressureSensor::read() {
    int32_t withOffset =
        convertSensor2Pressure(analogRead(PIN_PRESSURE_SENSOR)) - m_PressureSensorOffset;
    return  withOffset;
}
