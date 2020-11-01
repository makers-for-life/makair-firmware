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


PressureSensor inspiratoryPressureSensor;

PressureSensor::PressureSensor(){
	m_PressureSensorOffset = 0;
}

// Get the measured for the feedback control (in mmH2O)
int16_t PressureSensor::read() {
    int16_t withOffset = convertSensor2Pressure(analogRead(PIN_PRESSURE_SENSOR)) - m_PressureSensorOffset;
    return max(int16_t(0), withOffset);
}
