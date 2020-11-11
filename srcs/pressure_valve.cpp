/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure_valve.cpp
 * @brief Tools to control Pressure Valves
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/pressure_valve.h"

// External libraries
#include <algorithm>

// Internal libraries
#include "../includes/parameters.h"

// INITIALISATION =============================================================

PressureValve expiratoryValve;
PressureValve inspiratoryValve;

// FUNCTIONS ==================================================================

PressureValve::PressureValve() {}

PressureValve::PressureValve(HardwareTimer* p_hardwareTimer,
                             uint16_t p_timerChannel,
                             uint16_t p_valvePin,
                             uint16_t p_openApertureAngle,
                             uint16_t p_closeApertureAngle) {
    actuator = p_hardwareTimer;
    timerChannel = p_timerChannel;
    valvePin = p_valvePin;
    openApertureAngle = p_openApertureAngle;
    closeApertureAngle = p_closeApertureAngle;
    minApertureAngle = min(p_closeApertureAngle, p_openApertureAngle);
    maxApertureAngle = max(p_closeApertureAngle, p_openApertureAngle);
    command = p_closeApertureAngle;
    position = -1;
}

void PressureValve::setup() {
    actuator->setMode(timerChannel, TIMER_OUTPUT_COMPARE_PWM1, valvePin);
    actuator->setCaptureCompare(timerChannel, 0, MICROSEC_COMPARE_FORMAT);
}

void PressureValve::open() { command = openApertureAngle; }

void PressureValve::close() { command = closeApertureAngle; }

void PressureValve::open(uint16_t p_command) { command = p_command; }

uint16_t PressureValve::openLinear(uint16_t p_command) {

    uint32_t value = (uint32_t)((p_command * 70u / 125u + 30u)*10u);

    command = (uint16_t) ((76u * value / 10u - (985u * value*value / 100000u ) + (((44u *value*value) /1000u) * value ) / 10000u -1140u)/10u);

    return command;

}

uint16_t valveAngle2MicroSeconds(uint16_t value) {
    // Faulhaber motors works with PWM
    return map(value, 0, 125, FAULHABER_OPENED, FAULHABER_CLOSED);
}
