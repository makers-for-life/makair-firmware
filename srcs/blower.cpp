/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file blower.cpp
 * @brief Tools to control the blower
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/blower.h"

// Internal libraries
#include "../includes/parameters.h"

// INITIALISATION =============================================================

Blower blower;

// FUNCTIONS ==================================================================

Blower::Blower() {}

Blower::Blower(HardwareTimer* p_hardwareTimer, uint16_t p_timerChannel, uint16_t p_blowerPin) {
    actuator = p_hardwareTimer;
    timerChannel = p_timerChannel;
    blowerPin = p_blowerPin;
    m_stopped = true;
    m_speed = 0;
    m_targetSpeed = 0;
    m_lastCallDate = millis();
}

void Blower::setup() {
    actuator->setMode(timerChannel, TIMER_OUTPUT_COMPARE_PWM1, blowerPin);

    // Set PPM width to 1ms
    actuator->setCaptureCompare(timerChannel, BlowerSpeed2MicroSeconds(0), MICROSEC_COMPARE_FORMAT);
    actuator->resume();
}

void Blower::runSpeedWithRampUp(uint16_t p_targetSpeed) {
    // cppcheck-suppress unsignedPositive ; MIN_BLOWER_SPEED might not be equal to 0
    if ((p_targetSpeed >= MIN_BLOWER_SPEED) && (p_targetSpeed <= MAX_BLOWER_SPEED)) {

        if (p_targetSpeed != m_targetSpeed) {  // first time with new target
            m_lastCallDate = micros();
            m_targetSpeed = p_targetSpeed;
            m_speed = max(m_speed, uint16_t(MIN_BLOWER_SPEED));
        }
    }
}

void Blower::execute() {
    // apply ramp-up
    // Max acceleration is one unit per ms. This means full ramp-up in 1.8s
    uint32_t currentDate = micros();
    uint16_t runSpeed = 0;
    if (m_targetSpeed > m_speed) {
        runSpeed = min(m_targetSpeed, uint16_t(m_speed + (currentDate - m_lastCallDate) / 1000));
    } else {
        runSpeed = m_targetSpeed;
    }
    m_lastCallDate = currentDate;
    this->runSpeed(runSpeed);
}

void Blower::runSpeed(uint16_t p_runSpeed) {
    // cppcheck-suppress unsignedPositive ; MIN_BLOWER_SPEED might not be equal to 0
    if ((p_runSpeed >= MIN_BLOWER_SPEED) && (p_runSpeed <= MAX_BLOWER_SPEED)) {

        // do not forcefully set the capture compare again and again if speed do not change
        if (m_stopped || (m_speed != p_runSpeed)) {
            actuator->setCaptureCompare(timerChannel, BlowerSpeed2MicroSeconds(p_runSpeed),
                                        MICROSEC_COMPARE_FORMAT);
            m_speed = p_runSpeed;
            m_stopped = false;
        }
    } else {
        DBG_DO(Serial.print("Blower value is wrong: "));
        DBG_DO(Serial.println(p_runSpeed));
    }
}

int32_t Blower::getBlowerPressure(int32_t p_flow) {
    int32_t returnValue;
    // For now the blower has only been characterize at max speed
    if (m_speed == MAX_BLOWER_SPEED) {
        // This order 2 characteruzation has been made experimentally
        // todo sage overflow
        returnValue = 703 - 281 * p_flow / 100000 - 832 * (p_flow / 100) * (p_flow / 100) / 1000000;
    } else {
        // todo better characterization
        returnValue = 703 * m_speed / MAX_BLOWER_SPEED - 281 * p_flow / 100000
                      - 832 * (p_flow / 100) * (p_flow / 100) / 1000000;
    }

    return min(int32_t(703), max(returnValue, int32_t(0)));
}

uint16_t Blower::getSpeed() const { return m_speed; }

uint16_t Blower::getTargetSpeed() const { return m_targetSpeed; }

void Blower::stop() {
    actuator->setCaptureCompare(timerChannel, BlowerSpeed2MicroSeconds(0), MICROSEC_COMPARE_FORMAT);
    m_stopped = true;
    m_speed = 0;
    m_targetSpeed = 0;
}
