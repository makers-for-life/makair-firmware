/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file alarm.cpp
 * @brief Describes an alarm and handle its dynamic state
 *****************************************************************************/

// INCLUDES ===================================================================

// Externals
#include "Arduino.h"

// Internals
#include "../includes/alarm.h"

// FUNCTIONS ==================================================================

Alarm::Alarm(AlarmPriority p_priority, uint8_t p_code, uint8_t p_detectionThreshold) {
    m_priority = p_priority;
    m_code = p_code;
    m_detectionThreshold = p_detectionThreshold;
    m_cycleNumber =
        UINT32_MAX;  // We need this not to be 0 so that alarms can be triggered just after booting
    m_detectionNumber = 0u;
    m_cyclesSinceTrigger = 0u;
    m_enabled = true;
}

AlarmPriority Alarm::getPriority() const { return m_priority; }

uint8_t Alarm::getCode() const { return m_code; }

// cppcheck-suppress unusedFunction
uint32_t Alarm::getCyclesSinceTrigger() const { return m_cyclesSinceTrigger; }

bool Alarm::isTriggered() const { return m_enabled && (m_detectionNumber >= m_detectionThreshold); }

void Alarm::detected(uint32_t p_cycleNumber) {
    if (m_cycleNumber != p_cycleNumber) {
        m_cycleNumber = p_cycleNumber;
        m_detectionNumber++;
        if (m_detectionNumber > m_detectionThreshold) {
            m_detectionNumber = m_detectionThreshold;
            m_cyclesSinceTrigger++;
        }
    }
}

void Alarm::notDetected() {
    m_cycleNumber = UINT32_MAX;
    m_detectionNumber = 0u;
    m_cyclesSinceTrigger = 0u;
}

void Alarm::enable() { m_enabled = true; }

void Alarm::disable() { m_enabled = false; }

bool Alarm::isEnabled() { return m_enabled; }
