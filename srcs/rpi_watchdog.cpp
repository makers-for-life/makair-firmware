/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file rpi_watchdog.cpp
 * @brief Watchdog for the Raspberry PI
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

#include "../includes/rpi_watchdog.h"

// INITIALISATION =============================================================

RpiWatchdog rpiWatchdog = RpiWatchdog();

// FUNCTIONS ==================================================================

RpiWatchdog::RpiWatchdog() {
    m_rpiWatchdogStep = COUNT_DOWN;
    m_countDown = COUNTDOWN_IN_S;
}

void RpiWatchdog::update() {
    if (m_rpiWatchdogStep == COUNT_DOWN) {
        m_countDown--;
        if (m_countDown <= 0) {
            m_rpiWatchdogStep = SWITCH_OFF_RASPBERRY;
        }
    } else if (m_rpiWatchdogStep == SWITCH_OFF_RASPBERRY) {
        // Turn off the Raspberry Pi power
        digitalWrite(PIN_ENABLE_PWR_RASP, PWR_RASP_INACTIVE);
        m_rpiWatchdogStep = SWITCH_ON_RASPBERRY;
    } else if (m_rpiWatchdogStep == SWITCH_ON_RASPBERRY) {
        // Turn on the Raspberry Pi power
        digitalWrite(PIN_ENABLE_PWR_RASP, PWR_RASP_ACTIVE);
        m_countDown = COUNTDOWN_IN_S;
        m_rpiWatchdogStep = WAIT_FOR_FIRST_HEARTBEAT;
    } else if (m_rpiWatchdogStep == WAIT_FOR_FIRST_HEARTBEAT) {
        // Do nothing here
        // Next time resetCountDown() is called because a heartbeat was received, let's resume
    } else if (m_rpiWatchdogStep == DISABLED) {
        // Do nothing when watchdog is disabled
    } else {
        // Do nothing
    }
}

void RpiWatchdog::resetCountDown() {
    if (m_rpiWatchdogStep == WAIT_FOR_FIRST_HEARTBEAT) {
        m_rpiWatchdogStep = COUNT_DOWN;
    }

    m_countDown = COUNTDOWN_IN_S;
}

void RpiWatchdog::disable() { m_rpiWatchdogStep = DISABLED; }
