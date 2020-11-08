/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file main_state_machine.cpp
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

#include "../includes/rpi_watchdog.h"

// INITIALISATION =============================================================

RpiWatchdog rpiWatchdog = RpiWatchdog();



// FUNCTIONS ==================================================================

RpiWatchdog::RpiWatchdog() { RpiWatchdogStep m_rpiWatchdogStep = SETUP; }

void RpiWatchdog::update() {

    if (m_rpiWatchdogStep == SETUP) {
        m_countDown = COUNT_DOWN_IN_S;
        m_rpiWatchdogStep = COUNT_DOWN;
    }

    else if (m_rpiWatchdogStep == COUNT_DOWN) {
        m_countDown--;
        if (m_countDown <= 0) {
            m_rpiWatchdogStep = SWITCH_OFF_RASPBERRY;
        }
    }

    else if (m_rpiWatchdogStep == SWITCH_OFF_RASPBERRY) {
        // Turn off the Raspberry Pi power
        digitalWrite(PIN_ENABLE_PWR_RASP, PWR_RASP_INACTIVE);
        m_rpiWatchdogStep = SWITCH_ON_RASPBERRY;
    }

    else if (m_rpiWatchdogStep == SWITCH_ON_RASPBERRY) {
        // Turn on the Raspberry Pi power
        digitalWrite(PIN_ENABLE_PWR_RASP, PWR_RASP_ACTIVE);
        m_countDown = COUNT_DOWN_IN_S;
        m_rpiWatchdogStep = COUNT_DOWN;
    }

    else if (m_rpiWatchdogStep == DISABLED) {
        // do nothing when watchdog is disabled
    }
}

void RpiWatchdog::resetCountDown() { m_countDown = COUNT_DOWN_IN_S; }

void RpiWatchdog::disable() { m_rpiWatchdogStep = DISABLED; }
