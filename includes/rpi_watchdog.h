/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file main_state_machine.h
 * @brief Raspberry pi watchdog
 *****************************************************************************/

#pragma once
#include "../includes/parameters.h"

#define COUNT_DOWN_IN_S 60

/// Main state machine
class RpiWatchdog {
 public:
    /// Default constructor
    RpiWatchdog();

    /// This should be called by the main state machine every 1s
    void update();

    /// When raspberry pi UI send heart beat, reset count down
    void resetCountDown();

    /// Disable countdown mode (used for debug)
    void disable();

 private:
    int32_t m_countDown;
    enum RpiWatchdogStep { SETUP, COUNT_DOWN, SWITCH_OFF_RASPBERRY, SWITCH_ON_RASPBERRY, DISABLED };
    RpiWatchdogStep m_rpiWatchdogStep;
};

extern RpiWatchdog rpiWatchdog;
