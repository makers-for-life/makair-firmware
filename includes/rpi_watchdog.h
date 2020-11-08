/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file rpi_watchdog.h
 * @brief Watchdog for the Raspberry PI
 *****************************************************************************/

#pragma once
#include "../includes/parameters.h"

/// Number of seconds with no heartbeat after which the RPi should be restarted
#define COUNTDOWN_IN_S 60

/// Watchdog for the Raspberry PI
class RpiWatchdog {
 public:
    /// Default constructor
    RpiWatchdog();

    /// This should be called by the main state machine every 1s
    void update();

    /// When the UI software on the Raspberry PI sends a heartbeat, reset countdown
    void resetCountDown();

    /// Disable countdown mode (used for debug)
    void disable();

 private:
    int32_t m_countDown;
    enum RpiWatchdogStep { COUNT_DOWN, SWITCH_OFF_RASPBERRY, SWITCH_ON_RASPBERRY, DISABLED };
    RpiWatchdogStep m_rpiWatchdogStep;
};

extern RpiWatchdog rpiWatchdog;
