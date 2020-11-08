/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file serial_control.h
 * @brief Handle control protocol on the serial input
 *****************************************************************************/

#pragma once

/// Special value that can be used in a heartbeat control message to disable RPi watchdog
#define DISABLE_RPI_WATCHDOG 43690u

/// Available settings in the control protocol
enum ControlSetting {
    /// Heartbeat used for the RPi watchdog feature (value is ignored except for the special value
    /// `DISABLE_RPI_WATCHDOG` which disables watchdog)
    Heartbeat = 0,
    /// Peak pressure in mmH20 (value bounds must be between 0 and 700)
    PeakPressure = 1,
    /// Plateau pressure in mmH2O (value bounds must be between 100 and 400)
    PlateauPressure = 2,
    /// PEEP in mmH2O (value bounds must be between 0 and 300)
    PEEP = 3,
    /// Number of cycles per minute (value bounds must be between 5 and 35)
    CyclesPerMinute = 4,
    /// Expiration term in the "Inspiration/Expiration" ratio given that Inspiration = 10 (value
    /// bounds must be between 10 and 60)
    ExpiratoryTerm = 5,
    /// State of the trigger (value must be 1 if enabled and 0 if disabled)
    TriggerEnabled = 6,
    /// Trigger offset in mmH2O (value bounds must be between 0 and 100)
    TriggerOffset = 7,
    /// State of the respiration (value must be 1 if enabled and 0 if disabled)
    RespirationEnabled = 8,
    /// Alarm snooze (value must be 1 to snooze and 0 to unsnooze)
    AlarmSnooze = 9,
};

/**
 * Parse input and handle changes of settings
 *
 * @warning This must be used after `initTelemetry()`
 */
void serialControlLoop();
