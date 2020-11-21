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
    /// Ventilation mode, must be one of the following:
    /// - `1` → PC-CMV (default)
    /// - `2` → PC-AC
    /// - `3` → VC-CMV
    /// - `4` → PC-VSAI
    /// - `5` → VC-AC
    VentilationMode = 1,
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
    /// Inspiratory trigger flow in percent
    InspiratoryTriggerFlow = 10,
    /// Expiratory trigger flow in percent
    ExpiratoryTriggerFlow = 11,
    /// Minimum duration of inhalation in ms (value bounds must be between 100 and 3000)
    TiMin = 12,
    /// Maximum duration of inhalation in ms (value bounds must be between 200 and 5000)
    TiMax = 13,
    /// Threshold for low inspiratory minute volume alarm in L/min (value bounds must be between 0
    /// and 20)
    LowInspiratoryMinuteVolumeAlarmThreshold = 14,
    /// Threshold for high inspiratory minute volume alarm in L/min (value bounds must be between 10
    /// and 40)
    HighInspiratoryMinuteVolumeAlarmThreshold = 15,
    /// Threshold for low expiratory minute volume alarm in L/min (value bounds must be between 0
    /// and 20)
    LowExpiratoryMinuteVolumeAlarmThreshold = 16,
    /// Threshold for high expiratory minute volume alarm in L/min (value bounds must be between 10
    /// and 40)
    HighExpiratoryMinuteVolumeAlarmThreshold = 17,
    /// Threshold for low expiratory rate alarm in cycle per minute (value bounds must be between 5
    /// and 25)
    LowExpiratoryRateAlarmThreshold = 18,
    /// Threshold for high expiratory rate alarm in cycle per minute (value bounds must be between
    /// 20 and 35)
    HighExpiratoryRateAlarmThreshold = 19,
    /// Target tidal volume in mL (value bounds must be between 50 and 2000)
    TargetTidalVolume = 20,
    /// Threshold for low tidal volume in mL (value bounds must be between 0 and 1000)
    LowTidalVolumeAlarmTreshold = 21,
    /// Threshold for high tidal volume in mL (value bounds must be between 50 and 2000)
    HighTidalVolumeAlarmTreshold = 22,
    /// Duration in ms of closing both valves to effectively measure plateau pressure in volume
    /// control modes (value bounds must be between 100 and 1000)
    PlateauDuration = 23,
    /// Threshold for leak alarm in cL/min (value bounds must be between 0 and 10000)
    LeakAlarmThreshold = 24,
};

/**
 * Parse input and handle changes of settings
 *
 * @warning This must be used after `initTelemetry()`
 */
void serialControlLoop();
