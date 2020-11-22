/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file alarm.h
 * @brief Describe an alarm and handle its dynamic state
 *****************************************************************************/

#pragma once

// INCLUDES =====================================================================

// Externals
#include <stdint.h>

// ENUMS =====================================================================

/// Priority levels of an alarm
enum AlarmPriority { ALARM_NONE, ALARM_LOW, ALARM_MEDIUM, ALARM_HIGH };

// CLASS =====================================================================

/// Describe an alarm and handle its dynamic state
class Alarm {
 public:
    /**
     * Parameterized constructor
     *
     * @param p_priority Alarm priority
     * @param p_code Alarm code
     * @param p_detectionThreshold Number of detections in a row to trigger this alarm
     *
     */
    Alarm(AlarmPriority p_priority, uint8_t p_code, uint8_t p_detectionThreshold);

    /// Get the alarm priority
    AlarmPriority getPriority() const;

    /// Get the alarm code
    uint8_t getCode() const;

    /// True if the number of detections is equal or above the detection threshold, false otherwise
    bool isTriggered() const;

    /// Get the number of cycles since the alarm was triggered
    uint32_t getCyclesSinceTrigger() const;

    /**
     * If the alarm is detected, it increments the number of detection until the detection
     * threshold.
     *
     * @param p_cycleNumber The cycle where the detection is done
     */
    void detected(uint32_t p_cycleNumber);

    /// Reset to zero the number of detection.
    void notDetected();

    /// Enable this alarm
    void enable();

    /// Disable this alarm
    void disable();

    /// True if this alarm is enabled
    bool isEnabled();

 private:
    /// Alarm priority
    AlarmPriority m_priority;

    /// Alarm code
    uint8_t m_code;

    /// Alarm detection threshold
    uint8_t m_detectionThreshold;

    /// Number of detections
    uint8_t m_detectionNumber;

    /// Cycle number
    uint32_t m_cycleNumber;

    /// Number of cycles since the alarm was triggered
    uint32_t m_cyclesSinceTrigger;

    /// Whether or not this alarm is enabled
    bool m_enabled;
};
