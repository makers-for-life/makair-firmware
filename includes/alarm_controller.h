/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file alarm_controller.h
 * @brief Core logic to manage alarm features
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Internals
#include "../includes/alarm.h"
#include "../includes/cycle.h"

// CONSTANTS ==================================================================

#define ALARMS_SIZE 16u

#define RCM_SW_1 12u // Plateau pressure not reached
#define RCM_SW_2 11u // Patient is unplugged
#define RCM_SW_3 14u // Peep not reached
// #define RCM_SW_6 15u - NOT IN THIS VERSION
// #define RCM_SW_8 18u - NOT IN THIS VERSION
#define RCM_SW_11 21u // Battery Low
#define RCM_SW_12 13u // Battery vers Low
#define RCM_SW_14 22u // Plateau pressure not reached
#define RCM_SW_15 23u // Peep not reached
#define RCM_SW_16 31u // Mains disconnected
#define RCM_SW_18 17u // Pressure too high 
#define RCM_SW_19 24u // Patient is unplugged

#define RCM_SW_4 40u // Inspiratory minute Volume is too low
#define RCM_SW_5 41u // Inspiratory minute Volume is too high
#define RCM_SW_6 42u // Expiratory minute Volume is too low
#define RCM_SW_7 43u // Expiratory minute Volume is too high
#define RCM_SW_8 44u // Respiratory rate is too low
#define RCM_SW_9 45u // Respiratory rate is too high

/// List of alarms (named by their code)
struct Alarms {
    uint8_t alarms[ALARMS_SIZE];
};

// CLASS =====================================================================

/// Manage alarm features
class AlarmController {
 public:
    /// Default constructor
    AlarmController();

    /**
     * Snooze alarm for 2 minutes
     *
     * There is no more buzzer during the alarm
     */
    void snooze();

    /**
     * Unsnooze alarms
     */
    void unsnooze();

    /// Check if alarms are currently snoozed
    bool isSnoozed() const { return !m_unsnooze; }

    /**
     * Mark a specific alarm as detected
     *
     * @param p_alarmCode The code of the alarm
     * @param p_cycleNumber The cycle number since the device startup
     * @param p_expected The expected value
     * @param p_measured The measured value that was different from the expected value thus
     * triggering the alarm
     */
    void detectedAlarm(uint8_t p_alarmCode,
                       uint32_t p_cycleNumber,
                       uint32_t p_expected,
                       uint32_t p_measured);

    /**
     * Reset detection of a specific alarm
     * @param p_alarmCode The code of the alarm
     */
    void notDetectedAlarm(uint8_t p_alarmCode);

    /**
     * Run effects (buzzer, LCD message, LED) according to the currently triggered alarms
     *
     * @param p_tick Centile in the respiratory cycle
     */
    void runAlarmEffects(uint32_t p_tick);

    /// Update internal state of alarm controller with data from pressure controller
    void updateCoreData(uint32_t p_tick,
                        uint16_t p_pressure,
                        CyclePhases p_phase,
                        uint32_t p_cycle_number);

    /// Get the alarms triggered during this cycle
    uint8_t* triggeredAlarms() { return m_triggeredAlarms; }

    /**
     * Update the list of enabled alarms (alarms not provided here will have no effects)
     *
     * @param The new list of enabled alarms
     */
    void updateEnabledAlarms(Alarms enabledAlarms);

 private:
    /// Highest priority of the currently triggered alarms
    AlarmPriority m_highestPriority;

    /// Time when snoozed was triggered
    uint32_t m_snoozeTime;

    /// Collections of available alarms
    Alarm m_alarms[ALARMS_SIZE];

    /// Collections of snoozed alarms
    bool m_snoozedAlarms[ALARMS_SIZE];

    /// Alarms currently triggered
    uint8_t m_triggeredAlarms[ALARMS_SIZE];

    /// Current pressure
    uint16_t m_tick;

    /// Current pressure
    uint16_t m_pressure;

    /// Current phase
    CyclePhases m_phase;

    /// Current cycle number
    uint32_t m_cycle_number;

    /// Is unsnoozed right now
    bool m_unsnooze;
};

// INITIALISATION =============================================================

/// Instance of the alarm controller
extern AlarmController alarmController;
