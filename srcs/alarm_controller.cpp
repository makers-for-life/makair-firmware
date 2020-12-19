/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file alarm_controller.cpp
 * @brief Core logic to manage alarm features
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Externals

// Internals
#include "../includes/alarm_controller.h"
#include "../includes/buzzer.h"
#include "../includes/cycle.h"
#include "../includes/screen.h"
#include "../includes/telemetry.h"

// INITIALISATION =============================================================

AlarmController alarmController;

// FUNCTIONS ==================================================================

AlarmController::AlarmController()
    : m_highestPriority(AlarmPriority::ALARM_NONE),
      m_snoozeTime(0u),
      m_alarms({

          /**
           * RCM-SW-2
           * The device shall embed a high priority alarm 11 when the pressure is below < 2cmH2O
           * from the 3th cycle.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_2, 3u),

          /* RCM-SW-1
           * The device shall embed a high priority alarm 12 when the plateau pressure is not
           * reached (absolute difference > 20% in absolute value) from the 3th respiratory cycle.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_1, 3u),

          /**
           * RCM-SW-12
           * The device shall monitor the battery voltage and trig a high priority alarm 13 when
           * voltage is < 24V.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_12, 1u),

          /**
           * RCM-SW-3
           * The device shall embed a high priority alarm 14 when the PEEP target is not reached
           * (absolute difference > 2cmH2O) from the 3th respiratory cycle.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_3, 3u),

          /**
           * RCM-SW-4
           * The device shall embed a high priority alarm 40 when Inspiratory minute Volume is too low from the 3th respiratory cycle.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_4, 3u),

          /**
           * RCM-SW-5
           * The device shall embed a high priority alarm 41 when Inspiratory minute Volume is too high from the 3th respiratory cycle.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_5, 3u),

           /**
           * RCM-SW-6
           * The device shall embed a high priority alarm 42 when Expiratory minute Volume is too low from the 3th respiratory cycle.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_6, 3u),

          /**
           * RCM-SW-7
           * The device shall embed a high priority alarm 43 when Expiratory minute Volume is too high from the 3th respiratory cycle.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_7, 3u),

          /**
           * RCM-SW-8
           * The device shall embed a high priority alarm 44 when Respiratory rate is too low from the 3th respiratory cycle.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_8, 3u),

          /**
           * RCM-SW-9
           * The device shall embed a high priority alarm 45 when Respiratory rate is too high from the 3th respiratory cycle.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_9, 3u),

          /**
           * RCM-SW-10
           * The device shall embed a high priority alarm 46 when Leak is too high.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_10, 3u),

          /**
           * RCM-SW-18
           * The device shall embed a high priority alarm 17 when the peak pressure is > 80cmH2O.
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_18, 1u),

          /**
           * RCM-SW-11
           * The device shall monitor the battery voltage and trigger a medium priority alarm 21
           * when voltage is < 24,6V.
           */
          Alarm(AlarmPriority::ALARM_MEDIUM, RCM_SW_11, 1u),

          /**
           * RCM-SW-14
           * The device shall embed a medium priority alarm 22 when the plateau pressure is not
           * reached (absolute difference > 20% in absolute value) until the 2nd respiratory
           * cycle.
           */
          Alarm(AlarmPriority::ALARM_MEDIUM, RCM_SW_14, 2u),

          /**
           * RCM-SW-15
           * The device shall embed a medium priority alarm 23 when the PEEP target is not reached
           * (absolute difference > 2cmH2O) until the 2nd respiratory cycle.
           */
          Alarm(AlarmPriority::ALARM_MEDIUM, RCM_SW_15, 2u),

          /**
           * RCM-SW-19
           * The device shall embed a medium priority alarm 24 when the pressure is < 2cmH2O at the
           * second cycle (patient disconnection).
           */
          Alarm(AlarmPriority::ALARM_MEDIUM, RCM_SW_19, 2u),

          /**
           * RCM-SW-16
           * The device shall embed an information (audible) signal 31 when the mains are
           * disconnected to alert the user (vOut < 26,5V).
           */
          Alarm(AlarmPriority::ALARM_LOW, RCM_SW_16, 1u),

          /**
           * RCM-SW-20
           * The device shall embed a medium priority alarm 47 when Tidal Volume is too low
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_20, 3u),

          /**
           * RCM-SW-21
           * The device shall embed a medium priority alarm 48 when Tidal Volume is too high
           */
          Alarm(AlarmPriority::ALARM_HIGH, RCM_SW_21, 3u),
        }),


      m_tick(0u),
      m_unsnooze(true),
      m_pressure(0u),
      m_phase(CyclePhases::INHALATION),
      // cppcheck-suppress misra-c2012-5.2 ; false positive
      m_cycle_number(0u) {
    for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
        m_snoozedAlarms[i] = false;
    }
    for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
        m_triggeredAlarms[i] = 0u;
    }
}

void AlarmController::snooze() {
    if (m_unsnooze) {
        m_unsnooze = false;
        digitalWrite(PIN_LED_GREEN, LED_GREEN_ACTIVE);
        m_snoozeTime = millis();
        for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
            Alarm* current = &m_alarms[i];
            if (current->isTriggered()) {
                m_snoozedAlarms[i] = true;
            } else {
                m_snoozedAlarms[i] = false;
            }
        }

        Buzzer_Mute();
    }
    sendControlAck(9, !m_unsnooze);
}

void AlarmController::detectedAlarm(uint8_t p_alarmCode,
                                    uint32_t p_cycleNumber,
                                    uint32_t p_expected,
                                    uint32_t p_measured) {
    for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
        Alarm* current = &m_alarms[i];
        bool wasTriggered = current->isTriggered();
        if (current->isEnabled() && (current->getCode() == p_alarmCode)) {
            current->detected(p_cycleNumber);

            if (current->isTriggered()) {
                for (uint8_t j = 0; j < ALARMS_SIZE; j++) {
                    if (m_triggeredAlarms[j] == p_alarmCode) {
                        break;
                    }
                    if (m_triggeredAlarms[j] == 0u) {
                        m_triggeredAlarms[j] = p_alarmCode;
                        break;
                    }
                }

                if (!wasTriggered) {
                    sendAlarmTrap(m_tick, m_pressure, m_phase, m_cycle_number, current->getCode(),
                                  current->getPriority(), true, p_expected, p_measured,
                                  current->getCyclesSinceTrigger());
                }
            }
            break;
        }
    }
}

int compare(uint8_t a, uint8_t b) {
    int result = 0;

    if (a == b) {
        result = 0;
    } else if (a == 0u) {
        result = 1;
    } else if (b == 0u) {
        result = -1;
    } else if (a < b) {
        result = -1;
    } else {
        result = 1;
    }

    return result;
}

void AlarmController::notDetectedAlarm(uint8_t p_alarmCode) {
    for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
        Alarm* current = &m_alarms[i];
        bool wasTriggered = current->isTriggered();
        if (current->getCode() == p_alarmCode) {
            current->notDetected();

            if (!current->isTriggered()) {
                for (uint8_t j = 0; j < ALARMS_SIZE; j++) {
                    if (m_triggeredAlarms[j] == p_alarmCode) {
                        m_triggeredAlarms[j] = 0u;
                        std::sort(m_triggeredAlarms, &m_triggeredAlarms[ALARMS_SIZE], compare);
                        break;
                    }
                }

                if (wasTriggered) {
                    sendAlarmTrap(m_tick, m_pressure, m_phase, m_cycle_number, current->getCode(),
                                  current->getPriority(), false, 0u, 0u,
                                  current->getCyclesSinceTrigger());
                }
            }
            break;
        }
    }
}

void AlarmController::runAlarmEffects(uint32_t p_tick) {
    AlarmPriority highestPriority = AlarmPriority::ALARM_NONE;
    uint8_t triggeredAlarmCodes[ALARMS_SIZE];
    uint8_t numberOfTriggeredAlarms = 0;
    bool justUnsnoozed = false;

    for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
        Alarm* current = &m_alarms[i];
        if (current->isTriggered()) {
            if (numberOfTriggeredAlarms == 0u) {
                highestPriority = current->getPriority();
            }

            triggeredAlarmCodes[numberOfTriggeredAlarms] = current->getCode();
            numberOfTriggeredAlarms++;

            if (!m_unsnooze && !m_snoozedAlarms[i]) {
                unsnooze();
                justUnsnoozed = true;
            }
        } else {
            m_snoozedAlarms[i] = false;
        }
    }

    uint32_t millisSinceSnooze = millis() - m_snoozeTime;
    if (!m_unsnooze && (m_snoozeTime > 0u) && (millisSinceSnooze >= 120000u)) {
        unsnooze();
    }

    if ((p_tick % (LCD_UPDATE_PERIOD_US / MAIN_CONTROLLER_COMPUTE_PERIOD_MICROSECONDS)) == 0u) {
        displayAlarmInformation(triggeredAlarmCodes, numberOfTriggeredAlarms);
    }

    if (highestPriority == AlarmPriority::ALARM_HIGH) {
        if ((m_highestPriority != highestPriority) || justUnsnoozed) {
            if (m_unsnooze) {
                Buzzer_High_Prio_Start();
            }
        }

        if ((p_tick % 100u) == 50u) {
            digitalWrite(PIN_LED_RED, LED_RED_ACTIVE);
        } else if ((p_tick % 100u) == 0u) {
            digitalWrite(PIN_LED_RED, LED_RED_INACTIVE);
        } else {
        }
        digitalWrite(PIN_LED_YELLOW, LED_YELLOW_INACTIVE);
    } else if (highestPriority == AlarmPriority::ALARM_MEDIUM) {
        if ((m_highestPriority != highestPriority) || justUnsnoozed) {
            if (m_unsnooze) {
                Buzzer_Medium_Prio_Start();
            }
        }
        digitalWrite(PIN_LED_RED, LED_RED_INACTIVE);
        if ((p_tick % 100u) == 50u) {
            digitalWrite(PIN_LED_YELLOW, LED_YELLOW_ACTIVE);
        } else if ((p_tick % 100u) == 0u) {
            digitalWrite(PIN_LED_YELLOW, LED_YELLOW_INACTIVE);
        } else {
        }
    } else if (highestPriority == AlarmPriority::ALARM_LOW) {
        if ((m_highestPriority != highestPriority) || justUnsnoozed) {
            if (m_unsnooze) {
                Buzzer_Low_Prio_Start();
            }
        }

        digitalWrite(PIN_LED_RED, LED_RED_INACTIVE);
        digitalWrite(PIN_LED_YELLOW, LED_YELLOW_ACTIVE);
    } else {
        Buzzer_Stop();

        digitalWrite(PIN_LED_RED, LED_RED_INACTIVE);
        digitalWrite(PIN_LED_YELLOW, LED_YELLOW_INACTIVE);
    }

    m_highestPriority = highestPriority;
}

// cppcheck-suppress unusedFunction
void AlarmController::unsnooze() {
    digitalWrite(PIN_LED_GREEN, LED_GREEN_INACTIVE);
    m_snoozeTime = 0u;
    for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
        m_snoozedAlarms[i] = false;
    }
    m_unsnooze = true;
    sendControlAck(9, !m_unsnooze);
}

// cppcheck-suppress unusedFunction
void AlarmController::updateCoreData(uint32_t p_tick,
                                     uint16_t p_pressure,
                                     CyclePhases p_phase,
                                     uint32_t p_cycle_number) {
    m_tick = p_tick;
    m_pressure = p_pressure;
    m_phase = p_phase;
    m_cycle_number = p_cycle_number;
}

void AlarmController::updateEnabledAlarms(Alarms enabledAlarms) {
    // Disable every alarms
    for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
        m_alarms[i].disable();
    }

    // Enable provided alarms
    for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
        if (enabledAlarms.alarms[i] != 0u) {
            for (uint8_t j = 0; j < ALARMS_SIZE; j++) {
                if (m_alarms[j].getCode() == enabledAlarms.alarms[i]) {
                    m_alarms[j].enable();
                    break;
                }
            }
        }
    }
}
