/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure_controller.cpp
 * @brief Core logic to control the breathing cycle
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/main_controller.h"

// External
#include <algorithm>

// Internal
#include "../includes/alarm_controller.h"
#include "../includes/battery.h"
#include "../includes/blower.h"
#include "../includes/config.h"
#include "../includes/debug.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/parameters.h"
#include "../includes/pressure.h"
#include "../includes/pressure_valve.h"
#include "../includes/telemetry.h"

MainController mainController;

static const int32_t INVALID_ERROR_MARKER = INT32_MIN;

// FUNCTIONS ==================================================================

MainController::MainController() {}

void MainController::setup() {
    DBG_DO(Serial.println(VERSION);)
    DBG_DO(Serial.println("Setup the controller");)

    m_subPhase = HOLD_INSPIRATION;  // TODO remove subphase
    m_inspiratoryFlow = 0;
    m_expiratoryFlow = 0;

    m_cyclesPerMinuteCommand = DEFAULT_CYCLE_PER_MINUTE_COMMAND;
    m_cyclesPerMinuteNextCommand = DEFAULT_CYCLE_PER_MINUTE_COMMAND;
    m_peakPressureCommand = DEFAULT_PEAK_PRESSURE_COMMAND;
    m_peakPressureNextCommand = DEFAULT_PEAK_PRESSURE_COMMAND;
    m_plateauPressureCommand = DEFAULT_PLATEAU_COMMAND;
    m_plateauPressureNextCommand = DEFAULT_PLATEAU_COMMAND;
    m_peepCommand = DEFAULT_PEEP_COMMAND;
    m_peepNextCommand = DEFAULT_PEEP_COMMAND;

    m_pressureTriggerOffsetCommand = DEFAULT_TRIGGER_OFFSET;
    m_pressureTriggerOffsetNextCommand = DEFAULT_TRIGGER_OFFSET;
    m_triggerModeEnabledCommand = TRIGGER_MODE_ENABLED_BY_DEFAULT;
    m_triggerModeEnabledNextCommand = TRIGGER_MODE_ENABLED_BY_DEFAULT;
    m_expiratoryTermCommand = DEFAULT_EXPIRATORY_TERM_COMMAND;
    m_expiratoryTermNextCommand = DEFAULT_EXPIRATORY_TERM_COMMAND;

    m_ventilationController = &pcBipapController;
    m_ventilationControllerNextCommand = &pcBipapController;

    m_lastEndOfRespirationDateMs = 0;
    m_peakPressureMeasure = CONST_INITIAL_ZERO_PRESSURE;
    m_plateauPressureMeasure = CONST_INITIAL_ZERO_PRESSURE;
    m_plateauPressureToDisplay = CONST_INITIAL_ZERO_PRESSURE;
    m_peepMeasure = CONST_INITIAL_ZERO_PRESSURE;
    m_cyclesPerMinuteMeasure = DEFAULT_CYCLE_PER_MINUTE_COMMAND;
    m_tidalVolumeMeasure = 0;  // TODO CONST DEFINE

    m_pressure = CONST_INITIAL_ZERO_PRESSURE;
    m_pressureCommand = CONST_INITIAL_ZERO_PRESSURE;

    m_cycleNb = 0;

    m_dt = mainController_COMPUTE_PERIOD_US;

    m_sumOfPressures = 0;
    m_numberOfPressures = 0;
    m_PlateauMeasureSum = 0;
    m_PlateauMeasureCount = 0;

    m_triggered = false;
    m_isPeepDetected = false;
    m_tidalVolumeAlreadyRead = false;
    m_plateauDurationMs = 0;

    m_lastEndOfRespirationDateMs = 0;

    m_inspiratoryValveAngle = VALVE_CLOSED_STATE;

    computeTickParameters();
    reachSafetyPosition();

    m_lastPressureValuesIndex = 0;
    for (uint8_t i = 0u; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0u;
    }

    m_lastBreathPeriodsMsIndex = 0;
    for (uint8_t i = 0u; i < NUMBER_OF_BREATH_PERIOD; i++) {
        m_lastBreathPeriodsMs[i] = (1000u * 60u) / m_cyclesPerMinuteCommand;
    }

    m_ventilationController->setup();
}

void MainController::initRespiratoryCycle() {
    DBG_DO(Serial.println("Init respiratory cycle");)
    m_cycleNb++;
    m_plateauPressureMeasure = 0;

    m_peakPressureMeasure = 0;
    m_triggered = false;
    m_isPeepDetected = false;
    m_tidalVolumeAlreadyRead = false;

    // Update new settings at the beginning of the respiratory cycle
    m_cyclesPerMinuteCommand = m_cyclesPerMinuteNextCommand;
    m_peepCommand = m_peepNextCommand;
    m_plateauPressureCommand = m_plateauPressureNextCommand;
    m_triggerModeEnabledCommand = m_triggerModeEnabledNextCommand;
    m_pressureTriggerOffsetCommand = m_pressureTriggerOffsetNextCommand;
    m_expiratoryTermCommand = m_expiratoryTermNextCommand;

    // Run setup of the controller only if different from previous.
    if (m_ventilationController != m_ventilationControllerNextCommand) {
        m_ventilationController = m_ventilationControllerNextCommand;
        m_ventilationController->setup();
    }

    computeTickParameters();

    for (uint8_t i = 0; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0;
    }
    m_lastPressureValuesIndex = 0;

    m_sumOfPressures = 0u;  // Check if used
    m_numberOfPressures = 0u;

    m_PlateauMeasureSum = 0u;
    m_PlateauMeasureCount = 0u;

    // Reset integral of the mass flow meter.
    MFM_read_milliliters(true);

    m_ventilationController->initCycle();
}

void MainController::compute() {
    // Update the cycle phase
    updatePhase(m_tick);
    m_tick = m_tick;

    // Compute metrics for alarms
    m_sumOfPressures += m_pressure;
    m_numberOfPressures++;

    // Act accordingly
    switch (m_phase) {
    case CyclePhases::INHALATION:
        inhale();
        m_inhalationLastPressure = m_pressure;
        break;

    case CyclePhases::EXHALATION:
        exhale();
        break;
    }

    alarmController.updateCoreData(m_tick, m_pressure, m_phase, m_subPhase, m_cycleNb);
    sendDataSnapshot(m_tick, m_pressure, m_phase, m_subPhase, inspiratoryValve.position,
                     expiratoryValve.position, blower.getSpeed() / 10u, getBatteryLevel(),
                     m_inspiratoryFlow, m_expiratoryFlow);

    executeCommands();

#ifdef MASS_FLOW_METER
    // Measure Volume only during inspiration. Add 100ms to allow valve to close completely.
    if (m_tick > m_tickPerInhalation + 10 && !m_tidalVolumeAlreadyRead) {
        m_tidalVolumeAlreadyRead = true;
        int32_t volume = MFM_read_milliliters(true);
        m_tidalVolumeMeasure =
            ((volume > 0xFFFE) || (volume < 0)) ? 0xFFFFu : static_cast<uint16_t>(volume);
    }

#else
    m_tidalVolumeMeasure = UINT16_MAX;
#endif
    simulatorCommunication();
}

void MainController::updatePhase(uint16_t m_tick) {
    if (m_tick < m_tickPerInhalation) {
        m_phase = CyclePhases::INHALATION;
        m_pressureCommand = m_plateauPressureCommand;

    } else {
        m_phase = CyclePhases::EXHALATION;
        m_pressureCommand = m_peepCommand;
    }
}


void MainController::inhale() {

    // Control loop
    m_ventilationController->inhale();

    // Update peak pressure and rebounce peak pressure.
    if (m_pressure > m_peakPressureMeasure) {
        m_peakPressureMeasure = m_pressure;
        m_rebouncePeakPressureMeasure = m_pressure;
    } else if (m_pressure < m_rebouncePeakPressureMeasure) {
        m_rebouncePeakPressureMeasure = m_pressure;
    }

    // Compute plateau at the end of the cycle TODO 20 = 200ms should be a parameter
    if (m_tick > m_tickPerInhalation - 20) {
        m_PlateauMeasureSum += m_pressure;
        m_PlateauMeasureCount += 1u;
    }
}



void MainController::exhale() {

    // Control loop
    m_ventilationController->exhale();

    // Compute the PEEP pressure
    uint16_t minValue = m_lastPressureValues[0u];
    uint16_t maxValue = m_lastPressureValues[0u];
    uint16_t totalValues = m_lastPressureValues[0u];
    for (uint8_t index = 1u; index < MAX_PRESSURE_SAMPLES; index++) {
        minValue = min(minValue, m_lastPressureValues[index]);
        maxValue = max(maxValue, m_lastPressureValues[index]);
        totalValues += m_lastPressureValues[index];
    }

    // Update PEEP value, when pressure is stable and close to target pressure
    if (((maxValue - minValue) < 5u) && (abs(m_pressure - m_peepCommand) < 30)) {
        m_isPeepDetected = true;
        m_peepMeasure = totalValues / MAX_PRESSURE_SAMPLES;
    }

    // This case is usefull when PEEP is never detected during the cycle
    if (!m_isPeepDetected) {
        m_peepMeasure = m_pressure;
    }
}

void MainController::endRespiratoryCycle() {
    // Compute the respiratory rate: average on NUMBER_OF_BREATH_PERIOD breaths
    // Don't calculate the first one (when starting up the machine)
    uint32_t currentMillis = millis();
    if (m_lastEndOfRespirationDateMs != 0u) {
        m_lastBreathPeriodsMs[m_lastBreathPeriodsMsIndex] =
            currentMillis - m_lastEndOfRespirationDateMs;
        m_lastBreathPeriodsMsIndex++;
        if (m_lastBreathPeriodsMsIndex >= NUMBER_OF_BREATH_PERIOD) {
            m_lastBreathPeriodsMsIndex = 0;
        }
        uint32_t sum = 0;
        for (uint8_t i = 0u; i < NUMBER_OF_BREATH_PERIOD; i++) {
            sum += m_lastBreathPeriodsMs[i];
        }
        // Add "+(sum-1u)" to round instead of truncate
        m_cyclesPerMinuteMeasure = (((NUMBER_OF_BREATH_PERIOD * 60u) * 1000u) + (sum - 1u)) / sum;
    }
    m_lastEndOfRespirationDateMs = currentMillis;

    // Plateau pressure is the mean pressure during plateau
    m_plateauPressureMeasure = m_PlateauMeasureSum / m_PlateauMeasureCount;

    checkCycleAlarm();

    // If plateau is not detected or is too close to PEEP, mark it as "unknown"
    if ((m_plateauPressureMeasure == 0u) || (abs(m_plateauPressureMeasure - m_peepMeasure) < 10)) {
        m_plateauPressureMeasure = UINT16_MAX;
    }

    m_plateauPressureToDisplay = m_plateauPressureMeasure;
    if (m_plateauPressureToDisplay == UINT16_MAX) {
        m_plateauPressureToDisplay = 0;
    }

    // Send snapshot of the firmware to the UI
    sendSnapshot();

    m_ventilationController->endCycle();
}

void MainController::simulatorCommunication() {
#if DEBUG == 2
  Serial.print(m_pressure);
  Serial.print(",");
  Serial.print(m_pressureCommand);
  Serial.print(",");
  Serial.print(m_inspiratoryFlow/100);// division by 100 for homogenous scale in debug
  Serial.print(",");
  Serial.print(m_expiratoryFlow/100);// division by 100 for homogenous scale in debug
  Serial.print(",");
  Serial.print(inspiratoryValve.command);
  Serial.print(",");
  Serial.print(expiratoryValve.command);
  Serial.print(",");
  Serial.print(blower.getSpeed()/10); // division by 10 for homogenous scale in debug
  Serial.println();
#endif
    
}

void MainController::updatePressure(int16_t p_currentPressure) {
    m_pressure = p_currentPressure;

    // Store the current pressure to compute aggregates
    m_lastPressureValues[m_lastPressureValuesIndex] = p_currentPressure;
    m_lastPressureValuesIndex++;

    // Start over if we reached the max samples number
    if (m_lastPressureValuesIndex >= MAX_PRESSURE_SAMPLES) {
        m_lastPressureValuesIndex = 0;
    }
}

void MainController::updateDt(int32_t p_dt) { m_dt = p_dt; }

void MainController::updateTick(uint32_t p_tick) { m_tick = p_tick; }

void MainController::updateInspiratoryFlow(int32_t p_currentInspiratoryFlow) {
    // TODO check if value is valid.
    m_inspiratoryFlow = p_currentInspiratoryFlow;
}

void MainController::updateExpiratoryFlow(int32_t p_currentExpiratoryFlow) {
    // TODO check if value is valid.
    m_expiratoryFlow = p_currentExpiratoryFlow;
}


void MainController::computeTickParameters() {
    // Inspiratory term is always 10. Expiratory term is between 10 and 60 (default 20).
    // The folowing calculation is equivalent of  1000 * (10 / (10 + m_expiratoryTerm) * (60 /
    // m_cyclesPerMinute).
    m_plateauDurationMs =
        ((10000u / (10u + m_expiratoryTermCommand)) * 60u) / m_cyclesPerMinuteCommand;

    m_ticksPerCycle = 60u * (1000000u / mainController_COMPUTE_PERIOD_US) / m_cyclesPerMinuteCommand;
    m_tickPerInhalation = (m_plateauDurationMs * 1000000u / mainController_COMPUTE_PERIOD_US) / 1000u;
}

void MainController::executeCommands() {
    if (m_pressure > ALARM_THRESHOLD_MAX_PRESSURE) {
        inspiratoryValve.close();
        expiratoryValve.open();
        alarmController.detectedAlarm(RCM_SW_18, m_cycleNb, ALARM_THRESHOLD_MAX_PRESSURE,
                                      m_pressure);
    } else {
        alarmController.notDetectedAlarm(RCM_SW_18);
    }
    inspiratoryValve.execute();
    expiratoryValve.execute();
}

void MainController::checkCycleAlarm() {
    // RCM-SW-1 + RCM-SW-14 : Check if plateau is reached
    uint16_t minPlateauBeforeAlarm =
        (m_plateauPressureCommand * (100u - ALARM_THRESHOLD_DIFFERENCE_PERCENT)) / 100u;
    uint16_t maxPlateauBeforeAlarm =
        (m_plateauPressureCommand * (100u + ALARM_THRESHOLD_DIFFERENCE_PERCENT)) / 100u;
    if ((m_plateauPressureMeasure < minPlateauBeforeAlarm)
        || (m_plateauPressureMeasure > maxPlateauBeforeAlarm)) {
        alarmController.detectedAlarm(RCM_SW_1, m_cycleNb, m_plateauPressureCommand, m_pressure);
        alarmController.detectedAlarm(RCM_SW_14, m_cycleNb, m_plateauPressureCommand, m_pressure);
    } else {
        alarmController.notDetectedAlarm(RCM_SW_1);
        alarmController.notDetectedAlarm(RCM_SW_14);
    }

    // RCM-SW-2 + RCM-SW-19 : Check is mean pressure was < 2 cmH2O
    uint16_t meanPressure = m_sumOfPressures / m_numberOfPressures;
    if (meanPressure <= ALARM_THRESHOLD_MIN_PRESSURE) {
        alarmController.detectedAlarm(RCM_SW_2, m_cycleNb, ALARM_THRESHOLD_MIN_PRESSURE,
                                      m_pressure);
        alarmController.detectedAlarm(RCM_SW_19, m_cycleNb, ALARM_THRESHOLD_MIN_PRESSURE,
                                      m_pressure);
    } else {
        alarmController.notDetectedAlarm(RCM_SW_2);
        alarmController.notDetectedAlarm(RCM_SW_19);
    }

    // RCM-SW-3 + RCM-SW-15
    uint16_t PeepBeforeAlarm = m_peepCommand - ALARM_THRESHOLD_DIFFERENCE_PRESSURE;
    uint16_t maxPeepBeforeAlarm = m_peepCommand + ALARM_THRESHOLD_DIFFERENCE_PRESSURE;
    if ((m_peepMeasure < PeepBeforeAlarm) || (m_peepMeasure > maxPeepBeforeAlarm)) {
        alarmController.detectedAlarm(RCM_SW_3, m_cycleNb, m_peepCommand, m_pressure);
        alarmController.detectedAlarm(RCM_SW_15, m_cycleNb, m_peepCommand, m_pressure);
    } else {
        alarmController.notDetectedAlarm(RCM_SW_3);
        alarmController.notDetectedAlarm(RCM_SW_15);
    }
}

void MainController::reachSafetyPosition() {
    inspiratoryValve.open();
    expiratoryValve.open();
    executeCommands();
}

void MainController::stop() {
    digitalWrite(PIN_LED_START, LED_START_INACTIVE);
    blower.stop();
    sendStoppedMessage(mmH2OtoCmH2O(m_peakPressureNextCommand),
                       mmH2OtoCmH2O(m_plateauPressureNextCommand), mmH2OtoCmH2O(m_peepNextCommand),
                       m_cyclesPerMinuteNextCommand, m_expiratoryTermNextCommand,
                       m_triggerModeEnabledNextCommand, m_pressureTriggerOffsetNextCommand);
    // When stopped, open the valves
    reachSafetyPosition();
    // Stop alarms related to breathing cycle
    alarmController.notDetectedAlarm(RCM_SW_1);
    alarmController.notDetectedAlarm(RCM_SW_2);
    alarmController.notDetectedAlarm(RCM_SW_3);
    alarmController.notDetectedAlarm(RCM_SW_14);
    alarmController.notDetectedAlarm(RCM_SW_15);
    alarmController.notDetectedAlarm(RCM_SW_18);
    alarmController.notDetectedAlarm(RCM_SW_19);
}

void MainController::sendSnapshot() {
    // Send the next command, because command has not been updated yet (will be at the beginning of
    // the next cycle)
    sendMachineStateSnapshot(m_cycleNb, mmH2OtoCmH2O(m_peakPressureNextCommand),
                             mmH2OtoCmH2O(m_plateauPressureNextCommand),
                             mmH2OtoCmH2O(m_peepNextCommand), m_cyclesPerMinuteNextCommand,
                             m_peakPressureMeasure, m_plateauPressureToDisplay, m_peepMeasure,
                             alarmController.triggeredAlarms(), m_tidalVolumeMeasure,
                             m_expiratoryTermNextCommand, m_triggerModeEnabledNextCommand,
                             m_pressureTriggerOffsetNextCommand, m_cyclesPerMinuteMeasure);
}

void MainController::onCycleDecrease() {
    DBG_DO(Serial.println("Cycle --");)

    m_cyclesPerMinuteNextCommand = m_cyclesPerMinuteNextCommand - 1;

    if (m_cyclesPerMinuteNextCommand < CONST_MIN_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MIN_CYCLE;
    }

    // Send acknoledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteNextCommand);
}

void MainController::onCycleIncrease() {
    DBG_DO(Serial.println("Cycle ++");)

    m_cyclesPerMinuteNextCommand = m_cyclesPerMinuteNextCommand + 1;

    if (m_cyclesPerMinuteNextCommand > CONST_MAX_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MAX_CYCLE;
    }

    // Send acknoledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteNextCommand);
}

// cppcheck-suppress unusedFunction
void MainController::onCycleSet(uint16_t p_cpm) {
    if (p_cpm < CONST_MIN_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MIN_CYCLE;
    } else if (p_cpm > CONST_MAX_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MAX_CYCLE;
    } else {
        m_cyclesPerMinuteNextCommand = p_cpm;
    }

    // Send acknoledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteNextCommand);
}

void MainController::onPeepPressureDecrease() {
    DBG_DO(Serial.println("Peep Pressure --");)

    m_peepNextCommand = m_peepNextCommand - 10u;

    if (m_peepNextCommand < CONST_MIN_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MIN_PEEP_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(3, m_peepNextCommand);
}

void MainController::onPeepPressureIncrease() {
    DBG_DO(Serial.println("Peep Pressure ++");)

    m_peepNextCommand = m_peepNextCommand + 10u;

    if (m_peepNextCommand > CONST_MAX_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MAX_PEEP_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(3, m_peepNextCommand);
}

// cppcheck-suppress unusedFunction
void MainController::onPeepSet(uint16_t p_peep) {
    if (p_peep > CONST_MAX_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MAX_PEEP_PRESSURE;
    } else if (p_peep < CONST_MIN_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MIN_PEEP_PRESSURE;
    } else {
        m_peepNextCommand = p_peep;
    }

    // Send acknoledgment to the UI
    sendControlAck(3, m_peepNextCommand);
}

void MainController::onPlateauPressureDecrease() {
    DBG_DO(Serial.println("Plateau Pressure --");)

    m_plateauPressureNextCommand = m_plateauPressureNextCommand - 10u;

    if (m_plateauPressureNextCommand < CONST_MIN_PLATEAU_PRESSURE) {
        m_plateauPressureNextCommand = CONST_MIN_PLATEAU_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(2, m_plateauPressureNextCommand);
}

void MainController::onPlateauPressureIncrease() {
    DBG_DO(Serial.println("Plateau Pressure ++");)

    m_plateauPressureNextCommand = m_plateauPressureNextCommand + 10u;

    m_plateauPressureNextCommand =
        min(m_plateauPressureNextCommand, static_cast<uint16_t>(CONST_MAX_PLATEAU_PRESSURE));

    if (m_plateauPressureNextCommand > m_peakPressureNextCommand) {
        m_peakPressureNextCommand = m_plateauPressureNextCommand;
    }

    // Send acknoledgment to the UI
    sendControlAck(2, m_plateauPressureNextCommand);
}

// cppcheck-suppress unusedFunction
void MainController::onPlateauPressureSet(uint16_t p_plateauPressure) {
    if (p_plateauPressure > CONST_MAX_PLATEAU_PRESSURE) {
        m_plateauPressureNextCommand = CONST_MAX_PLATEAU_PRESSURE;
    } else if (p_plateauPressure < CONST_MIN_PLATEAU_PRESSURE) {
        m_plateauPressureNextCommand = CONST_MIN_PLATEAU_PRESSURE;
    } else {
        m_plateauPressureNextCommand = p_plateauPressure;
    }

    // Send acknoledgment to the UI
    sendControlAck(2, m_plateauPressureNextCommand);
}

void MainController::onPeakPressureDecrease() {
    DBG_DO(Serial.println("Peak Pressure --");)
    // TODO : remove this !! Only for debug
    m_peakPressureNextCommand = 20;
    m_ventilationControllerNextCommand = &pcBipapController;
}

void MainController::onPeakPressureIncrease() {
    DBG_DO(Serial.println("Peak Pressure ++");)
    // TODO : remove this !! Only for debug
    m_ventilationControllerNextCommand = &pcCmvController;
    m_peakPressureNextCommand = 0;
}

// cppcheck-suppress unusedFunction
void MainController::onExpiratoryTermSet(uint16_t p_expiratoryTerm) {
    if (p_expiratoryTerm > CONST_MAX_EXPIRATORY_TERM) {
        m_expiratoryTermNextCommand = CONST_MAX_EXPIRATORY_TERM;
    } else if (p_expiratoryTerm < CONST_MIN_EXPIRATORY_TERM) {
        m_expiratoryTermNextCommand = CONST_MIN_EXPIRATORY_TERM;
    } else {
        m_expiratoryTermNextCommand = p_expiratoryTerm;
    }

    // Send acknoledgment to the UI
    sendControlAck(5, m_expiratoryTermNextCommand);
}

// cppcheck-suppress unusedFunction
void MainController::onTriggerEnabledSet(uint16_t p_triggerEnabled) {
    if ((p_triggerEnabled == 0u) || (p_triggerEnabled == 1u)) {
        m_triggerModeEnabledNextCommand = p_triggerEnabled;
    }

    // Send acknoledgment to the UI
    sendControlAck(6, m_triggerModeEnabledNextCommand);
}

// cppcheck-suppress unusedFunction
void MainController::onTriggerOffsetSet(uint16_t p_triggerOffset) {
    if (p_triggerOffset > CONST_MAX_TRIGGER_OFFSET) {
        m_pressureTriggerOffsetNextCommand = CONST_MAX_TRIGGER_OFFSET;
        // cppcheck-suppress unsignedLessThanZero
    } else if (p_triggerOffset < CONST_MIN_TRIGGER_OFFSET) {
        m_pressureTriggerOffsetNextCommand = CONST_MIN_TRIGGER_OFFSET;
    } else {
        m_pressureTriggerOffsetNextCommand = p_triggerOffset;
    }

    // Send acknoledgment to the UI
    sendControlAck(7, m_pressureTriggerOffsetNextCommand);
}