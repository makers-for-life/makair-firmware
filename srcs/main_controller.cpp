/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file main_controller.cpp
 * @brief Core logic to control the breathing cycle
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/main_controller.h"

// Internal
#include "../includes/cpu_load.h"

// INITIALISATION =============================================================

MainController mainController;

static const int32_t INVALID_ERROR_MARKER = INT32_MIN;

// FUNCTIONS ==================================================================

MainController::MainController() {
    m_tick = 0;
    m_phase = CyclePhases::INHALATION;

    m_inspiratoryFlow = 0;
    m_expiratoryFlow = 0;
    m_currentDeliveredVolume = 0;
    m_expiratoryVolume = 0;

    m_cyclesPerMinuteCommand = DEFAULT_CYCLE_PER_MINUTE_COMMAND;
    m_cyclesPerMinuteNextCommand = DEFAULT_CYCLE_PER_MINUTE_COMMAND;
    m_peakPressureCommand = DEFAULT_PEAK_PRESSURE_COMMAND;
    m_peakPressureNextCommand = DEFAULT_PEAK_PRESSURE_COMMAND;
    m_plateauPressureCommand = DEFAULT_PLATEAU_COMMAND;
    m_plateauPressureNextCommand = DEFAULT_PLATEAU_COMMAND;
    m_peepCommand = DEFAULT_PEEP_COMMAND;
    m_peepNextCommand = DEFAULT_PEEP_COMMAND;
    m_tidalVolumeCommand = DEFAULT_TIDAL_VOLUME_COMMAND;
    m_tidalVolumeNextCommand = DEFAULT_TIDAL_VOLUME_COMMAND;
    m_plateauDurationCommand = DEFAULT_PLATEAU_DURATION_COMMAND;
    m_plateauDurationNextCommand = DEFAULT_PLATEAU_DURATION_COMMAND;
    m_inspiratoryTriggerFlowCommand = DEFAULT_INSPIRATORY_TRIGGER_FLOW_COMMAND;
    m_inspiratoryTriggerFlowNextCommand = DEFAULT_INSPIRATORY_TRIGGER_FLOW_COMMAND;
    m_expiratoryTriggerFlowCommand = DEFAULT_EXPIRATORY_TRIGGER_FLOW_COMMAND;
    m_expiratoryTriggerFlowNextCommand = DEFAULT_EXPIRATORY_TRIGGER_FLOW_COMMAND;
    m_tiMinCommand = DEFAULT_MIN_INSPIRATION_DURATION_COMMAND;
    m_tiMinNextCommand = DEFAULT_MIN_INSPIRATION_DURATION_COMMAND;
    m_tiMaxCommand = DEFAULT_MAX_INSPIRATION_DURATION_COMMAND;
    m_tiMaxNextCommand = DEFAULT_MAX_INSPIRATION_DURATION_COMMAND;
    m_targetInspiratoryFlowCommand = DEFAULT_TARGET_FLOW_COMMAND;
    m_targetInspiratoryFlowNextCommand = DEFAULT_TARGET_FLOW_COMMAND;
    m_inspiratoryDurationCommand = DEFAULT_INSPIRATORY_DURATION;
    m_inspiratoryDurationNextCommand = DEFAULT_INSPIRATORY_DURATION;

    m_pressureTriggerOffsetCommand = DEFAULT_TRIGGER_OFFSET;
    m_pressureTriggerOffsetNextCommand = DEFAULT_TRIGGER_OFFSET;
    m_triggerModeEnabledCommand = TRIGGER_MODE_ENABLED_BY_DEFAULT;
    m_triggerModeEnabledNextCommand = TRIGGER_MODE_ENABLED_BY_DEFAULT;
    m_expiratoryTermCommand = DEFAULT_EXPIRATORY_TERM_COMMAND;
    m_expiratoryTermNextCommand = DEFAULT_EXPIRATORY_TERM_COMMAND;

    m_ventilationControllersTable[PC_CMV] = &pcCmvController;
    m_ventilationControllersTable[PC_AC] = &pcAcController;
    m_ventilationControllersTable[VC_CMV] = &vcCmvController;
    m_ventilationControllersTable[PC_VSAI] = &pcVsaiController;
    m_ventilationControllersTable[VC_AC] = &vcAcController;

    m_ventilationControllerMode = PC_AC;

    m_ventilationController = m_ventilationControllersTable[m_ventilationControllerMode];
    m_ventilationControllerNextCommand = m_ventilationControllersTable[m_ventilationControllerMode];

    m_lastEndOfRespirationDateMs = 0;
    m_peakPressureMeasure = CONST_INITIAL_ZERO_PRESSURE;
    m_rebouncePeakPressureMeasure = 0;
    m_inhalationLastPressure = 0;
    m_plateauPressureMeasure = CONST_INITIAL_ZERO_PRESSURE;
    m_plateauPressureToDisplay = CONST_INITIAL_ZERO_PRESSURE;
    m_peepMeasure = CONST_INITIAL_ZERO_PRESSURE;
    m_cyclesPerMinuteMeasure = DEFAULT_CYCLE_PER_MINUTE_COMMAND;
    m_tidalVolumeMeasure = CONST_INITIAL_ZERO_VOLUME;

    m_pressure = CONST_INITIAL_ZERO_PRESSURE;
    m_pressureCommand = CONST_INITIAL_ZERO_PRESSURE;

    m_cycleNb = 0;

    m_dt = MAIN_CONTROLLER_COMPUTE_PERIOD_MICROSECONDS;

    m_sumOfPressures = 0;
    m_numberOfPressures = 0;
    m_PlateauMeasureSum = 0;
    m_PlateauMeasureCount = 0;

    m_triggered = false;
    m_isPeepDetected = false;
    m_tidalVolumeAlreadyRead = false;
    m_plateauDurationMs = 0;

    m_inspiratoryValveAngle = VALVE_CLOSED_STATE;

    computeTickParameters();

    m_lastPressureValuesIndex = 0;
    for (int8_t i = 0u; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0;
    }

    m_lastBreathPeriodsMsIndex = 0;
    for (uint8_t i = 0u; i < NUMBER_OF_BREATH_PERIOD; i++) {
        m_lastBreathPeriodsMs[i] = (1000u * 60u) / m_cyclesPerMinuteCommand;
    }
}

void MainController::setup() {
    DBG_DO(Serial.println(VERSION);)
    DBG_DO(Serial.println("Setup the controller");)

    reachSafetyPosition();

    m_ventilationController->setup();

    alarmController.updateEnabledAlarms(m_ventilationController->enabledAlarms());

}

void MainController::initRespiratoryCycle() {
    m_expiratoryVolume = 0;
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
    m_tidalVolumeCommand = m_tidalVolumeNextCommand;
    m_plateauDurationCommand = m_plateauDurationNextCommand;
    m_inspiratoryTriggerFlowCommand = m_inspiratoryTriggerFlowNextCommand;
    m_expiratoryTriggerFlowCommand = m_expiratoryTriggerFlowNextCommand;
    m_tiMaxCommand = m_tiMaxNextCommand;
    m_tiMinCommand = m_tiMinNextCommand;
    m_targetInspiratoryFlowCommand = m_targetInspiratoryFlowNextCommand;
    m_inspiratoryDurationCommand = m_inspiratoryDurationNextCommand;

    // Run setup of the controller only if different from previous cycle
    if (m_ventilationController != m_ventilationControllerNextCommand) {
        m_ventilationController = m_ventilationControllerNextCommand;
        m_ventilationController->setup();
        alarmController.updateEnabledAlarms(m_ventilationController->enabledAlarms());
    }

    computeTickParameters();

    for (int8_t i = 0; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0;
    }
    m_lastPressureValuesIndex = 0;

    m_sumOfPressures = 0u;
    m_numberOfPressures = 0u;

    m_PlateauMeasureSum = 0u;
    m_PlateauMeasureCount = 0u;

    m_ventilationController->initCycle();
}

void MainController::compute() {
    // Update the cycle phase
    updatePhase();

    // Compute metrics for alarms
    m_sumOfPressures += static_cast<uint32_t>(m_pressure);
    m_numberOfPressures++;

    // Store last pressure values only every 10ms
    uint32_t moduloValue = max(1u, (10u / MAIN_CONTROLLER_COMPUTE_PERIOD_MS));
    if (m_tick % moduloValue == 0) {
        // Store the current pressure to compute aggregates
        m_lastPressureValues[m_lastPressureValuesIndex] = m_pressure;
        m_lastPressureValuesIndex++;

        // Start over if we reached the max samples number
        if (m_lastPressureValuesIndex >= static_cast<uint16_t>(MAX_PRESSURE_SAMPLES)) {
            m_lastPressureValuesIndex = 0;
        }
    }

    // Act accordingly
    switch (m_phase) {
    case CyclePhases::INHALATION:
        inhale();
        m_inhalationLastPressure = m_pressure;
        break;

    case CyclePhases::EXHALATION:
        exhale();
        break;

    default:
        // Do nothing
        break;
    }

    alarmController.updateCoreData(m_tick, m_pressure, m_phase, m_cycleNb);

    // Send data snaphshot only every 10 ms
    if ((m_tick % moduloValue) == 0u) {
        sendDataSnapshot(m_tick, max((int16_t)0, m_pressure), m_phase, inspiratoryValve.position,
                         expiratoryValve.position, blower.getSpeed() / 100u, getBatteryLevel(),
                         max(int32_t(0), m_inspiratoryFlow / 10),
                         max(int32_t(0), m_expiratoryFlow / 10));
    }

    executeCommands();

#ifdef MASS_FLOW_METER_ENABLED
    // Measure volume only during inspiration
    // Add 100 ms to allow valve to close completely
    if ((m_tick > (m_ticksPerInhalation + (100u / MAIN_CONTROLLER_COMPUTE_PERIOD_MS)))
        && !m_tidalVolumeAlreadyRead) {
        m_tidalVolumeAlreadyRead = true;
        int32_t volume = m_currentDeliveredVolume;
        m_tidalVolumeMeasure =
            ((volume > 0xFFFE) || (volume < 0)) ? 0xFFFFu : static_cast<uint16_t>(volume);
    }

#else
    m_tidalVolumeMeasure = UINT16_MAX;
#endif
    printDebugValues();
}

void MainController::updatePhase() {
    if (m_tick < m_ticksPerInhalation) {
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

    // Update peak pressure and rebounce peak pressure
    if (m_pressure > m_peakPressureMeasure) {
        m_peakPressureMeasure = max((int16_t)0, m_pressure);
        m_rebouncePeakPressureMeasure = m_pressure;
    } else if (m_pressure < m_rebouncePeakPressureMeasure) {
        m_rebouncePeakPressureMeasure = m_pressure;
    } else {
        // Do nothing
    }

    // Compute plateau at the end of the cycle
    if (m_tick > (m_ticksPerInhalation - (200u / MAIN_CONTROLLER_COMPUTE_PERIOD_MS))) {
        m_PlateauMeasureSum += m_pressure;
        m_PlateauMeasureCount += 1u;
    }
}

void MainController::exhale() {
    // Control loop
    m_ventilationController->exhale();

    // Compute the PEEP pressure
    int16_t minValue = m_lastPressureValues[0u];
    int16_t maxValue = m_lastPressureValues[0u];
    int16_t totalValues = m_lastPressureValues[0u];
    for (int8_t index = 1u; index < MAX_PRESSURE_SAMPLES; index++) {
        minValue = min(minValue, m_lastPressureValues[index]);
        maxValue = max(maxValue, m_lastPressureValues[index]);
        totalValues += m_lastPressureValues[index];
    }

    // Update PEEP value, when pressure is stable and close to target pressure
    if (((maxValue - minValue) < 5) && (abs(m_pressure - m_peepCommand) < 30)) {
        m_isPeepDetected = true;
        m_peepMeasure = max(0, totalValues / MAX_PRESSURE_SAMPLES);
    }

    // This case is usefull when PEEP is never detected during the cycle
    if (!m_isPeepDetected) {
        m_peepMeasure = max(static_cast<int16_t>(0), m_pressure);
    }
}

void MainController::endRespiratoryCycle(uint32_t p_currentMillis) {
    // Compute the respiratory rate: average on NUMBER_OF_BREATH_PERIOD breaths
    uint32_t currentMillis = p_currentMillis;
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
    m_lastEndOfRespirationDateMs = currentMillis;

    // Plateau pressure is the mean pressure during plateau
    m_plateauPressureMeasure = max(0, static_cast<int16_t>(m_PlateauMeasureSum)
                                          / static_cast<int16_t>(m_PlateauMeasureCount));

    checkCycleAlarm();

    m_plateauPressureToDisplay = m_plateauPressureMeasure;

    // Send telemetry machine state snapshot message
    sendMachineState();

    m_ventilationController->endCycle();
}

void MainController::printDebugValues() {
#if DEBUG == 2
    Serial.print(m_pressure);
    Serial.print(",");
    Serial.print(m_pressureCommand);
    Serial.print(",");
    Serial.print(m_inspiratoryFlow / 100);  // division by 100 for homogenous scale in debug
    Serial.print(",");
    Serial.print(m_expiratoryFlow / 100);  // division by 100 for homogenous scale in debug
    Serial.print(",");
    Serial.print(inspiratoryValve.command);
    Serial.print(",");
    Serial.print(expiratoryValve.command);
    Serial.print(",");
    Serial.print(blower.getSpeed() / 10);  // division by 10 for homogenous scale in debug
    Serial.println();
#endif
}

void MainController::updatePressure(int16_t p_currentPressure) { m_pressure = p_currentPressure; }

void MainController::updateDt(int32_t p_dt) { m_dt = p_dt; }

void MainController::updateTick(uint32_t p_tick) { m_tick = p_tick; }

void MainController::updateInspiratoryFlow(int32_t p_currentInspiratoryFlow) {
    if (p_currentInspiratoryFlow != MASS_FLOW_ERROR_VALUE) {
        m_inspiratoryFlow = p_currentInspiratoryFlow;
    }
}

// cppcheck-suppress unusedFunction
void MainController::updateExpiratoryFlow(int32_t p_currentExpiratoryFlow) {
    if (p_currentExpiratoryFlow != MASS_FLOW_ERROR_VALUE) {
        m_expiratoryFlow = p_currentExpiratoryFlow;
    }
}

// cppcheck-suppress unusedFunction
void MainController::updateFakeExpiratoryFlow() {
    int32_t openning;
    if (m_ventilationControllerMode == PC_VSAI) {
        openning = expiratoryValve.positionLinear;
    } else {
        openning = expiratoryValve.position;
    }

    if (openning == 125) {
        m_expiratoryFlow = 0;
    } else {
        int32_t aMultiplyBy100 =
            ((((-585 * openning) * openning) / 100000) + ((118 * openning) / 100)) - 62;
        int32_t bMultiplyBy100 = ((((195 * openning) * openning) / 100) + 33300 - (489 * openning));
        int32_t cMultiplyBy100 = 279000 - (2170 * openning);

        int32_t p = m_pressure;
        m_expiratoryFlow =
            (((aMultiplyBy100 * p) * p) + (bMultiplyBy100 * p) + cMultiplyBy100) / 100;
    }

    m_expiratoryVolume += ((m_expiratoryFlow / 60) * m_dt) / 1000;
}

void MainController::updateCurrentDeliveredVolume(int32_t p_currentDeliveredVolume) {
    if (p_currentDeliveredVolume != MASS_FLOW_ERROR_VALUE) {
        m_currentDeliveredVolume = p_currentDeliveredVolume;
    }
}

void MainController::computeTickParameters() {
    // Inspiratory term is always 10. Expiratory term is between 10 and 60 (default 20).
    // The folowing calculation is equivalent of  1000 * (10 / (10 + m_expiratoryTerm) * (60 /
    // m_cyclesPerMinute).
    m_plateauDurationMs =
        ((10000u / (10u + m_expiratoryTermCommand)) * 60u) / m_cyclesPerMinuteCommand;

    m_ticksPerCycle =
        60u * (1000000u / MAIN_CONTROLLER_COMPUTE_PERIOD_MICROSECONDS) / m_cyclesPerMinuteCommand;
    m_ticksPerInhalation =
        (m_plateauDurationMs * 1000000u / MAIN_CONTROLLER_COMPUTE_PERIOD_MICROSECONDS) / 1000u;
}

void MainController::executeCommands() {
    if (m_pressure > ALARM_THRESHOLD_MAX_PRESSURE) {
        inspiratoryValve.close();
        expiratoryValve.open();
#if !SIMULATION
        alarmController.detectedAlarm(RCM_SW_18, m_cycleNb, ALARM_THRESHOLD_MAX_PRESSURE,
                                      m_pressure);
#endif
    } else {
#if !SIMULATION
        alarmController.notDetectedAlarm(RCM_SW_18);
#endif
    }
    inspiratoryValve.execute();
    expiratoryValve.execute();
}

void MainController::checkCycleAlarm() {
#if !SIMULATION
    // RCM-SW-1 + RCM-SW-14: check if plateau is reached
    int16_t minPlateauBeforeAlarm =
        (m_plateauPressureCommand * (100 - ALARM_THRESHOLD_DIFFERENCE_PERCENT)) / 100;
    int16_t maxPlateauBeforeAlarm =
        (m_plateauPressureCommand * (100 + ALARM_THRESHOLD_DIFFERENCE_PERCENT)) / 100;
    if ((m_plateauPressureMeasure < minPlateauBeforeAlarm)
        || (m_plateauPressureMeasure > maxPlateauBeforeAlarm)) {
        alarmController.detectedAlarm(RCM_SW_1, m_cycleNb, m_plateauPressureCommand, m_pressure);
        alarmController.detectedAlarm(RCM_SW_14, m_cycleNb, m_plateauPressureCommand, m_pressure);
    } else {
        alarmController.notDetectedAlarm(RCM_SW_1);
        alarmController.notDetectedAlarm(RCM_SW_14);
    }

    // RCM-SW-2 + RCM-SW-19: check is mean pressure was < 2 cmH2O
    int16_t meanPressure = m_sumOfPressures / m_numberOfPressures;
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
    int16_t PeepBeforeAlarm = m_peepCommand - ALARM_THRESHOLD_DIFFERENCE_PRESSURE;
    int16_t maxPeepBeforeAlarm = m_peepCommand + ALARM_THRESHOLD_DIFFERENCE_PRESSURE;
    if ((m_peepMeasure < PeepBeforeAlarm) || (m_peepMeasure > maxPeepBeforeAlarm)) {
        alarmController.detectedAlarm(RCM_SW_3, m_cycleNb, m_peepCommand, m_pressure);
        alarmController.detectedAlarm(RCM_SW_15, m_cycleNb, m_peepCommand, m_pressure);
    } else {
        alarmController.notDetectedAlarm(RCM_SW_3);
        alarmController.notDetectedAlarm(RCM_SW_15);
    }
#endif
}

void MainController::reachSafetyPosition() {
    inspiratoryValve.open();
    expiratoryValve.open();
    executeCommands();
}

void MainController::sendStopMessageToUi() {
#if !SIMULATION
    sendStoppedMessage(
        mmH2OtoCmH2O(m_peakPressureNextCommand), mmH2OtoCmH2O(m_plateauPressureNextCommand),
        mmH2OtoCmH2O(m_peepNextCommand), m_cyclesPerMinuteNextCommand, m_expiratoryTermNextCommand,
        m_triggerModeEnabledNextCommand, m_pressureTriggerOffsetNextCommand,
        alarmController.isSnoozed(), readCpuLoadPercent(), m_ventilationControllerMode,
        m_inspiratoryTriggerFlowNextCommand, m_expiratoryTriggerFlowNextCommand, m_tiMinNextCommand,
        m_tiMaxNextCommand, 0u, 0u, 0u, 0u, 0u, 0u, m_tidalVolumeNextCommand, 0u, 0u,
        m_plateauDurationNextCommand, 0u, static_cast<uint8_t>(m_targetInspiratoryFlowNextCommand/1000), m_inspiratoryDurationNextCommand);
#endif
}

void MainController::stop(uint32_t p_currentMillis) {
    blower.stop();
    sendStopMessageToUi();
    // When stopped, open the valves
    reachSafetyPosition();
    m_lastEndOfRespirationDateMs = p_currentMillis;

#if !SIMULATION
    // Stop alarms related to breathing cycle
    alarmController.notDetectedAlarm(RCM_SW_1);
    alarmController.notDetectedAlarm(RCM_SW_2);
    alarmController.notDetectedAlarm(RCM_SW_3);
    alarmController.notDetectedAlarm(RCM_SW_14);
    alarmController.notDetectedAlarm(RCM_SW_15);
    alarmController.notDetectedAlarm(RCM_SW_18);
    alarmController.notDetectedAlarm(RCM_SW_19);
    digitalWrite(PIN_LED_START, LED_START_INACTIVE);
#endif
}

void MainController::sendMachineState() {
#if !SIMULATION
    // Send the next command, because command has not been updated yet (will be at the beginning of
    // the next cycle)
    sendMachineStateSnapshot(
        m_cycleNb, mmH2OtoCmH2O(m_peakPressureNextCommand),
        mmH2OtoCmH2O(m_plateauPressureNextCommand), mmH2OtoCmH2O(m_peepNextCommand),
        m_cyclesPerMinuteNextCommand, m_peakPressureMeasure, m_plateauPressureToDisplay,
        m_peepMeasure, alarmController.triggeredAlarms(), m_tidalVolumeMeasure,
        m_expiratoryTermNextCommand, m_triggerModeEnabledNextCommand,
        m_pressureTriggerOffsetNextCommand, m_cyclesPerMinuteMeasure, alarmController.isSnoozed(),
        readCpuLoadPercent(), m_ventilationControllerMode, m_inspiratoryTriggerFlowNextCommand,
        m_expiratoryTriggerFlowNextCommand, m_tiMinNextCommand, m_tiMaxNextCommand, 0u, 0u, 0u, 0u,
        0u, 0u, m_tidalVolumeNextCommand, 0u, 0u, m_plateauDurationNextCommand, 0u, static_cast<uint8_t>(m_targetInspiratoryFlowNextCommand/1000), m_inspiratoryDurationNextCommand);
#endif
}

void MainController::onVentilationModeSet(uint16_t p_ventilationControllerMode) {
    if (m_ventilationControllerMode >= 1
        && m_ventilationControllerMode <= NUMBER_OF_VENTILATION_MODES) {
        m_ventilationControllerMode = static_cast<VentilationModes>(p_ventilationControllerMode);
        m_ventilationControllerNextCommand =
            m_ventilationControllersTable[m_ventilationControllerMode];
    }
#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(1, m_ventilationControllerMode);
#endif
}

void MainController::onCycleDecrease() {
    DBG_DO(Serial.println("Cycle --");)

    m_cyclesPerMinuteNextCommand = m_cyclesPerMinuteNextCommand - 1u;

    if (m_cyclesPerMinuteNextCommand < CONST_MIN_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MIN_CYCLE;
    }
#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteNextCommand);
#endif
}

void MainController::onCycleIncrease() {
    DBG_DO(Serial.println("Cycle ++");)

    m_cyclesPerMinuteNextCommand = m_cyclesPerMinuteNextCommand + 1u;

    if (m_cyclesPerMinuteNextCommand > CONST_MAX_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MAX_CYCLE;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteNextCommand);
#endif
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

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteNextCommand);
#endif
}

void MainController::onPeepPressureDecrease() {
    DBG_DO(Serial.println("Peep Pressure --");)

    if (m_peepNextCommand >= 10) {
        m_peepNextCommand = m_peepNextCommand - 10;
    } else {
        // Do nothing
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(3, m_peepNextCommand);
#endif
}

void MainController::onPeepPressureIncrease() {
    DBG_DO(Serial.println("Peep Pressure ++");)

    // Peep target should be lower than plateau target
    if ((m_peepNextCommand + 10) < m_plateauPressureNextCommand) {
        m_peepNextCommand = m_peepNextCommand + 10;
    }

    if (m_peepNextCommand > CONST_MAX_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MAX_PEEP_PRESSURE;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(3, m_peepNextCommand);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onPeepSet(int16_t p_peep) {
    if (p_peep > CONST_MAX_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MAX_PEEP_PRESSURE;
        // cppcheck-suppress unsignedLessThanZero ; CONST_MIN_PEEP_PRESSURE might not be equal to 0
    } else if (p_peep < CONST_MIN_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MIN_PEEP_PRESSURE;
    } else {
        m_peepNextCommand = p_peep;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(3, m_peepNextCommand);
#endif
}

void MainController::onPlateauPressureDecrease() {
    DBG_DO(Serial.println("Plateau Pressure --");)

    if ((m_plateauPressureNextCommand - 10) > m_peepNextCommand) {
        m_plateauPressureNextCommand = m_plateauPressureNextCommand - 10;
    }

    if (m_plateauPressureNextCommand < CONST_MIN_PLATEAU_PRESSURE) {
        m_plateauPressureNextCommand = CONST_MIN_PLATEAU_PRESSURE;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(2, m_plateauPressureNextCommand);
#endif
}

void MainController::onPlateauPressureIncrease() {
    DBG_DO(Serial.println("Plateau Pressure ++");)

    m_plateauPressureNextCommand = m_plateauPressureNextCommand + 10;

    m_plateauPressureNextCommand =
        min(m_plateauPressureNextCommand, static_cast<int16_t>(CONST_MAX_PLATEAU_PRESSURE));

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(2, m_plateauPressureNextCommand);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onPlateauPressureSet(int16_t p_plateauPressure) {
    if (p_plateauPressure > CONST_MAX_PLATEAU_PRESSURE) {
        m_plateauPressureNextCommand = CONST_MAX_PLATEAU_PRESSURE;
    } else if (p_plateauPressure < CONST_MIN_PLATEAU_PRESSURE) {
        m_plateauPressureNextCommand = CONST_MIN_PLATEAU_PRESSURE;
    } else {
        m_plateauPressureNextCommand = p_plateauPressure;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(2, m_plateauPressureNextCommand);
#endif
}

void MainController::onPeakPressureDecrease() {
    DBG_DO(Serial.println("Peak Pressure --");)
#if DEBUG != 0
    switch (m_ventilationControllerMode) {
    case PC_CMV:
        m_ventilationControllerMode = PC_CMV;
        break;
    case PC_AC:
        m_ventilationControllerMode = PC_CMV;
        break;
    case VC_CMV:
        m_ventilationControllerMode = PC_AC;
        break;
    case PC_VSAI:
        m_ventilationControllerMode = VC_CMV;
        break;
    }
    m_ventilationControllerNextCommand = m_ventilationControllersTable[m_ventilationControllerMode];

    m_peakPressureNextCommand = m_ventilationControllerMode * 10;
#endif
}

void MainController::onPeakPressureIncrease() {
    DBG_DO(Serial.println("Peak Pressure ++");)
#if DEBUG != 0
    switch (m_ventilationControllerMode) {
    case PC_CMV:
        m_ventilationControllerMode = PC_AC;
        break;
    case PC_AC:
        m_ventilationControllerMode = VC_CMV;
        break;
    case VC_CMV:
        m_ventilationControllerMode = PC_VSAI;
        break;
    case PC_VSAI:
        m_ventilationControllerMode = PC_VSAI;
        break;
    }
    m_ventilationControllerNextCommand = m_ventilationControllersTable[m_ventilationControllerMode];

    m_peakPressureNextCommand = m_ventilationControllerMode * 10;
#endif
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

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(5, m_expiratoryTermNextCommand);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onTriggerModeEnabledSet(uint16_t p_triggerEnabled) {
    if ((p_triggerEnabled == 0u) || (p_triggerEnabled == 1u)) {
        m_triggerModeEnabledNextCommand = p_triggerEnabled;
    }
#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(6, m_triggerModeEnabledNextCommand);
#endif
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

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(7, m_pressureTriggerOffsetNextCommand);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onInspiratoryTriggerFlowSet(uint16_t p_inspiratoryTriggerFlow) {
    if (p_inspiratoryTriggerFlow >= CONST_MIN_INSPIRATORY_TRIGGER_FLOW
        && p_inspiratoryTriggerFlow <= CONST_MAX_INSPIRATORY_TRIGGER_FLOW) {
        m_inspiratoryTriggerFlowNextCommand = p_inspiratoryTriggerFlow;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(10, m_inspiratoryTriggerFlowNextCommand);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onExpiratoryTriggerFlowSet(uint16_t p_expiratoryTriggerFlow) {
    if (p_expiratoryTriggerFlow >= CONST_MIN_EXPIRATORY_TRIGGER_FLOW
        && p_expiratoryTriggerFlow <= CONST_MAX_EXPIRATORY_TRIGGER_FLOW) {
        m_expiratoryTriggerFlowNextCommand = p_expiratoryTriggerFlow;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(11, m_expiratoryTriggerFlowNextCommand);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onTiMinSet(uint16_t p_tiMin) {
    if (p_tiMin >= CONST_MIN_MIN_INSPIRATION_DURATION
        && p_tiMin <= CONST_MAX_MIN_INSPIRATION_DURATION) {
        m_tiMinNextCommand = p_tiMin;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(12, m_tiMinNextCommand);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onTiMaxSet(uint16_t p_tiMax) {
    if (p_tiMax >= CONST_MIN_MAX_INSPIRATION_DURATION
        && p_tiMax <= CONST_MAX_MAX_INSPIRATION_DURATION) {
        m_tiMaxNextCommand = p_tiMax;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(13, m_tiMaxNextCommand);
#endif
}

// cppcheck-suppress unusedFunction

void MainController::onLowInspiratoryMinuteVolumeAlarmThresholdSet(
    uint16_t p_lowInspiratoryMinuteVolumeAlarmThreshold) {
    // TODO

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(14, p_lowInspiratoryMinuteVolumeAlarmThreshold);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onHighInspiratoryMinuteVolumeAlarmThresholdSet(
    uint16_t p_highInspiratoryMinuteVolumeAlarmThreshold) {
    // TODO

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(15, p_highInspiratoryMinuteVolumeAlarmThreshold);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onLowExpiratoryMinuteVolumeAlarmThresholdSet(
    uint16_t p_lowExpiratoryMinuteVolumeAlarmThreshold) {
    // TODO

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(16, p_lowExpiratoryMinuteVolumeAlarmThreshold);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onHighExpiratoryMinuteVolumeAlarmThresholdSet(
    uint16_t p_highExpiratoryMinuteVolumeAlarmThreshold) {
    // TODO

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(17, p_highExpiratoryMinuteVolumeAlarmThreshold);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onLowExpiratoryRateAlarmThresholdSet(
    uint16_t p_lowExpiratoryRateAlarmThreshold) {
    // TODO

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(18, p_lowExpiratoryRateAlarmThreshold);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onHighExpiratoryRateAlarmThresholdSet(
    uint16_t p_highExpiratoryRateAlarmThreshold) {
    // TODO

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(19, p_highExpiratoryRateAlarmThreshold);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onTargetTidalVolumeSet(uint16_t p_targetTidalVolume) {
    if (p_targetTidalVolume >= CONST_MIN_TIDAL_VOLUME
        && p_targetTidalVolume <= CONST_MAX_TIDAL_VOLUME)
        m_tidalVolumeNextCommand = p_targetTidalVolume;
#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(20, m_tidalVolumeNextCommand);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onLowTidalVolumeAlarmTresholdSet(uint16_t p_lowTidalVolumeAlarmTreshold) {
    // TODO

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(21, p_lowTidalVolumeAlarmTreshold);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onHighTidalVolumeAlarmTresholdSet(uint16_t p_highTidalVolumeAlarmTreshold) {
    // TODO

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(22, p_highTidalVolumeAlarmTreshold);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onPlateauDurationSet(uint16_t p_plateauDuration) {
    if (p_plateauDuration >= CONST_MIN_PLATEAU_DURATION
        && p_plateauDuration <= CONST_MAX_PLATEAU_DURATION) {
        m_plateauDurationNextCommand = p_plateauDuration;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(23, m_plateauDurationNextCommand);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onLeakAlarmThresholdSet(uint16_t p_leakAlarmThreshold) {
    // TODO

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(24, p_leakAlarmThreshold);
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onTargetInspiratoryFlow(uint16_t p_targetInspiratoryFlow) {
    int32_t temporaryValue = static_cast<uint32_t>(p_targetInspiratoryFlow)*1000;
    if (temporaryValue >= CONST_MIN_INSPIRATORY_FLOW && temporaryValue <= CONST_MAX_INSPIRATORY_FLOW){
        m_targetInspiratoryFlowNextCommand = temporaryValue;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(25, static_cast<uint16_t>(m_targetInspiratoryFlowNextCommand/1000));
#endif
}

// cppcheck-suppress unusedFunction
void MainController::onInspiratoryDuration(uint16_t p_inspiratoryDuration) {
    if (p_inspiratoryDuration >= CONST_MIN_INSPIRATORY_DURATION && p_inspiratoryDuration <= CONST_MAX_INSPIRATORY_DURATION){
        m_inspiratoryDurationNextCommand = p_inspiratoryDuration;
    }

#if !SIMULATION
    // Send acknowledgment to the UI
    sendControlAck(26, m_inspiratoryDurationNextCommand);
#endif
}
