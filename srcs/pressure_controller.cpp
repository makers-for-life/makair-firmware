/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure_controller.cpp
 * @brief Core logic to control the breathing cycle
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/pressure_controller.h"

// External
#include <algorithm>

// Internal
#include "../includes/alarm_controller.h"
#include "../includes/battery.h"
#include "../includes/config.h"
#include "../includes/debug.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/parameters.h"
#include "../includes/pressure_valve.h"
#include "../includes/telemetry.h"

static const int32_t INVALID_ERROR_MARKER = INT32_MIN;

// INITIALISATION =============================================================

PressureController pController;

// FUNCTIONS ==================================================================

PressureController::PressureController()
    : m_cyclesPerMinuteCommand(INITIAL_CYCLE_NUMBER),
      m_minPeepCommand(DEFAULT_MIN_PEEP_COMMAND),                   // [mmH20]
      m_maxPlateauPressureCommand(DEFAULT_MAX_PLATEAU_COMMAND),     // [mmH20]
      m_maxPeakPressureCommand(DEFAULT_MAX_PEAK_PRESSURE_COMMAND),  // [mmH20]
      m_cyclesPerMinute(INITIAL_CYCLE_NUMBER),
      m_maxPeakPressure(CONST_MAX_PEAK_PRESSURE),        // [mmH20]
      m_maxPlateauPressure(CONST_MAX_PLATEAU_PRESSURE),  // [mmH20]
      m_minPeep(CONST_MIN_PEEP_PRESSURE),
      m_pressure(CONST_INITIAL_ZERO_PRESSURE),
      m_inhalationLastPressure(CONST_INITIAL_ZERO_PRESSURE),
      m_peakPressure(CONST_INITIAL_ZERO_PRESSURE),
      m_plateauPressure(CONST_INITIAL_ZERO_PRESSURE),
      m_peep(CONST_INITIAL_ZERO_PRESSURE),
      m_measuredCyclesPerMinute(INITIAL_CYCLE_NUMBER),
      m_phase(CyclePhases::INHALATION),
      m_subPhase(CycleSubPhases::INSPIRATION),
      m_blower(nullptr),
      m_alarmController(nullptr),
      m_blower_increment(0),
      m_cycleNb(0),
      m_dt(0),
      m_pressureCommand(0),
      PC_inspiratory_PID_integral(0),
      PC_inspiratory_PID_LastError(0),
      PC_expiratory_PID_integral(0),
      inspiratoryValveLastAperture(0),
      PC_expiratory_PID_LastError(0),
      m_startPlateauComputation(false),
      m_plateauComputed(false),
      m_lastPressureValuesIndex(0),
      m_sumOfPressures(0),
      m_numberOfPressures(0),
      m_plateauStartTime(0u),
      m_squarePlateauSum(0),
      m_squarePlateauCount(0),
      PC_inspiratory_PID_last_errorsIndex(0),
      PC_expiratory_PID_last_errorsIndex(0),
      PC_expiratory_PID_fast_mode(true),
      PC_inspiratory_PID_fast_mode(true),
      m_tick(0),
      expiratoryValveLastAperture(0),
      m_triggered(false),
      m_pressureTriggerOffset(0),
      m_isPeepDetected(false),
      m_triggerModeEnabled(false),
      m_plateauDurationMs(0),
      m_lastBreathPeriodsMsIndex(0),
      m_lastEndOfRespirationDateMs(0),
      m_ExpiratoryTerm(DEFAULT_EXPIRATORY_TERM_COMMAND),
      m_peakBlowerValveAngle(VALVE_CLOSED_STATE) {
    computeTickParameters();
    for (uint8_t i = 0u; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0u;
    }
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        PC_inspiratory_PID_last_errors[i] = 0u;
        PC_expiratory_PID_last_errors[i] = 0u;
    }
    for (uint8_t i = 0u; i < NUMBER_OF_BREATH_PERIOD; i++) {
        m_lastBreathPeriodsMs[i] = (1000u * 60u) / m_cyclesPerMinute;
    }
}

PressureController::PressureController(int16_t p_cyclesPerMinute,
                                       int16_t p_minPeepCommand,
                                       int16_t p_maxPlateauPressure,
                                       int16_t p_maxPeakPressure,
                                       const PressureValve& p_blower_valve,
                                       const PressureValve& p_patient_valve,
                                       AlarmController* p_alarmController,
                                       Blower* p_blower)
    : m_cyclesPerMinuteCommand(p_cyclesPerMinute),
      m_minPeepCommand(p_minPeepCommand),
      m_maxPlateauPressureCommand(p_maxPlateauPressure),
      m_maxPeakPressureCommand(DEFAULT_MAX_PEAK_PRESSURE_COMMAND),
      m_cyclesPerMinute(p_cyclesPerMinute),
      m_maxPeakPressure(p_maxPeakPressure),
      m_maxPlateauPressure(p_maxPlateauPressure),
      m_minPeep(p_minPeepCommand),
      m_pressure(CONST_INITIAL_ZERO_PRESSURE),
      m_inhalationLastPressure(CONST_INITIAL_ZERO_PRESSURE),
      // cppcheck-suppress misra-c2012-12.3
      m_peakPressure(CONST_INITIAL_ZERO_PRESSURE),
      m_plateauPressure(CONST_INITIAL_ZERO_PRESSURE),
      m_peep(CONST_INITIAL_ZERO_PRESSURE),
      m_measuredCyclesPerMinute(INITIAL_CYCLE_NUMBER),
      m_phase(CyclePhases::INHALATION),
      m_subPhase(CycleSubPhases::INSPIRATION),
      // cppcheck-suppress misra-c2012-12.3
      m_inspiratoryValve(p_blower_valve),
      m_expiratoryValve(p_patient_valve),
      m_blower(p_blower),
      m_alarmController(p_alarmController),
      m_blower_increment(0),
      m_cycleNb(0),
      m_dt(0),
      m_pressureCommand(0),
      PC_inspiratory_PID_integral(0),
      PC_inspiratory_PID_LastError(0),
      // cppcheck-suppress misra-c2012-12.3
      PC_expiratory_PID_integral(0),
      inspiratoryValveLastAperture(0),
      PC_expiratory_PID_LastError(0),
      m_startPlateauComputation(false),
      m_plateauComputed(false),
      m_lastPressureValuesIndex(0),
      m_sumOfPressures(0),
      m_numberOfPressures(0),
      m_plateauStartTime(0u),
      m_squarePlateauSum(0),
      m_squarePlateauCount(0),
      PC_inspiratory_PID_last_errorsIndex(0),
      PC_expiratory_PID_last_errorsIndex(0),
      PC_expiratory_PID_fast_mode(true),
      PC_inspiratory_PID_fast_mode(true),
      m_tick(0),
      expiratoryValveLastAperture(0),
      m_triggered(false),
      m_pressureTriggerOffset(0),
      m_isPeepDetected(false),
      m_triggerModeEnabled(false),
      m_plateauDurationMs(0),
      m_lastBreathPeriodsMsIndex(0),
      m_lastEndOfRespirationDateMs(0),
      m_ExpiratoryTerm(DEFAULT_EXPIRATORY_TERM_COMMAND),
      m_peakBlowerValveAngle(VALVE_CLOSED_STATE) {
    computeTickParameters();
    
    for (uint8_t i = 0u; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0u;
    }
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        PC_inspiratory_PID_last_errors[i] = 0u;
        PC_expiratory_PID_last_errors[i] = 0u;
    }
    for (uint8_t i = 0u; i < NUMBER_OF_BREATH_PERIOD; i++) {
        m_lastBreathPeriodsMs[i] = (1000u * 60u) / m_cyclesPerMinute;
    }
}

void PressureController::setup() {
    DBG_DO(Serial.println(VERSION);)
    DBG_DO(Serial.println("mise en secu initiale");)

    m_inspiratoryValve.close();
    m_expiratoryValve.close();

    m_inspiratoryValve.execute();
    m_expiratoryValve.execute();

    m_peakPressure = 0;
    m_plateauPressure = 0;
    m_peep = 0;

    m_cycleNb = 0;

    m_pressureTriggerOffset = DEFAULT_TRIGGER_OFFSET;

    m_triggerModeEnabled = TRIGGER_MODE_ENABLED_BY_DEFAULT;

    computeTickParameters();

    m_lastEndOfRespirationDateMs = 0;
}

void PressureController::initRespiratoryCycle() {
    m_phase = CyclePhases::INHALATION;
    setSubPhase(CycleSubPhases::INSPIRATION, 0);
    m_cycleNb++;
    m_plateauPressure = 0;

    // Reset PID values
    PC_inspiratory_PID_integral = 0;
    PC_expiratory_PID_integral = 0;
    PC_inspiratory_PID_LastError = m_maxPlateauPressureCommand - m_minPeepCommand;
    PC_expiratory_PID_LastError = m_minPeepCommand - m_maxPlateauPressureCommand;
    PC_inspiratory_PID_fast_mode = true;
    PC_expiratory_PID_fast_mode = true;
    for (uint8_t i = 0; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        PC_inspiratory_PID_last_errors[i] = 0;
        PC_expiratory_PID_last_errors[i] = m_minPeepCommand - m_maxPlateauPressureCommand;
    }

    inspiratoryValveLastAperture = m_inspiratoryValve.maxAperture();
    expiratoryValveLastAperture = m_expiratoryValve.maxAperture();

    
    computeTickParameters();

    m_peakPressure = 0;
    m_triggered = false;
    m_isPeepDetected = false;

    DBG_AFFICHE_CSPCYCLE_CSPINSPI(m_ticksPerCycle, m_tickPerInhalation)


    // Update new settings at the beginning of the respiratory cycle
    m_cyclesPerMinute = m_cyclesPerMinuteCommand;
    m_minPeep = m_minPeepCommand;
    m_maxPlateauPressure = m_maxPlateauPressureCommand;

    // Apply blower ramp-up
    if (m_blower_increment >= 0) {
        m_blower->runSpeed(m_blower->getSpeed() + static_cast<uint16_t>(abs(m_blower_increment)));
    } else {
        // When blower increment is negative, we need to check that it is less than current speed
        // If not, it would result in an overflow
        if (static_cast<uint16_t>(abs(m_blower_increment)) < m_blower->getSpeed()) {
            m_blower->runSpeed(m_blower->getSpeed()
                               - static_cast<uint16_t>(abs(m_blower_increment)));
        } else {
            m_blower->runSpeed(MIN_BLOWER_SPEED);
        }
    }
    m_blower_increment = 0;


    for (uint8_t i = 0; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0;
    }
    m_lastPressureValuesIndex = 0;
    m_startPlateauComputation = false;
    m_plateauComputed = false;

    m_sumOfPressures = 0u;
    m_numberOfPressures = 0u;

    m_plateauStartTime = m_tickPerInhalation;

    m_squarePlateauSum = 0u;
    m_squarePlateauCount = 0u;

    
}

void PressureController::endRespiratoryCycle() {
    // Compute the respiratory rate: average on NUMBER_OF_BREATH_PERIOD breaths
    // Don't calculate the first one (when starting up the machine)
    if (m_lastEndOfRespirationDateMs != 0u) {
        m_lastBreathPeriodsMs[m_lastBreathPeriodsMsIndex] = millis() - m_lastEndOfRespirationDateMs;
        m_lastBreathPeriodsMsIndex++;
        if (m_lastBreathPeriodsMsIndex >= NUMBER_OF_BREATH_PERIOD) {
            m_lastBreathPeriodsMsIndex = 0;
        }
        uint32_t sum = 0;
        for (uint8_t i = 0u; i < NUMBER_OF_BREATH_PERIOD; i++) {
            sum += m_lastBreathPeriodsMs[i];
        }
        // Add "+(sum-1u)" to round instead of truncate
        m_measuredCyclesPerMinute = (((NUMBER_OF_BREATH_PERIOD * 60u) * 1000u) + (sum - 1u)) / sum;
    }
    m_lastEndOfRespirationDateMs = millis();

    // Plateau pressure is the mean pressure during plateau
    m_plateauPressure = m_squarePlateauSum / m_squarePlateauCount;
    updateOnlyBlower();

    checkCycleAlarm();

    // If plateau is not detected or is too close to PEEP, mark it as "unknown"
    if ((m_plateauPressure == 0u) || (abs(m_plateauPressure - m_peep) < 10)) {
        m_plateauPressure = UINT16_MAX;
    }

    // RCM-SW-18
    if (m_pressure <= ALARM_THRESHOLD_MAX_PRESSURE) {
        m_alarmController->notDetectedAlarm(RCM_SW_18);
    }

    uint16_t plateauPressureToDisplay = m_plateauPressure;
    if (plateauPressureToDisplay == UINT16_MAX) {
        plateauPressureToDisplay = 0;
    }

#ifdef MASS_FLOW_METER
    int32_t volume = MFM_read_milliliters(true);
    uint16_t telemetryVolume =
        ((volume > 0xFFFE) || (volume < 0)) ? 0xFFFFu : static_cast<uint16_t>(volume);
#else
    uint16_t telemetryVolume = UINT16_MAX;
#endif

    sendMachineStateSnapshot(m_cycleNb, mmH2OtoCmH2O(m_maxPeakPressureCommand),
                             mmH2OtoCmH2O(m_maxPlateauPressureCommand),
                             mmH2OtoCmH2O(m_minPeepCommand), m_cyclesPerMinuteCommand,
                             m_peakPressure, plateauPressureToDisplay, m_peep,
                             m_alarmController->triggeredAlarms(), telemetryVolume,
                             m_ExpiratoryTerm, m_triggerModeEnabled, m_pressureTriggerOffset);
}

void PressureController::updatePressure(int16_t p_currentPressure) {
    m_pressure = p_currentPressure;

    // Store the current pressure to compute aggregates
    m_lastPressureValues[m_lastPressureValuesIndex] = p_currentPressure;
    m_lastPressureValuesIndex++;

    // Start over if we reached the max samples number
    if (m_lastPressureValuesIndex >= MAX_PRESSURE_SAMPLES) {
        m_lastPressureValuesIndex = 0;
    }
}

void PressureController::compute(uint16_t p_tick) {
    // Update the cycle phase
    updatePhase(p_tick);
    m_tick = p_tick;

    // Compute metrics for alarms
    m_sumOfPressures += m_pressure;
    m_numberOfPressures++;

    // Act accordingly
    switch (m_subPhase) {
    case CycleSubPhases::INSPIRATION: {
        inhale();
        m_inhalationLastPressure = m_pressure;
        break;
    }
    case CycleSubPhases::HOLD_INSPIRATION: {
        plateau();
        m_inhalationLastPressure = m_pressure;
        break;
    }
    case CycleSubPhases::EXHALE:
    default: {
        exhale();
        // Plateau happens with delay related to the pressure command.
        computePlateau(p_tick);
        break;
    }
    }

    // RCM-SW-18
    if (m_pressure > ALARM_THRESHOLD_MAX_PRESSURE) {
        m_alarmController->detectedAlarm(RCM_SW_18, m_cycleNb, ALARM_THRESHOLD_MAX_PRESSURE,
                                         m_pressure);
    }

    DBG_PHASE_PRESSION(m_cycleNb, p_tick, 1u, m_phase, m_subPhase, m_pressure,
                       m_inspiratoryValve.command, m_inspiratoryValve.position,
                       m_expiratoryValve.command, m_expiratoryValve.position)

    m_alarmController->updateCoreData(p_tick, m_pressure, m_phase, m_subPhase, m_cycleNb);
    sendDataSnapshot(p_tick, m_pressure, m_phase, m_subPhase, m_inspiratoryValve.position,
                     m_expiratoryValve.position, m_blower->getSpeed() / 10u, getBatteryLevel());

    executeCommands();
}

// cppcheck-suppress unusedFunction
void PressureController::computePlateau(uint16_t p_tick) {
    uint16_t minValue = m_lastPressureValues[0u];
    uint16_t maxValue = m_lastPressureValues[0u];
    uint16_t totalValues = m_lastPressureValues[0u];

    for (uint8_t index = 1u; index < MAX_PRESSURE_SAMPLES; index++) {
        minValue = min(minValue, m_lastPressureValues[index]);
        maxValue = max(maxValue, m_lastPressureValues[index]);
        totalValues += m_lastPressureValues[index];
    }

    uint16_t diff = (maxValue - minValue);

    // Start computing plateau pressure when:
    // - the last pressure values were close enough
    // - the hold inspiration phase is about to end
    // - plateau pressure computation was not already started
    if (!m_plateauComputed && (diff < 10u) && (p_tick >= ((m_tickPerInhalation * 95u) / 100u))) {
        m_startPlateauComputation = true;
    }

    // Stop computing plateau pressure when pressure drops
    if (m_startPlateauComputation && (diff > 10u)) {
        m_startPlateauComputation = false;
        m_plateauComputed = true;
    }

    if (m_startPlateauComputation) {
        m_plateauPressure = totalValues / MAX_PRESSURE_SAMPLES;
    }
}

void PressureController::onCycleDecrease() {
    DBG_DO(Serial.println("Cycle --");)

    m_cyclesPerMinuteCommand--;

    if (m_cyclesPerMinuteCommand < CONST_MIN_CYCLE) {
        m_cyclesPerMinuteCommand = CONST_MIN_CYCLE;
    }
    // Update internal values depending on cpm
    computeTickParameters();

    // Send acknoledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteCommand);
}

void PressureController::onCycleIncrease() {
    DBG_DO(Serial.println("Cycle ++");)

    m_cyclesPerMinuteCommand++;

    if (m_cyclesPerMinuteCommand > CONST_MAX_CYCLE) {
        m_cyclesPerMinuteCommand = CONST_MAX_CYCLE;
    }
    // Update internal values depending on cpm
    computeTickParameters();

    // Send acknoledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteCommand);
}

// cppcheck-suppress unusedFunction
void PressureController::onCycleSet(uint16_t cpm) {
    if (cpm < CONST_MIN_CYCLE) {
        m_cyclesPerMinuteCommand = CONST_MIN_CYCLE;
    } else if (cpm > CONST_MAX_CYCLE) {
        m_cyclesPerMinuteCommand = CONST_MAX_CYCLE;
    } else {
        m_cyclesPerMinuteCommand = cpm;
    }
    // Update values depending on cpm
    computeTickParameters();

    // Send acknoledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteCommand);
}

void PressureController::onPeepPressureDecrease() {
    DBG_DO(Serial.println("Peep Pressure --");)

    m_minPeepCommand = m_minPeepCommand - 10u;

    if (m_minPeepCommand < CONST_MIN_PEEP_PRESSURE) {
        m_minPeepCommand = CONST_MIN_PEEP_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(3, m_minPeepCommand);
}

void PressureController::onPeepPressureIncrease() {
    DBG_DO(Serial.println("Peep Pressure ++");)

    m_minPeepCommand = m_minPeepCommand + 10u;

    if (m_minPeepCommand > CONST_MAX_PEEP_PRESSURE) {
        m_minPeepCommand = CONST_MAX_PEEP_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(3, m_minPeepCommand);
}

// cppcheck-suppress unusedFunction
void PressureController::onPeepSet(uint16_t peep) {
    if (peep > CONST_MAX_PEEP_PRESSURE) {
        m_minPeepCommand = CONST_MAX_PEEP_PRESSURE;
    } else if (peep < CONST_MIN_PEEP_PRESSURE) {
        m_minPeepCommand = CONST_MIN_PEEP_PRESSURE;
    } else {
        m_minPeepCommand = peep;
    }

    // Send acknoledgment to the UI
    sendControlAck(3, m_minPeepCommand);
}

void PressureController::onPlateauPressureDecrease() {
    DBG_DO(Serial.println("Plateau Pressure --");)

    m_maxPlateauPressureCommand = m_maxPlateauPressureCommand - 10u;

    if (m_maxPlateauPressureCommand < CONST_MIN_PLATEAU_PRESSURE) {
        m_maxPlateauPressureCommand = CONST_MIN_PLATEAU_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(2, m_maxPlateauPressureCommand);
}

void PressureController::onPlateauPressureIncrease() {
    DBG_DO(Serial.println("Plateau Pressure ++");)

    m_maxPlateauPressureCommand = m_maxPlateauPressureCommand + 10u;

    m_maxPlateauPressureCommand =
        min(m_maxPlateauPressureCommand, static_cast<uint16_t>(CONST_MAX_PLATEAU_PRESSURE));

    if (m_maxPlateauPressureCommand > m_maxPeakPressureCommand) {
        m_maxPeakPressureCommand = m_maxPlateauPressureCommand;
    }

    // Send acknoledgment to the UI
    sendControlAck(2, m_maxPlateauPressureCommand);
}

// cppcheck-suppress unusedFunction
void PressureController::onPlateauPressureSet(uint16_t plateauPressure) {
    if (plateauPressure > CONST_MAX_PLATEAU_PRESSURE) {
        m_maxPlateauPressureCommand = CONST_MAX_PLATEAU_PRESSURE;
    } else if (plateauPressure < CONST_MIN_PLATEAU_PRESSURE) {
        m_maxPlateauPressureCommand = CONST_MIN_PLATEAU_PRESSURE;
    } else {
        m_maxPlateauPressureCommand = plateauPressure;
    }

    // Send acknoledgment to the UI
    sendControlAck(2, m_maxPlateauPressureCommand);
}

void PressureController::onPeakPressureDecrease(uint8_t p_decrement) {
    DBG_DO(Serial.println("Peak Pressure --");)

    m_maxPeakPressureCommand = m_maxPeakPressureCommand - p_decrement;

    m_maxPeakPressureCommand =
        max(m_maxPeakPressureCommand, static_cast<uint16_t>(CONST_MIN_PEAK_PRESSURE));

    if (m_maxPeakPressureCommand < m_maxPlateauPressureCommand) {
        m_maxPlateauPressureCommand = m_maxPeakPressureCommand;
    }

    // Send acknoledgment to the UI
    sendControlAck(1, m_maxPeakPressureCommand);
}

void PressureController::onPeakPressureIncrease(uint8_t p_increment) {
    DBG_DO(Serial.println("Peak Pressure ++");)

    m_maxPeakPressureCommand = m_maxPeakPressureCommand + p_increment;

    if (m_maxPeakPressureCommand > CONST_MAX_PEAK_PRESSURE) {
        m_maxPeakPressureCommand = CONST_MAX_PEAK_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(1, m_maxPeakPressureCommand);
}

// cppcheck-suppress unusedFunction
void PressureController::onPeakPressureSet(uint16_t peakPressure) {
    if (peakPressure > CONST_MAX_PEAK_PRESSURE) {
        m_maxPeakPressureCommand = CONST_MAX_PEAK_PRESSURE;
    } else if (peakPressure < CONST_MIN_PEAK_PRESSURE) {
        m_maxPeakPressureCommand = CONST_MIN_PEAK_PRESSURE;
    } else {
        m_maxPeakPressureCommand = peakPressure;
    }

    // Send acknoledgment to the UI
    sendControlAck(1, m_maxPeakPressureCommand);
}

// cppcheck-suppress unusedFunction
void PressureController::onExpiratoryTermSet(uint16_t ExpiratoryTerm) {
    if (ExpiratoryTerm > CONST_MAX_EXPIRATORY_TERM) {
        m_ExpiratoryTerm = CONST_MAX_EXPIRATORY_TERM;
    } else if (ExpiratoryTerm < CONST_MIN_EXPIRATORY_TERM) {
        m_ExpiratoryTerm = CONST_MIN_EXPIRATORY_TERM;
    } else {
        m_ExpiratoryTerm = ExpiratoryTerm;
    }
    // Update internal values depending on ExpiratoryTerm
    computeTickParameters();

    // Send acknoledgment to the UI
    sendControlAck(5, m_ExpiratoryTerm);
}

// cppcheck-suppress unusedFunction
void PressureController::onTriggerEnabledSet(uint16_t TriggerEnabled) {
    if ((TriggerEnabled == 0u) || (TriggerEnabled == 1u)) {
        m_triggerModeEnabled = TriggerEnabled;
    }

    // Send acknoledgment to the UI
    sendControlAck(6, m_triggerModeEnabled);
}

// cppcheck-suppress unusedFunction
void PressureController::onTriggerOffsetSet(uint16_t TriggerOffset) {
    if (TriggerOffset > CONST_MAX_TRIGGER_OFFSET) {
        m_pressureTriggerOffset = CONST_MAX_TRIGGER_OFFSET;
        // cppcheck-suppress unsignedLessThanZero
    } else if (TriggerOffset < CONST_MIN_TRIGGER_OFFSET) {
        m_pressureTriggerOffset = CONST_MIN_TRIGGER_OFFSET;
    } else {
        m_pressureTriggerOffset = TriggerOffset;
    }

    // Send acknoledgment to the UI
    sendControlAck(7, m_pressureTriggerOffset);
}

void PressureController::updatePhase(uint16_t p_tick) {
    if (p_tick < m_tickPerInhalation) {
        m_phase = CyclePhases::INHALATION;

        // -5 mmH2O is added to prevent peak pressure not reached in case the pressure is almost
        // reached. This is mandatory to help the blower regulation to converge
        uint16_t pressureToTest = m_maxPeakPressureCommand - 5u;

        if (p_tick < (m_tickPerInhalation * 80u) / 100u && (m_pressure < pressureToTest)) {
            if (m_subPhase != CycleSubPhases::HOLD_INSPIRATION) {
                m_pressureCommand = pressureToTest;
                setSubPhase(CycleSubPhases::INSPIRATION, p_tick);
            }
        } else {
            m_pressureCommand = m_maxPlateauPressureCommand;
            setSubPhase(CycleSubPhases::HOLD_INSPIRATION, p_tick);
        }
    } else {
        m_phase = CyclePhases::EXHALATION;
        m_pressureCommand = m_minPeepCommand;

        setSubPhase(CycleSubPhases::EXHALE, p_tick);
    }
}

void PressureController::inhale() {
    // Open the inspiratory valve using a PID
    m_peakBlowerValveAngle = pidBlower(m_pressureCommand, m_pressure, m_dt);
    m_inspiratoryValve.open(m_peakBlowerValveAngle);

    // Close the expiratory valve
    m_expiratoryValve.close();

    // Update the peak pressure
    m_peakPressure = max(m_pressure, m_peakPressure);
}

void PressureController::plateau() {
    // Keep the inspiratory valve open using a PID. The openning of the valve will be very small
    // during this phase.
    m_inspiratoryValve.open(pidBlower(m_pressureCommand, m_pressure, m_dt));

    // Compute mean plateau pressure only after the peak.
    if (m_pressure > m_peakPressure) {
        m_peakPressure = m_pressure;
        m_squarePlateauCount = 0;
        m_squarePlateauSum = 0;
    } else {
        m_squarePlateauSum += m_pressure;
        m_squarePlateauCount += 1u;
    }

    // Close the inspiratory valve
    m_expiratoryValve.close();
}

void PressureController::exhale() {
    // Close the inspiratory valve
    m_inspiratoryValve.close();

    // Open the valve expiratory so the patient can exhale outside
    m_expiratoryValve.open(pidPatient(m_pressureCommand, m_pressure, m_dt));

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
    if (((maxValue - minValue) < 5u) && (abs(m_pressure - m_minPeepCommand) < 30)) {
        m_isPeepDetected = true;
        m_peep = totalValues / MAX_PRESSURE_SAMPLES;
    }

    // This case is usefull when PEEP is never detected during the cycle
    if (!m_isPeepDetected) {
        m_peep = m_pressure;
    }

    // In case the pressure trigger mode is enabled, check if inspiratory trigger is raised
    if (m_triggerModeEnabled && m_isPeepDetected) {
        // m_peakPressure > CONST_MIN_PEAK_PRESSURE ensure that the patient is plugged on the
        // machine.
        if (static_cast<int32_t>(m_pressure)
                < (m_pressureCommand - static_cast<int32_t>(m_pressureTriggerOffset))
            && (m_peakPressure > CONST_MIN_PEAK_PRESSURE)) {
            m_triggered = true;
        }
    }
}

void PressureController::updateDt(int32_t p_dt) { m_dt = p_dt; }

void PressureController::updateOnlyBlower() {
    int16_t peakDelta = m_peakPressure - m_maxPeakPressureCommand;

    // Number of tick for the half ramp (120ms)
    int32_t halfRampNumberfTick = 1000 * 120 / static_cast<int32_t>(PCONTROLLER_COMPUTE_PERIOD_US);

    if (m_plateauStartTime < ((m_tickPerInhalation * 30u) / 100u)) {
        // Only case for decreasing the blower : ramping is too fast or overshooting is too high
        if ((m_plateauStartTime < static_cast<uint32_t>(abs(halfRampNumberfTick)))
            || ((peakDelta > 15) && (m_plateauStartTime < ((m_tickPerInhalation * 20u) / 100u)))
            || (peakDelta > 25)) {
            m_blower_increment = -100;
            DBG_DO(Serial.println("BLOWER -100");)
        } else {
            m_blower_increment = 0;
            DBG_DO(Serial.println("BLOWER 0");)
        }
    } else if (m_plateauStartTime < ((m_tickPerInhalation * 40u) / 100u)) {
        DBG_DO(Serial.println("BLOWER +0");)
        m_blower_increment = 0;
    } else if (m_plateauStartTime < ((m_tickPerInhalation * 50u) / 100u)) {
        m_blower_increment = +25;
        DBG_DO(Serial.println("BLOWER +25"));
    } else if (m_plateauStartTime < ((m_tickPerInhalation * 60u) / 100u)) {
        m_blower_increment = +50;
        DBG_DO(Serial.println("BLOWER +50"));
    } else {
        m_blower_increment = +100;
        DBG_DO(Serial.println("BLOWER +100"));
    }

    DBG_DO(Serial.print("Plateau Start time:");)
    DBG_DO(Serial.println(m_plateauStartTime);)
}

void PressureController::computeTickParameters() {
    // Inspiratory term is always 10. Expiratory term is between 10 and 60 (default 20).
    // The folowing calculation is equivalent of  1000 * (10 / (10 + m_ExpiratoryTerm) * (60 /
    // m_cyclesPerMinute).
    m_plateauDurationMs = ((10000u / (10u + m_ExpiratoryTerm)) * 60u) / m_cyclesPerMinute;

    m_ticksPerCycle = 60u * (1000000u / PCONTROLLER_COMPUTE_PERIOD_US) / m_cyclesPerMinute;
    m_tickPerInhalation = (m_plateauDurationMs * 1000000u / PCONTROLLER_COMPUTE_PERIOD_US) / 1000u;
}

void PressureController::executeCommands() {
    m_inspiratoryValve.execute();
    m_expiratoryValve.execute();
}

void PressureController::checkCycleAlarm() {
    // RCM-SW-1 + RCM-SW-14 : Check if plateau is reached
    uint16_t minPlateauBeforeAlarm =
        (m_maxPlateauPressureCommand * (100u - ALARM_THRESHOLD_DIFFERENCE_PERCENT)) / 100u;
    uint16_t maxPlateauBeforeAlarm =
        (m_maxPlateauPressureCommand * (100u + ALARM_THRESHOLD_DIFFERENCE_PERCENT)) / 100u;
    if ((m_plateauPressure < minPlateauBeforeAlarm)
        || (m_plateauPressure > maxPlateauBeforeAlarm)) {
        m_alarmController->detectedAlarm(RCM_SW_1, m_cycleNb, m_maxPlateauPressureCommand,
                                         m_pressure);
        m_alarmController->detectedAlarm(RCM_SW_14, m_cycleNb, m_maxPlateauPressureCommand,
                                         m_pressure);
    } else {
        m_alarmController->notDetectedAlarm(RCM_SW_1);
        m_alarmController->notDetectedAlarm(RCM_SW_14);
    }

    // RCM-SW-2 + RCM-SW-19 : Check is mean pressure was < 2 cmH2O
    uint16_t meanPressure = m_sumOfPressures / m_numberOfPressures;
    if (meanPressure <= ALARM_THRESHOLD_MIN_PRESSURE) {
        m_alarmController->detectedAlarm(RCM_SW_2, m_cycleNb, ALARM_THRESHOLD_MIN_PRESSURE,
                                         m_pressure);
        m_alarmController->detectedAlarm(RCM_SW_19, m_cycleNb, ALARM_THRESHOLD_MIN_PRESSURE,
                                         m_pressure);
    } else {
        m_alarmController->notDetectedAlarm(RCM_SW_2);
        m_alarmController->notDetectedAlarm(RCM_SW_19);
    }

    // RCM-SW-3 + RCM-SW-15
    uint16_t minPeepBeforeAlarm = m_minPeepCommand - ALARM_THRESHOLD_DIFFERENCE_PRESSURE;
    uint16_t maxPeepBeforeAlarm = m_minPeepCommand + ALARM_THRESHOLD_DIFFERENCE_PRESSURE;
    if ((m_peep < minPeepBeforeAlarm) || (m_peep > maxPeepBeforeAlarm)) {
        m_alarmController->detectedAlarm(RCM_SW_3, m_cycleNb, m_minPeepCommand, m_pressure);
        m_alarmController->detectedAlarm(RCM_SW_15, m_cycleNb, m_minPeepCommand, m_pressure);
    } else {
        m_alarmController->notDetectedAlarm(RCM_SW_3);
        m_alarmController->notDetectedAlarm(RCM_SW_15);
    }
}

void PressureController::setSubPhase(CycleSubPhases p_subPhase, uint16_t p_tick) {
    if ((m_subPhase == CycleSubPhases::INSPIRATION)
        && (p_subPhase == CycleSubPhases::HOLD_INSPIRATION)) {
        m_plateauStartTime = p_tick;
    }
    m_subPhase = p_subPhase;
}

int32_t PressureController::pidBlower(int32_t targetPressure, int32_t currentPressure, int32_t dt) {

    int32_t minAperture = m_inspiratoryValve.minAperture();
    int32_t maxAperture = m_inspiratoryValve.maxAperture();
    uint32_t blowerAperture;
    int32_t derivative = 0;
    int32_t smoothError = 0;
    int32_t totalValues = 0;
    int32_t proportionnalWeight;
    int32_t derivativeWeight;

    int32_t coefficientP;
    int32_t coefficientI;
    int32_t coefficientD;

    int32_t temporaryPC_inspiratory_PID_integral = 0;

    // Compute error
    int32_t error = targetPressure - currentPressure;

    // Windowing. It overides the parameter.h coefficients
    if (error < 0) {
        coefficientI = 200;
        coefficientP = 2500;
        coefficientD = 0;
    } else {
        coefficientI = 50;
        coefficientP = 2500;
        coefficientD = 0;
    }

    // Calculate Derivative part. Include a moving average on error for smoothing purpose
    PC_inspiratory_PID_last_errors[PC_inspiratory_PID_last_errorsIndex] = error;
    PC_inspiratory_PID_last_errorsIndex++;
    if (PC_inspiratory_PID_last_errorsIndex
        >= static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN)) {
        PC_inspiratory_PID_last_errorsIndex = 0;
    }
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        totalValues += PC_inspiratory_PID_last_errors[i];
    }
    smoothError =
        totalValues / static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN);

    // Fast mode ends at 20mmH20 from target. When changing from fast-mode to PID, Set the
    // integral to the previous value
    if (error < 20) {
        if (PC_inspiratory_PID_fast_mode) {
            proportionnalWeight = (coefficientP * error) / 1000;
            derivativeWeight = (coefficientD * derivative / 1000);
            PC_inspiratory_PID_integral =
                1000 * ((int32_t)inspiratoryValveLastAperture - maxAperture)
                    / (minAperture - maxAperture)
                - (proportionnalWeight + derivativeWeight);
        }
        PC_inspiratory_PID_fast_mode = false;
    }

    // In fast mode : everything is openned (Open loop)
    if (PC_inspiratory_PID_fast_mode) {
        // Ramp from 125 to 0 angle during 250ms.
        int32_t increment = 5 * ((int32_t)PCONTROLLER_COMPUTE_PERIOD_US) / 10000;
        if (inspiratoryValveLastAperture >= static_cast<uint32_t>(abs(increment))) {
            blowerAperture = max(
                minAperture,
                min(maxAperture, (static_cast<int32_t>(inspiratoryValveLastAperture) - increment)));
        } else {
            blowerAperture = 0;
        }
    }

    // In not fast mode the PID is used
    else {
        derivative =
            ((dt == 0)) ? 0 : ((1000000 * (PC_inspiratory_PID_LastError - smoothError)) / dt);

        temporaryPC_inspiratory_PID_integral =
            PC_inspiratory_PID_integral + ((coefficientI * error * dt) / 1000000);
        temporaryPC_inspiratory_PID_integral =
            max(PID_BLOWER_INTEGRAL_MIN,
                min(PID_BLOWER_INTEGRAL_MAX, temporaryPC_inspiratory_PID_integral));

        proportionnalWeight = ((coefficientP * error) / 1000);
        int32_t integralWeight = temporaryPC_inspiratory_PID_integral;
        derivativeWeight = coefficientD * derivative / 1000;

        int32_t blowerCommand = proportionnalWeight + integralWeight + derivativeWeight;
        blowerAperture =
            max(minAperture,
                min(maxAperture, maxAperture + (minAperture - maxAperture) * blowerCommand / 1000));
    }

    // If the valve is completly open or completly closed, dont update Integral
    if ((blowerAperture != static_cast<uint32_t>(minAperture))
        && (blowerAperture != static_cast<uint32_t>(maxAperture))) {
        PC_inspiratory_PID_integral = temporaryPC_inspiratory_PID_integral;
    }

    inspiratoryValveLastAperture = blowerAperture;
    PC_inspiratory_PID_LastError = smoothError;

    return blowerAperture;
}

int32_t
PressureController::pidPatient(int32_t targetPressure, int32_t currentPressure, int32_t dt) {

    int32_t minAperture = m_expiratoryValve.minAperture();
    int32_t maxAperture = m_expiratoryValve.maxAperture();
    uint32_t patientAperture;
    int32_t derivative = 0;
    int32_t smoothError = 0;
    int32_t totalValues = 0;
    int32_t temporaryPC_expiratory_PID_integral = 0;
    int32_t proportionnalWeight;
    int32_t derivativeWeight;

    int32_t coefficientP;
    int32_t coefficientI;
    int32_t coefficientD;

    // Compute error
    int32_t error = targetPressure + PID_PATIENT_SAFETY_PEEP_OFFSET - currentPressure;

    // Calculate Derivative part. Include a moving average on error for smoothing purpose
    PC_expiratory_PID_last_errors[PC_expiratory_PID_last_errorsIndex] = error;
    PC_expiratory_PID_last_errorsIndex++;
    if (PC_expiratory_PID_last_errorsIndex
        >= static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN)) {
        PC_expiratory_PID_last_errorsIndex = 0;
    }
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        totalValues += PC_expiratory_PID_last_errors[i];
    }
    smoothError =
        totalValues / static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN);
    derivative = (dt == 0) ? 0 : (1000000 * (smoothError - PC_expiratory_PID_LastError)) / dt;

    //  Windowing. It overides the parameter.h coefficients
    if (error < 0) {
        coefficientI = 50;
        coefficientP = 2500;
        coefficientD = 0;
    } else {
        // For a high peep, a lower KI is requiered. For Peep = 100mmH2O, KI = 120. For Peep =
        // 50mmH2O, KI = 250.
        coefficientI = ((-130 * ((int32_t)m_minPeepCommand)) / 50) + 380;
        coefficientP = 2500;
        coefficientD = 0;
    }

    // Fast mode ends at 30mmH20 from target. When changing from fast-mode to
    // PID, Set the integral to the previous value
    if (error > -30) {
        if (PC_expiratory_PID_fast_mode) {
            proportionnalWeight = (coefficientP * error) / 1000;
            derivativeWeight = (coefficientD * derivative / 1000);
            PC_expiratory_PID_integral = 1000 * ((int32_t)expiratoryValveLastAperture - maxAperture)
                                             / (maxAperture - minAperture)
                                         - (proportionnalWeight + derivativeWeight);
        }
        PC_expiratory_PID_fast_mode = false;
    }

    // Fast mode : open loop with ramp
    if (PC_expiratory_PID_fast_mode) {
        // Ramp from 125 to 0 angle during 250ms.
        int32_t increment = 5 * ((int32_t)PCONTROLLER_COMPUTE_PERIOD_US) / 10000;
        if (expiratoryValveLastAperture >= static_cast<uint32_t>(abs(increment))) {
            patientAperture = max(
                minAperture,
                min(maxAperture, (static_cast<int32_t>(expiratoryValveLastAperture) - increment)));
        } else {
            patientAperture = 0;
        }
    }

    // In not fast mode the PID is used
    else {
        temporaryPC_expiratory_PID_integral =
            PC_expiratory_PID_integral + ((coefficientI * error * dt) / 1000000);
        temporaryPC_expiratory_PID_integral =
            max(PID_PATIENT_INTEGRAL_MIN,
                min(PID_PATIENT_INTEGRAL_MAX, temporaryPC_expiratory_PID_integral));

        proportionnalWeight = ((coefficientP * error) / 1000);
        int32_t integralWeight = temporaryPC_expiratory_PID_integral;
        derivativeWeight = coefficientD * derivative / 1000;

        int32_t patientCommand = proportionnalWeight + integralWeight + derivativeWeight;

        patientAperture = max(
            minAperture,
            min(maxAperture, maxAperture + (maxAperture - minAperture) * patientCommand / 1000));
    }

    // If the valve is completly open or completly closed, dont update Integral
    if ((patientAperture != static_cast<uint32_t>(minAperture))
        && (patientAperture != static_cast<uint32_t>(maxAperture))) {
        PC_expiratory_PID_integral = temporaryPC_expiratory_PID_integral;
    }

    PC_expiratory_PID_LastError = smoothError;
    expiratoryValveLastAperture = patientAperture;

    return patientAperture;
}

void PressureController::reachSafetyPosition() {
    m_inspiratoryValve.open();
    m_inspiratoryValve.execute();
    m_expiratoryValve.open();
    m_expiratoryValve.execute();
}
