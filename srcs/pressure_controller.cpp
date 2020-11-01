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
#include "../includes/pressure.h"

static const int32_t INVALID_ERROR_MARKER = INT32_MIN;

// FUNCTIONS ==================================================================

PressureController::PressureController() {}

PressureController::PressureController(const PressureValve& p_inspiratory_valve,
                                       const PressureValve& p_expiratory_valve,
                                       AlarmController* p_alarmController,
                                       Blower* p_blower)
    : m_inspiratoryValve(p_inspiratory_valve),
      m_expiratoryValve(p_expiratory_valve),
      m_blower(p_blower),
      m_alarmController(p_alarmController) {}

void PressureController::setup() {
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

    m_pressureTriggerOffset = DEFAULT_TRIGGER_OFFSET;
    m_pressureTriggerOffsetNextCommand = DEFAULT_TRIGGER_OFFSET;
    m_triggerModeEnabled = TRIGGER_MODE_ENABLED_BY_DEFAULT;
    m_triggerModeEnabledNextCommand = TRIGGER_MODE_ENABLED_BY_DEFAULT;
    m_expiratoryTerm = DEFAULT_EXPIRATORY_TERM_COMMAND;
    m_expiratoryTermNextCommand = DEFAULT_EXPIRATORY_TERM_COMMAND;

    m_CyclesPerMinuteMeasure = DEFAULT_CYCLE_PER_MINUTE_COMMAND;
    m_lastEndOfRespirationDateMs = 0;
    m_peakPressureMeasure = CONST_INITIAL_ZERO_PRESSURE;
    m_plateauPressureMeasure = CONST_INITIAL_ZERO_PRESSURE;
    m_plateauPressureToDisplay = CONST_INITIAL_ZERO_PRESSURE;
    m_peepMeasure = CONST_INITIAL_ZERO_PRESSURE;

    m_pressure = CONST_INITIAL_ZERO_PRESSURE;
    m_pressureCommand = CONST_INITIAL_ZERO_PRESSURE;

    m_cycleNb = 0;
    m_blower_increment = 0;

    m_dt = 0;

    inspiratoryValveLastAperture = 0;
    expiratoryValveLastAperture = 0;

    m_startPlateauComputation = false;
    m_plateauComputed = false;

    m_sumOfPressures = 0;
    m_numberOfPressures = 0;
    m_plateauStartTime = 0u;
    m_squarePlateauSum = 0;
    m_squarePlateauCount = 0;

    m_triggered = false;
    m_pressureTriggerOffset = 0;

    m_isPeepDetected = false;
    m_plateauDurationMs = 0;

    m_lastEndOfRespirationDateMs = 0;

    m_inspiratoryValveAngle = VALVE_CLOSED_STATE;

    computeTickParameters();
    reachSafetyPosition();

    m_lastPressureValuesIndex = 0;
    for (uint8_t i = 0u; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0u;
    }

    PC_inspiratory_PID_last_errorsIndex = 0;
    PC_expiratory_PID_last_errorsIndex = 0;
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        PC_inspiratory_PID_last_errors[i] = 0u;
        PC_expiratory_PID_last_errors[i] = 0u;
    }

    m_lastBreathPeriodsMsIndex = 0;
    for (uint8_t i = 0u; i < NUMBER_OF_BREATH_PERIOD; i++) {
        m_lastBreathPeriodsMs[i] = (1000u * 60u) / m_cyclesPerMinuteCommand;
    }
}

void PressureController::initRespiratoryCycle() {
    DBG_DO(Serial.println("Init respiratory cycle");)
    m_cycleNb++;
    m_plateauPressureMeasure = 0;

    // Reset PID values
    PC_inspiratory_PID_integral = 0;
    PC_expiratory_PID_integral = 0;
    PC_inspiratory_PID_LastError = m_plateauPressureCommand - m_peepCommand;
    PC_expiratory_PID_LastError = m_peepCommand - m_plateauPressureCommand;
    PC_inspiratory_PID_fast_mode = true;
    PC_expiratory_PID_fast_mode = true;
    for (uint8_t i = 0; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        PC_inspiratory_PID_last_errors[i] = 0;
        PC_expiratory_PID_last_errors[i] = m_peepCommand - m_plateauPressureCommand;
    }

    inspiratoryValveLastAperture = m_inspiratoryValve.maxAperture();
    expiratoryValveLastAperture = m_expiratoryValve.maxAperture();

    m_peakPressureMeasure = 0;
    m_triggered = false;
    m_isPeepDetected = false;
    m_startPlateauComputation = false;
    m_plateauComputed = false;
    m_plateauPressureReached = false;

    // Update new settings at the beginning of the respiratory cycle
    m_cyclesPerMinuteCommand = m_cyclesPerMinuteNextCommand;
    m_peepCommand = m_peepNextCommand;
    m_plateauPressureCommand = m_plateauPressureNextCommand;
    m_triggerModeEnabled = m_triggerModeEnabledNextCommand;
    m_pressureTriggerOffset = m_pressureTriggerOffsetNextCommand;
    m_expiratoryTerm = m_expiratoryTermNextCommand;
    computeTickParameters();
    DBG_AFFICHE_CSPCYCLE_CSPINSPI(m_ticksPerCycle, m_tickPerInhalation)

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
        m_CyclesPerMinuteMeasure = (((NUMBER_OF_BREATH_PERIOD * 60u) * 1000u) + (sum - 1u)) / sum;
    }
    m_lastEndOfRespirationDateMs = millis();

    // Plateau pressure is the mean pressure during plateau
    m_plateauPressureMeasure = m_squarePlateauSum / m_squarePlateauCount;
    calculateBlowerIncrement();

    checkCycleAlarm();

    // If plateau is not detected or is too close to PEEP, mark it as "unknown"
    if ((m_plateauPressureMeasure == 0u) || (abs(m_plateauPressureMeasure - m_peepMeasure) < 10)) {
        m_plateauPressureMeasure = UINT16_MAX;
    }

    // RCM-SW-18 // TODO check why this code is here ??
    if (m_pressure <= ALARM_THRESHOLD_MAX_PRESSURE) {
        m_alarmController->notDetectedAlarm(RCM_SW_18);
    }

    m_plateauPressureToDisplay = m_plateauPressureMeasure;
    if (m_plateauPressureToDisplay == UINT16_MAX) {
        m_plateauPressureToDisplay = 0;
    }

#ifdef MASS_FLOW_METER
    int32_t volume = MFM_read_milliliters(true);
    uint16_t m_tidalVolumeMeasure =
        ((volume > 0xFFFE) || (volume < 0)) ? 0xFFFFu : static_cast<uint16_t>(volume);
#else
    uint16_t m_tidalVolumeMeasure = UINT16_MAX;
#endif

    // Send snapshot of the firmware to th UI
    sendSnapshot(true);
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

void PressureController::updateInspiratoryFlow(int16_t p_currentInspiratoryFlow) {
    // TODO check if value is valid.
    m_inspiratoryFlow = p_currentInspiratoryFlow;
}

void PressureController::updateExpiratoryFlow(int16_t p_currentExpiratoryFlow) {
    // TODO check if value is valid.
    m_expiratoryFlow = p_currentExpiratoryFlow;
}

void PressureController::compute(uint16_t p_tick) {
    // Update the cycle phase
    updatePhase(p_tick);
    m_tick = p_tick;

    // Compute metrics for alarms
    m_sumOfPressures += m_pressure;
    m_numberOfPressures++;

    // Act accordingly
    switch (m_phase) {
    case CyclePhases::INHALATION:
        inhale(p_tick);
        m_inhalationLastPressure = m_pressure;
        break;

    case CyclePhases::EXHALATION:
        exhale();
        // Plateau happens with delay related to the pressure command.
        computePlateau(p_tick);
        break;
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
    sendDataSnapshot(p_tick, m_pressure, m_inspiratoryFlow, m_expiratoryFlow, m_phase, m_subPhase,
                     m_inspiratoryValve.position, m_expiratoryValve.position,
                     m_blower->getSpeed() / 10u, getBatteryLevel());

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
        m_plateauPressureMeasure = totalValues / MAX_PRESSURE_SAMPLES;
    }
}

void PressureController::updatePhase(uint16_t p_tick) {
    if (p_tick < m_tickPerInhalation) {
        m_phase = CyclePhases::INHALATION;
        m_pressureCommand = m_plateauPressureCommand;

    } else {
        m_phase = CyclePhases::EXHALATION;
        m_pressureCommand = m_peepCommand;
    }
}

void PressureController::inhale(uint16_t p_tick) {
    // Keep the inspiratory valve open using a PID. The openning of the valve will be very small
    // during this phase.
    m_inspiratoryValve.open(PCinspiratoryPID(m_pressureCommand, m_pressure, m_dt));

    // -5 is added to help blower convergence
    if (m_pressure > m_plateauPressureCommand - 5 && !m_plateauPressureReached) {
        m_plateauStartTime = p_tick;  // TODO make this only dependent of open loop rampup.
        m_plateauPressureReached = true;
    }
    // Compute mean plateau pressure only after the peak.
    if (m_pressure > m_peakPressureMeasure) {
        m_peakPressureMeasure = m_pressure;
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
    m_expiratoryValve.open(PCexpiratoryPID(m_pressureCommand, m_pressure, m_dt));

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

    // In case the pressure trigger mode is enabled, check if inspiratory trigger is raised
    if (m_triggerModeEnabled && m_isPeepDetected) {
        // m_peakPressure > CONST_MIN_PEAK_PRESSURE ensure that the patient is plugged on the
        // machine.
        if (static_cast<int32_t>(m_pressure)
                < (m_pressureCommand - static_cast<int32_t>(m_pressureTriggerOffset))
            && (m_peakPressureMeasure > CONST_MIN_PEAK_PRESSURE)) {
            m_triggered = true;
        }
    }
}

void PressureController::updateDt(int32_t p_dt) { m_dt = p_dt; }

void PressureController::calculateBlowerIncrement() {
    int16_t peakDelta = m_peakPressureMeasure - m_plateauPressureCommand;

    // Number of tick for the half ramp (120ms)
    int32_t halfRampNumberfTick = 1000 * 120 / static_cast<int32_t>(PCONTROLLER_COMPUTE_PERIOD_US);

    // Update blower only if patient is plugged on the machine
    if (m_peakPressureMeasure > 20) {
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
    }

    DBG_DO(Serial.print("Plateau Start time:");)
    DBG_DO(Serial.println(m_plateauStartTime);)
}

void PressureController::computeTickParameters() {
    // Inspiratory term is always 10. Expiratory term is between 10 and 60 (default 20).
    // The folowing calculation is equivalent of  1000 * (10 / (10 + m_expiratoryTerm) * (60 /
    // m_cyclesPerMinute).
    m_plateauDurationMs = ((10000u / (10u + m_expiratoryTerm)) * 60u) / m_cyclesPerMinuteCommand;

    m_ticksPerCycle = 60u * (1000000u / PCONTROLLER_COMPUTE_PERIOD_US) / m_cyclesPerMinuteCommand;
    m_tickPerInhalation = (m_plateauDurationMs * 1000000u / PCONTROLLER_COMPUTE_PERIOD_US) / 1000u;
}

void PressureController::executeCommands() {
    m_inspiratoryValve.execute();
    m_expiratoryValve.execute();
}

void PressureController::checkCycleAlarm() {
    // RCM-SW-1 + RCM-SW-14 : Check if plateau is reached
    uint16_t minPlateauBeforeAlarm =
        (m_plateauPressureCommand * (100u - ALARM_THRESHOLD_DIFFERENCE_PERCENT)) / 100u;
    uint16_t maxPlateauBeforeAlarm =
        (m_plateauPressureCommand * (100u + ALARM_THRESHOLD_DIFFERENCE_PERCENT)) / 100u;
    if ((m_plateauPressureMeasure < minPlateauBeforeAlarm)
        || (m_plateauPressureMeasure > maxPlateauBeforeAlarm)) {
        m_alarmController->detectedAlarm(RCM_SW_1, m_cycleNb, m_plateauPressureCommand, m_pressure);
        m_alarmController->detectedAlarm(RCM_SW_14, m_cycleNb, m_plateauPressureCommand,
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
    uint16_t PeepBeforeAlarm = m_peepCommand - ALARM_THRESHOLD_DIFFERENCE_PRESSURE;
    uint16_t maxPeepBeforeAlarm = m_peepCommand + ALARM_THRESHOLD_DIFFERENCE_PRESSURE;
    if ((m_peepMeasure < PeepBeforeAlarm) || (m_peepMeasure > maxPeepBeforeAlarm)) {
        m_alarmController->detectedAlarm(RCM_SW_3, m_cycleNb, m_peepCommand, m_pressure);
        m_alarmController->detectedAlarm(RCM_SW_15, m_cycleNb, m_peepCommand, m_pressure);
    } else {
        m_alarmController->notDetectedAlarm(RCM_SW_3);
        m_alarmController->notDetectedAlarm(RCM_SW_15);
    }
}

int32_t
PressureController::PCinspiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt) {

    int32_t minAperture = m_inspiratoryValve.minAperture();
    int32_t maxAperture = m_inspiratoryValve.maxAperture();
    uint32_t inspiratoryValveAperture;
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
    smoothError = totalValues / static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN);

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
            inspiratoryValveAperture = max(
                minAperture,
                min(maxAperture, (static_cast<int32_t>(inspiratoryValveLastAperture) - increment)));
        } else {
            inspiratoryValveAperture = 0;
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
        inspiratoryValveAperture =
            max(minAperture,
                min(maxAperture, maxAperture + (minAperture - maxAperture) * blowerCommand / 1000));
    }

    // If the valve is completly open or completly closed, dont update Integral
    if ((inspiratoryValveAperture != static_cast<uint32_t>(minAperture))
        && (inspiratoryValveAperture != static_cast<uint32_t>(maxAperture))) {
        PC_inspiratory_PID_integral = temporaryPC_inspiratory_PID_integral;
    }

    inspiratoryValveLastAperture = inspiratoryValveAperture;
    PC_inspiratory_PID_LastError = smoothError;

    return inspiratoryValveAperture;
}

int32_t
PressureController::PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt) {

    int32_t minAperture = m_expiratoryValve.minAperture();
    int32_t maxAperture = m_expiratoryValve.maxAperture();
    uint32_t expiratoryValveAperture;
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
    smoothError = totalValues / static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN);
    derivative = (dt == 0) ? 0 : (1000000 * (smoothError - PC_expiratory_PID_LastError)) / dt;

    //  Windowing. It overides the parameter.h coefficients
    if (error < 0) {
        coefficientI = 50;
        coefficientP = 2500;
        coefficientD = 0;
    } else {
        // For a high peep, a lower KI is requiered. For Peep = 100mmH2O, KI = 120. For Peep =
        // 50mmH2O, KI = 250.
        if (m_peepCommand > 100) {
            coefficientI = 120;
        } else {
            coefficientI = ((-130 * ((int32_t)m_peepCommand)) / 50) + 380;
        }

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
            expiratoryValveAperture = max(
                minAperture,
                min(maxAperture, (static_cast<int32_t>(expiratoryValveLastAperture) - increment)));
        } else {
            expiratoryValveAperture = 0;
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

        expiratoryValveAperture = max(
            minAperture,
            min(maxAperture, maxAperture + (maxAperture - minAperture) * patientCommand / 1000));
    }

    // If the valve is completly open or completly closed, dont update Integral
    if ((expiratoryValveAperture != static_cast<uint32_t>(minAperture))
        && (expiratoryValveAperture != static_cast<uint32_t>(maxAperture))) {
        PC_expiratory_PID_integral = temporaryPC_expiratory_PID_integral;
    }

    PC_expiratory_PID_LastError = smoothError;
    expiratoryValveLastAperture = expiratoryValveAperture;

    return expiratoryValveAperture;
}

void PressureController::reachSafetyPosition() {
    m_inspiratoryValve.open();
    m_inspiratoryValve.execute();
    m_expiratoryValve.open();
    m_expiratoryValve.execute();
}

void PressureController::stop() {
    digitalWrite(PIN_LED_START, LED_START_INACTIVE);
    m_blower->stop();
    sendStoppedMessage();
    // When stopped, open the valves
    reachSafetyPosition();
    // Stop alarms related to breathing cycle
    m_alarmController->notDetectedAlarm(RCM_SW_1);
    m_alarmController->notDetectedAlarm(RCM_SW_2);
    m_alarmController->notDetectedAlarm(RCM_SW_3);
    m_alarmController->notDetectedAlarm(RCM_SW_14);
    m_alarmController->notDetectedAlarm(RCM_SW_15);
    m_alarmController->notDetectedAlarm(RCM_SW_18);
    m_alarmController->notDetectedAlarm(RCM_SW_19);
}

void PressureController::sendSnapshot(bool isRunning) {
    // Send the next command, because command has not been updated yet (will be at the beginning of
    // the next cycle)
    sendMachineStateSnapshot(m_cycleNb, mmH2OtoCmH2O(m_peakPressureNextCommand),
                             mmH2OtoCmH2O(m_plateauPressureNextCommand),
                             mmH2OtoCmH2O(m_peepNextCommand), m_cyclesPerMinuteNextCommand,
                             m_peakPressureMeasure, m_plateauPressureToDisplay, m_peepMeasure,
                             m_CyclesPerMinuteMeasure, m_alarmController->triggeredAlarms(), m_tidalVolumeMeasure,
                             m_expiratoryTermNextCommand, m_triggerModeEnabledNextCommand,
                             m_pressureTriggerOffsetNextCommand, isRunning);
    // TODO : store volume
}

void PressureController::onCycleDecrease() {
    DBG_DO(Serial.println("Cycle --");)

    m_cyclesPerMinuteNextCommand = m_cyclesPerMinuteNextCommand - 1;

    if (m_cyclesPerMinuteNextCommand < CONST_MIN_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MIN_CYCLE;
    }

    // Send acknoledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteNextCommand);
}

void PressureController::onCycleIncrease() {
    DBG_DO(Serial.println("Cycle ++");)

    m_cyclesPerMinuteNextCommand = m_cyclesPerMinuteNextCommand + 1;

    if (m_cyclesPerMinuteNextCommand > CONST_MAX_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MAX_CYCLE;
    }

    // Send acknoledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteNextCommand);
}

// cppcheck-suppress unusedFunction
void PressureController::onCycleSet(uint16_t cpm) {
    if (cpm < CONST_MIN_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MIN_CYCLE;
    } else if (cpm > CONST_MAX_CYCLE) {
        m_cyclesPerMinuteNextCommand = CONST_MAX_CYCLE;
    } else {
        m_cyclesPerMinuteNextCommand = cpm;
    }

    // Send acknoledgment to the UI
    sendControlAck(4, m_cyclesPerMinuteNextCommand);
}

void PressureController::onPeepPressureDecrease() {
    DBG_DO(Serial.println("Peep Pressure --");)

    m_peepNextCommand = m_peepNextCommand - 10u;

    if (m_peepNextCommand < CONST_MIN_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MIN_PEEP_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(3, m_peepNextCommand);
}

void PressureController::onPeepPressureIncrease() {
    DBG_DO(Serial.println("Peep Pressure ++");)

    m_peepNextCommand = m_peepNextCommand + 10u;

    if (m_peepNextCommand > CONST_MAX_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MAX_PEEP_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(3, m_peepNextCommand);
}

// cppcheck-suppress unusedFunction
void PressureController::onPeepSet(uint16_t peep) {
    if (peep > CONST_MAX_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MAX_PEEP_PRESSURE;
    } else if (peep < CONST_MIN_PEEP_PRESSURE) {
        m_peepNextCommand = CONST_MIN_PEEP_PRESSURE;
    } else {
        m_peepNextCommand = peep;
    }

    // Send acknoledgment to the UI
    sendControlAck(3, m_peepNextCommand);
}

void PressureController::onPlateauPressureDecrease() {
    DBG_DO(Serial.println("Plateau Pressure --");)

    m_plateauPressureNextCommand = m_plateauPressureNextCommand - 10u;

    if (m_plateauPressureNextCommand < CONST_MIN_PLATEAU_PRESSURE) {
        m_plateauPressureNextCommand = CONST_MIN_PLATEAU_PRESSURE;
    }

    // Send acknoledgment to the UI
    sendControlAck(2, m_plateauPressureNextCommand);
}

void PressureController::onPlateauPressureIncrease() {
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
void PressureController::onPlateauPressureSet(uint16_t plateauPressure) {
    if (plateauPressure > CONST_MAX_PLATEAU_PRESSURE) {
        m_plateauPressureNextCommand = CONST_MAX_PLATEAU_PRESSURE;
    } else if (plateauPressure < CONST_MIN_PLATEAU_PRESSURE) {
        m_plateauPressureNextCommand = CONST_MIN_PLATEAU_PRESSURE;
    } else {
        m_plateauPressureNextCommand = plateauPressure;
    }

    // Send acknoledgment to the UI
    sendControlAck(2, m_plateauPressureNextCommand);
}

void PressureController::onPeakPressureDecrease() {

    // DO NOTHING
}

void PressureController::onPeakPressureIncrease() {
    // DO NOTHING
}

// cppcheck-suppress unusedFunction
void PressureController::onExpiratoryTermSet(uint16_t ExpiratoryTerm) {
    if (ExpiratoryTerm > CONST_MAX_EXPIRATORY_TERM) {
        m_expiratoryTermNextCommand = CONST_MAX_EXPIRATORY_TERM;
    } else if (ExpiratoryTerm < CONST_MIN_EXPIRATORY_TERM) {
        m_expiratoryTermNextCommand = CONST_MIN_EXPIRATORY_TERM;
    } else {
        m_expiratoryTermNextCommand = ExpiratoryTerm;
    }

    // Send acknoledgment to the UI
    sendControlAck(5, m_expiratoryTermNextCommand);
}

// cppcheck-suppress unusedFunction
void PressureController::onTriggerEnabledSet(uint16_t TriggerEnabled) {
    if ((TriggerEnabled == 0u) || (TriggerEnabled == 1u)) {
        m_triggerModeEnabledNextCommand = TriggerEnabled;
    }

    // Send acknoledgment to the UI
    sendControlAck(6, m_triggerModeEnabledNextCommand);
}

// cppcheck-suppress unusedFunction
void PressureController::onTriggerOffsetSet(uint16_t TriggerOffset) {
    if (TriggerOffset > CONST_MAX_TRIGGER_OFFSET) {
        m_pressureTriggerOffsetNextCommand = CONST_MAX_TRIGGER_OFFSET;
        // cppcheck-suppress unsignedLessThanZero
    } else if (TriggerOffset < CONST_MIN_TRIGGER_OFFSET) {
        m_pressureTriggerOffsetNextCommand = CONST_MIN_TRIGGER_OFFSET;
    } else {
        m_pressureTriggerOffsetNextCommand = TriggerOffset;
    }

    // Send acknoledgment to the UI
    sendControlAck(7, m_pressureTriggerOffsetNextCommand);
}