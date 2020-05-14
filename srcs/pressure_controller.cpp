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
#include "../includes/config.h"
#include "../includes/debug.h"
#include "../includes/parameters.h"
#include "../includes/pressure_valve.h"
#if HARDWARE_VERSION == 2
#include "../includes/battery.h"
#include "../includes/telemetry.h"
#endif

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
      m_phase(CyclePhases::INHALATION),
      m_subPhase(CycleSubPhases::INSPIRATION),
      m_blower(nullptr),
      m_alarmController(nullptr),
      m_blower_increment(0),
      m_cycleNb(0),
      m_dt(0),
      m_pressureCommand(0),
      blowerIntegral(0),
      blowerLastError(0),
      patientIntegral(0),
      lastBlowerAperture(0),
      patientLastError(0),
      m_startPlateauComputation(false),
      m_plateauComputed(false),
      m_lastPressureValuesIndex(0),
      m_sumOfPressures(0),
      m_numberOfPressures(0),
      m_plateauStartTime(0u),
      m_squarePlateauSum(0),
      m_squarePlateauCount(0),
      m_lastBlowerPIDErrorIndex(0),
      m_lastPatientPIDErrorIndex(0),
      patientPIDFastMode(true),
      blowerPIDFastMode(true),
      patientDamping(0),
      m_tick(0),
      m_peakBlowerValveAngle(VALVE_CLOSED_STATE) {
    computetickParameters();
    for (uint8_t i = 0u; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0u;
    }
    for (uint8_t i = 0u; i < NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN; i++) {
        m_lastBlowerPIDError[i] = 0u;
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
      m_phase(CyclePhases::INHALATION),
      m_subPhase(CycleSubPhases::INSPIRATION),
      // cppcheck-suppress misra-c2012-12.3
      m_blower_valve(p_blower_valve),
      m_patient_valve(p_patient_valve),
      m_blower(p_blower),
      m_alarmController(p_alarmController),
      m_blower_increment(0),
      m_cycleNb(0),
      m_dt(0),
      m_pressureCommand(0),
      blowerIntegral(0),
      blowerLastError(0),
      patientIntegral(0),
      lastBlowerAperture(0),
      patientLastError(0),
      m_startPlateauComputation(false),
      m_plateauComputed(false),
      m_lastPressureValuesIndex(0),
      m_sumOfPressures(0),
      m_numberOfPressures(0),
      m_plateauStartTime(0u),
      m_squarePlateauSum(0),
      m_squarePlateauCount(0),
      m_lastBlowerPIDErrorIndex(0),
      m_lastPatientPIDErrorIndex(0),
      patientPIDFastMode(true),
      blowerPIDFastMode(true),
      patientDamping(0),
      m_tick(0),
      m_peakBlowerValveAngle(VALVE_CLOSED_STATE) {
    computetickParameters();
    for (uint8_t i = 0u; i < MAX_PRESSURE_SAMPLES; i++) {
        m_lastPressureValues[i] = 0u;
    }
    for (uint8_t i = 0u; i < NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN; i++) {
        m_lastBlowerPIDError[i] = 0u;
        m_lastPatientPIDError[i] = 0u;
    }
}

void PressureController::setup() {
    DBG_DO(Serial.println(VERSION);)
    DBG_DO(Serial.println("mise en secu initiale");)

    m_blower_valve.close();
    m_patient_valve.close();

    m_blower_valve.execute();
    m_patient_valve.execute();

    m_peakPressure = 0;
    m_plateauPressure = 0;
    m_peep = 0;

    m_cycleNb = 0;
}

void PressureController::initRespiratoryCycle() {
    m_phase = CyclePhases::INHALATION;
    setSubPhase(CycleSubPhases::INSPIRATION, 0);
    m_cycleNb++;
    m_plateauPressure = 0;

#if VALVE_TYPE == VT_FAULHABER
    // Blowererror is initialised as command - peep. With Faulhaber valves, peak command is plateau
    // command
    blowerLastError = m_maxPlateauPressureCommand - m_minPeepCommand;
#else
    blowerLastError = m_maxPeakPressureCommand - m_minPeepCommand;
#endif

    // Reset PID integrals
    blowerIntegral = 0;
    patientIntegral = 0;
    patientLastError = m_minPeepCommand - m_maxPlateauPressureCommand;
    lastBlowerAperture = m_blower_valve.maxAperture();
    blowerPIDFastMode = false;
    patientPIDFastMode = true;

    m_peakPressure = 0;
    computetickParameters();

    DBG_AFFICHE_CSPCYCLE_CSPINSPI(m_ticksPerCycle, m_tickPerInhalation)

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

    m_plateauStartTime = 0u;

    m_squarePlateauSum = 0u;
    m_squarePlateauCount = 0u;

    for (uint8_t i = 0; i < NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN; i++) {
        m_lastBlowerPIDError[i] = 0;
        m_lastPatientPIDError[i] = m_minPeepCommand - m_maxPlateauPressureCommand;
    }
}

void PressureController::endRespiratoryCycle() {
#if VALVE_TYPE == VT_FAULHABER
    // In square plateau mode, plateau pressure is the mean pressure during plateau
    m_plateauPressure = m_squarePlateauSum / m_squarePlateauCount;
    updateOnlyBlower();
#else
    updatePeakPressure();
#endif
    checkCycleAlarm();

    // If plateau is not detected or is too close to PEEP, mark it as "unknown"
    if ((m_plateauPressure == 0u) || (abs(m_plateauPressure - m_peep) < 10)) {
        m_plateauPressure = UINT16_MAX;
    }

    // RCM-SW-18
    if (m_pressure <= ALARM_THRESHOLD_MAX_PRESSURE) {
        m_alarmController->notDetectedAlarm(RCM_SW_18);
    }

#if HARDWARE_VERSION == 2
    uint16_t plateauPressureToDisplay = m_plateauPressure;
    if (plateauPressureToDisplay == UINT16_MAX) {
        plateauPressureToDisplay = 0;
    }

    sendMachineStateSnapshot(m_cycleNb, mmH2OtoCmH2O(m_maxPeakPressureCommand),
                             mmH2OtoCmH2O(m_maxPlateauPressureCommand),
                             mmH2OtoCmH2O(m_minPeepCommand), m_cyclesPerMinuteCommand,
                             m_peakPressure, plateauPressureToDisplay, m_peep,
                             m_alarmController->triggeredAlarms());
#endif
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
        // Plateau happens with delay related to the pressure command. With faulhaber valves,
        // plateau is computed differently
#if VALVE_TYPE != VT_FAULHABER
        computePlateau(p_tick);
#endif
        break;
    }
    }

    // RCM-SW-18
    if (m_pressure > ALARM_THRESHOLD_MAX_PRESSURE) {
        m_alarmController->detectedAlarm(RCM_SW_18, m_cycleNb, ALARM_THRESHOLD_MAX_PRESSURE,
                                         m_pressure);
    }

    DBG_PHASE_PRESSION(m_cycleNb, p_tick, 1u, m_phase, m_subPhase, m_pressure,
                       m_blower_valve.command, m_blower_valve.position, m_patient_valve.command,
                       m_patient_valve.position)

#if HARDWARE_VERSION == 2
    m_alarmController->updateCoreData(p_tick, m_pressure, m_phase, m_subPhase, m_cycleNb);
    if (p_tick%2==0){
      sendDataSnapshot(p_tick/2, m_pressure, m_phase, m_subPhase, m_blower_valve.position,
                       m_patient_valve.position, m_blower->getSpeed() / 10u, getBatteryLevel());
    }
#endif

    executeCommands();
}

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
    if (!m_plateauComputed && (diff < 10u)
        && (p_tick >= ((m_tickPerInhalation * 95u) / 100u))) {
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
}

void PressureController::onCycleIncrease() {
#if SIMULATION != 1
    // During simulation without electronic board there is a noise on the button pin. It increases
    // the cycle and the simulation fail.
    DBG_DO(Serial.println("Cycle ++");)

    m_cyclesPerMinuteCommand++;

    if (m_cyclesPerMinuteCommand > CONST_MAX_CYCLE) {
        m_cyclesPerMinuteCommand = CONST_MAX_CYCLE;
    }

#endif
}

void PressureController::onPeepPressureDecrease() {
    DBG_DO(Serial.println("Peep Pressure --");)

    m_minPeepCommand = m_minPeepCommand - 10u;

    if (m_minPeepCommand < CONST_MIN_PEEP_PRESSURE) {
        m_minPeepCommand = CONST_MIN_PEEP_PRESSURE;
    }
}

void PressureController::onPeepPressureIncrease() {
    DBG_DO(Serial.println("Peep Pressure ++");)

    m_minPeepCommand = m_minPeepCommand + 10u;

    if (m_minPeepCommand > CONST_MAX_PEEP_PRESSURE) {
        m_minPeepCommand = CONST_MAX_PEEP_PRESSURE;
    }
}

void PressureController::onPlateauPressureDecrease() {
    DBG_DO(Serial.println("Plateau Pressure --");)

    m_maxPlateauPressureCommand = m_maxPlateauPressureCommand - 10u;

    if (m_maxPlateauPressureCommand < CONST_MIN_PLATEAU_PRESSURE) {
        m_maxPlateauPressureCommand = CONST_MIN_PLATEAU_PRESSURE;
    }
}

void PressureController::onPlateauPressureIncrease() {
    DBG_DO(Serial.println("Plateau Pressure ++");)

    m_maxPlateauPressureCommand = m_maxPlateauPressureCommand + 10u;

    m_maxPlateauPressureCommand =
        min(m_maxPlateauPressureCommand, static_cast<uint16_t>(CONST_MAX_PLATEAU_PRESSURE));
}

void PressureController::onPeakPressureDecrease(uint8_t p_decrement) {
    DBG_DO(Serial.println("Peak Pressure --");)

    m_maxPeakPressureCommand = m_maxPeakPressureCommand - p_decrement;

    m_maxPeakPressureCommand =
        max(m_maxPeakPressureCommand, static_cast<uint16_t>(CONST_MIN_PEAK_PRESSURE));
}

void PressureController::onPeakPressureIncrease(uint8_t p_increment) {
    DBG_DO(Serial.println("Peak Pressure ++");)

    m_maxPeakPressureCommand = m_maxPeakPressureCommand + p_increment;

    if (m_maxPeakPressureCommand > CONST_MAX_PEAK_PRESSURE) {
        m_maxPeakPressureCommand = CONST_MAX_PEAK_PRESSURE;
    }
}

void PressureController::updatePhase(uint16_t p_tick) {
    if (p_tick < m_tickPerInhalation) {
        m_phase = CyclePhases::INHALATION;

#if VALVE_TYPE == VT_FAULHABER
        uint16_t pressureToTest = m_maxPlateauPressureCommand;
#else
        uint16_t pressureToTest = m_maxPeakPressureCommand;
#endif

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
    // Open the air stream towards the patient's lungs
    m_peakBlowerValveAngle = pidBlower(m_pressureCommand, m_pressure, m_dt);
    m_blower_valve.open(m_peakBlowerValveAngle);

    // Open the air stream towards the patient's lungs
    m_patient_valve.close();

    // Update the peak pressure
    m_peakPressure = max(m_pressure, m_peakPressure);
}

void PressureController::plateau() {
    // Deviate the air stream outside
#if VALVE_TYPE == VT_FAULHABER
    m_blower_valve.open(pidBlower(m_pressureCommand, m_pressure, m_dt));
    m_squarePlateauSum += m_pressure;
    m_squarePlateauCount += 1u;
#else
    m_blower_valve.close();
#endif

    // Close the air stream towards the patient's lungs
    m_patient_valve.close();

    // Update the peak pressure
    m_peakPressure = max(m_pressure, m_peakPressure);
}

void PressureController::exhale() {
    // Deviate the air stream outside
    m_blower_valve.close();

    // Open the valve so the patient can exhale outside
    m_patient_valve.open(pidPatient(m_pressureCommand, m_pressure, m_dt));

    // Update the PEEP
    m_peep = m_pressure;
}

void PressureController::updateDt(int32_t p_dt) { m_dt = p_dt; }

#if VALVE_TYPE == VT_FAULHABER
void PressureController::updateOnlyBlower() {
    int16_t plateauDelta = m_maxPlateauPressureCommand - m_plateauPressure;

    DBG_DO(Serial.println("updatePeakPressure");)
    // If plateau was reached quite early
    if (m_plateauStartTime < ((m_tickPerInhalation * 10u) / 100u)) {
        DBG_DO(Serial.println("BLOWER -20");)

        // If the peak delta is high, decrease blower's speed a lot
        if (plateauDelta < -20) {
            m_blower_increment = -60;
            DBG_DO(Serial.print("BLOWER -60, peak: ");)
            DBG_DO(Serial.println(plateauDelta);)
        } else {
            m_blower_increment = -20;
            DBG_DO(Serial.println("BLOWER -20");)
        }
    } else if ((m_plateauStartTime >= ((m_tickPerInhalation * 10u) / 100u))
               && (m_plateauStartTime < ((m_tickPerInhalation * 20u) / 100u))) {
        DBG_DO(Serial.println("BLOWER -10");)
        m_blower_increment = -10;
    } else if ((m_plateauStartTime > ((m_tickPerInhalation * 30u) / 100u))
               && (m_plateauStartTime <= ((m_tickPerInhalation * 40u) / 100u))
               && abs(plateauDelta) > 10) {
        DBG_DO(Serial.println("BLOWER +10");)
        m_blower_increment = +10;
    } else if (m_plateauStartTime > ((m_tickPerInhalation * 40u) / 100u)) {
        if (plateauDelta > 60) {
            m_blower_increment = +60;
            DBG_DO(Serial.print("BLOWER +60, peak: ");)
            DBG_DO(Serial.print(m_maxPlateauPressureCommand);)
            DBG_DO(Serial.print("-");)
            DBG_DO(Serial.println(m_plateauPressure);)
        } else if (plateauDelta > 40) {
            m_blower_increment = +40;
            DBG_DO(Serial.print("BLOWER +40, peak: ");)
            DBG_DO(Serial.println(plateauDelta);)
        } else {
            m_blower_increment = +20;
            DBG_DO(Serial.println("BLOWER +20");)
        }
    } else {
        m_blower_increment = 0;
        DBG_DO(Serial.println("BLOWER +0");)
    }
    DBG_DO(Serial.print("Plateau Start time:");)
    DBG_DO(Serial.println(m_plateauStartTime);)
}
#endif

void PressureController::updatePeakPressure() {
    int16_t plateauDelta = m_maxPlateauPressureCommand - m_plateauPressure;
    int16_t peakDelta = m_maxPeakPressureCommand - m_peakPressure;

    DBG_DO(Serial.println("updatePeakPressure");)

    // If a plateau was detected
    if ((m_plateauPressure > 0u) && (m_plateauPressure < UINT16_MAX)) {
        DBG_DO(Serial.println("Plateau detected");)

        if (plateauDelta > 60) {
            m_maxPeakPressureCommand =
                // cppcheck-suppress misra-c2012-12.3
                min(min(m_peakPressure, m_maxPeakPressureCommand) + 60,
                    static_cast<int>(CONST_MAX_PEAK_PRESSURE));
        } else if (abs(plateauDelta) > 20) {
            m_maxPeakPressureCommand =
                // cppcheck-suppress misra-c2012-12.3
                max(min(min(m_peakPressure, m_maxPeakPressureCommand) + plateauDelta,
                        // cppcheck-suppress misra-c2012-12.3
                        static_cast<int>(CONST_MAX_PEAK_PRESSURE)),
                    static_cast<int>(m_maxPlateauPressureCommand));
        } else if ((abs(plateauDelta) < 20) && (abs(plateauDelta) > 5)) {
            m_maxPeakPressureCommand =
                // cppcheck-suppress misra-c2012-12.3
                max(min(min(m_peakPressure, m_maxPeakPressureCommand) + (plateauDelta / 2),
                        // cppcheck-suppress misra-c2012-12.3
                        static_cast<int>(CONST_MAX_PEAK_PRESSURE)),
                    static_cast<int>(m_maxPlateauPressureCommand));
        } else {
            // Do nothing
        }

        m_maxPeakPressureCommand = min(m_maxPeakPressureCommand,
                                       static_cast<uint16_t>(m_maxPlateauPressureCommand + 150u));

        DBG_DO(Serial.print("Peak command:");)
        DBG_DO(Serial.println(m_maxPeakPressureCommand);)

        DBG_DO(Serial.print("m_plateauStartTime:");)
        DBG_DO(Serial.println(m_plateauStartTime);)

        // If plateau was reached quite early
        if (m_plateauStartTime < ((m_tickPerInhalation * 30u) / 100u)) {
            DBG_DO(Serial.println("BLOWER -20");)

            // If the peak delta is high, decrease blower's speed a lot
            if (peakDelta < -20) {
                m_blower_increment = -60;
                DBG_DO(Serial.print("BLOWER -60, peak: ");)
                DBG_DO(Serial.println(peakDelta);)
            } else {
                m_blower_increment = -20;
                DBG_DO(Serial.println("BLOWER -20");)
            }
        } else if ((m_plateauStartTime >= ((m_tickPerInhalation * 30u) / 100u))
                   && (m_plateauStartTime < ((m_tickPerInhalation * 40u) / 100u))) {
            DBG_DO(Serial.println("BLOWER -10");)
            m_blower_increment = -10;
        } else if ((m_plateauStartTime > ((m_tickPerInhalation * 60u) / 100u))
                   && (m_plateauStartTime <= ((m_tickPerInhalation * 70u) / 100u))
                   && abs(plateauDelta) > 10) {
            DBG_DO(Serial.println("BLOWER +10");)
            m_blower_increment = +10;
        } else if (m_plateauStartTime > ((m_tickPerInhalation * 70u) / 100u)) {
            if (peakDelta > 60) {
                m_blower_increment = +60;
                DBG_DO(Serial.print("BLOWER +60, peak: ");)
                DBG_DO(Serial.println(peakDelta);)
            } else if (peakDelta > 40) {
                m_blower_increment = +40;
                DBG_DO(Serial.print("BLOWER +40, peak: ");)
                DBG_DO(Serial.println(peakDelta);)
            } else {
                m_blower_increment = +20;
                DBG_DO(Serial.println("BLOWER +20");)
            }
        } else {
            m_blower_increment = 0;
            DBG_DO(Serial.println("BLOWER +0");)
        }
    } else {  // If no plateau was detected: only do blower ramp-up
        DBG_DO(Serial.println("Plateau not detected");)

        if (m_plateauStartTime < ((m_tickPerInhalation * 30u) / 100u)) {
            DBG_DO(Serial.println("BLOWER -40");)
            m_blower_increment = -40;
        } else if ((m_plateauStartTime >= ((m_tickPerInhalation * 30u) / 100u))
                   && (m_plateauStartTime < ((m_tickPerInhalation * 40u) / 100u))) {
            DBG_DO(Serial.println("BLOWER -10");)
            m_blower_increment = -10;
        } else if ((m_plateauStartTime > ((m_tickPerInhalation * 60u) / 100u))
                   && (m_plateauStartTime <= ((m_tickPerInhalation * 70u) / 100u))) {
            DBG_DO(Serial.println("BLOWER +10");)
            m_blower_increment = +10;
        } else if (m_plateauStartTime > ((m_tickPerInhalation * 70u) / 100u)) {
            DBG_DO(Serial.println("BLOWER +40");)
            m_blower_increment = +40;
        } else {
            m_blower_increment = 0;
            DBG_DO(Serial.println("BLOWER +0");)
        }
    }
}

void PressureController::computetickParameters() {
    m_ticksPerCycle = 60u * (1000000u / PCONTROLLER_COMPUTE_PERIOD_US)/ m_cyclesPerMinute;
    // Inhalation = 1/3 of the cycle duration,
    // Exhalation = 2/3 of the cycle duration
    m_tickPerInhalation = m_ticksPerCycle / 3u;
}

void PressureController::executeCommands() {
    m_blower_valve.execute();
    m_patient_valve.execute();
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
#if VALVE_TYPE == VT_FAULHABER
    // Compute error
    int32_t error = targetPressure - currentPressure;
    int32_t minAperture = m_blower_valve.minAperture();
    int32_t maxAperture = m_blower_valve.maxAperture();
    uint32_t blowerAperture;
    int32_t derivative = 0;
    int32_t smoothError = 0;
    int32_t totalValues = 0;
    int32_t coefficientI;

    // Different KI in case of overshooting
    if (error < 0) {
        coefficientI = PID_BLOWER_KI * 2;
    } else {
        coefficientI = PID_BLOWER_KI;
    }

    m_lastBlowerPIDError[m_lastBlowerPIDErrorIndex] = error;
    m_lastBlowerPIDErrorIndex++;
    if (m_lastBlowerPIDErrorIndex
        >= static_cast<int32_t>(NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN)) {
        m_lastBlowerPIDErrorIndex = 0;
    }

    for (uint8_t i = 0u; i < NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN; i++) {
        totalValues += m_lastBlowerPIDError[i];
    }
    smoothError =
        totalValues / static_cast<int32_t>(NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN);

    derivative = ((dt == 0)) ? 0 : ((1000000 * (blowerLastError - smoothError)) / dt);

    blowerIntegral = blowerIntegral + ((coefficientI * error * dt) / 1000000);
    blowerIntegral = max(PID_BLOWER_INTEGRAL_MIN, min(PID_BLOWER_INTEGRAL_MAX, blowerIntegral));
    
    int32_t blowerCommand =
        ((PID_BLOWER_KP * error) / 1000) + blowerIntegral + ((PID_BLOWER_KD * derivative) / 1000);
    blowerAperture =
        max(minAperture,
            min(maxAperture, maxAperture + (minAperture - maxAperture) * blowerCommand / 1000));

    // Slow down aperture at starting
    if ((error > 50) && (blowerAperture > (lastBlowerAperture + 15u))) {
        blowerAperture = lastBlowerAperture + 15u;
    } else if ((error > 50) && (lastBlowerAperture > 15u)
               && (blowerAperture < (lastBlowerAperture - 15u))) {
        blowerAperture = lastBlowerAperture - 15u;
    } else {
        // Do nothing
    }

    lastBlowerAperture = blowerAperture;
    blowerLastError = smoothError;
    /*Serial.print(((PID_BLOWER_KP * error) / 1000));
    Serial.print(",");
    Serial.print(blowerIntegral);
    Serial.print(",");
    Serial.print(((PID_BLOWER_KD * derivative) / 1000));
    Serial.print(",");
    Serial.println();*/
#else
    // Compute error
    int32_t error = targetPressure - currentPressure;

    // Compute integral
    blowerIntegral = blowerIntegral + ((PID_BLOWER_KI * error * dt) / 1000000);
    blowerIntegral = max(PID_BLOWER_INTEGRAL_MIN, min(PID_BLOWER_INTEGRAL_MAX, blowerIntegral));

    // Compute derivative
    int32_t derivative = ((blowerLastError == INVALID_ERROR_MARKER) || (dt == 0))
                             ? 0
                             : ((1000000 * (error - blowerLastError)) / dt);
    blowerLastError = error;

    int32_t blowerCommand = (PID_BLOWER_KP * error) + blowerIntegral
                            + ((PID_BLOWER_KD * derivative) / 1000);  // Command computation

    int32_t minAperture = m_blower_valve.minAperture();
    int32_t maxAperture = m_blower_valve.maxAperture();

    uint32_t blowerAperture =
        max(minAperture,
            min(maxAperture, maxAperture + (minAperture - maxAperture) * blowerCommand / 1000));
#endif

    return blowerAperture;
}

int32_t
PressureController::pidPatient(int32_t targetPressure, int32_t currentPressure, int32_t dt) {
#if VALVE_TYPE == VT_FAULHABER
    // Compute error
    int32_t error = targetPressure + PID_PATIENT_SAFETY_PEEP_OFFSET - currentPressure;
    int32_t minAperture = m_patient_valve.minAperture();
    int32_t maxAperture = m_patient_valve.maxAperture();
    uint32_t patientAperture;
    int32_t derivative = 0;
    int32_t smoothError = 0;
    int32_t totalValues = 0;
    int32_t coefficientI = PID_PATIENT_KI;
    int32_t coefficientP = PID_PATIENT_KP;
    int32_t coefficientD = PID_PATIENT_KD;
    int32_t slope = 0;
    int32_t temporaryPatientIntegral = 0;
    



    if (error > -10) {
        patientPIDFastMode = false;
    }


    m_lastPatientPIDError[m_lastPatientPIDErrorIndex] = error;
    m_lastPatientPIDErrorIndex++;
    if (m_lastPatientPIDErrorIndex
        >= static_cast<int32_t>(NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN)) {
        m_lastPatientPIDErrorIndex = 0;
    }

    for (uint8_t i = 0u; i < NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN; i++) {
        totalValues += m_lastPatientPIDError[i];
    }
    smoothError =
        totalValues / static_cast<int32_t>(NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN);
    
    

    derivative = (dt == 0) ? 0 : (1000000 * (smoothError - patientLastError)) / dt;
    if (patientPIDFastMode) {

        int32_t initialError = m_minPeepCommand - m_inhalationLastPressure;
        slope = ((m_tick - m_tickPerInhalation) == 0) ? 0 : 
          ((1000000/(int32_t)PCONTROLLER_COMPUTE_PERIOD_US) * (error - initialError)) / (m_tick - m_tickPerInhalation);

        coefficientP = ((-2*PID_PATIENT_KP*slope/4000 + PID_PATIENT_KP) * (1000 + 10000*(m_tick - m_tickPerInhalation)/(2*m_tickPerInhalation)))/1000;
        coefficientP = max(PID_PATIENT_KP/4, min(coefficientP, 2*PID_PATIENT_KP));
        coefficientI = coefficientI /4 ;

    } else {
      coefficientP = PID_PATIENT_KP / 4;
      coefficientI = coefficientI;
    }

    temporaryPatientIntegral = patientIntegral + ((coefficientI * error * dt) / 1000000);
    temporaryPatientIntegral =
        max(PID_PATIENT_INTEGRAL_MIN, min(PID_PATIENT_INTEGRAL_MAX, temporaryPatientIntegral));

    int32_t patientCommand =
        ((coefficientP * error) / 1000) + patientIntegral - coefficientD*derivative/1000; 

    

    /*if (derivative < 0){
      patientDamping = min((int32_t)0, patientDamping - coefficientD*derivative/1000/2);
    }*/

    patientAperture = max(
        minAperture,
        min(maxAperture, maxAperture + (maxAperture - minAperture) * patientCommand / 1000));

    if (patientAperture !=  minAperture && patientAperture != maxAperture){
      patientIntegral = temporaryPatientIntegral;
    }

    
    
    /*Serial.print(dt);
    Serial.print(",");*/
    Serial.print(targetPressure);
    Serial.print(",");
    Serial.print(currentPressure);
    Serial.print(",");
    Serial.print(((coefficientP * error) / 1000));
    Serial.print(",");
    Serial.print(patientIntegral);
    Serial.print(",");
    Serial.print(PID_PATIENT_KD*derivative/1000);
    Serial.print(",");
    Serial.print(slope);
    Serial.print(",");
    Serial.print(patientAperture);
    Serial.print(",");
    Serial.println();
    patientLastError = smoothError;
#else
    // Compute error
    int32_t error = targetPressure + PID_PATIENT_SAFETY_PEEP_OFFSET - currentPressure;

    // Compute integral
    patientIntegral = patientIntegral + ((PID_PATIENT_KI * error * dt) / 1000000);
    patientIntegral = max(PID_PATIENT_INTEGRAL_MIN, min(PID_PATIENT_INTEGRAL_MAX, patientIntegral));

    // Compute derivative
    int32_t derivative = ((patientLastError == INVALID_ERROR_MARKER) || (dt == 0))
                             ? 0
                             : ((1000000 * (error - patientLastError)) / dt);
    patientLastError = error;

    int32_t patientCommand = (PID_PATIENT_KP * error) + patientIntegral
                             + ((PID_PATIENT_KD * derivative) / 1000);  // Command computation

    int32_t minAperture = m_blower_valve.minAperture();
    int32_t maxAperture = m_blower_valve.maxAperture();

    uint32_t patientAperture =
        max(minAperture,
            min(maxAperture, maxAperture + (maxAperture - minAperture) * patientCommand / 1000));

#endif

    return patientAperture;
}


void PressureController::reachSafetyPosition(){
  m_blower_valve.open();
  m_blower_valve.execute();
  m_patient_valve.open();
  m_patient_valve.execute();
}