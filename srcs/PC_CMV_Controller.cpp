

/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure.cpp
 * @brief Pressure sensor related functions
 *****************************************************************************/

// INCLUDES ==================================================================

// Associated header
#include "../includes/PC_CMV_Controller.h"

// External
#include "Arduino.h"
#include <algorithm>

// Internal

#include "../includes/pressure_controller.h"
#include "../includes/pressure_valve.h"

PC_CMV_Controller pcCmvController;

PC_CMV_Controller::PC_CMV_Controller() {}

void PC_CMV_Controller::setup() {

    m_inspiratoryValveLastAperture = inspiratoryValve.maxAperture();
    m_expiratoryValveLastAperture = expiratoryValve.maxAperture();
    m_plateauPressureReached = false;
    m_plateauStartTime = mainController.tickPerInhalation();

    m_inspiratoryPidLastErrorsIndex = 0;
    m_expiratoryPidLastErrorsIndex = 0;
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        m_inspiratoryPidLastErrorsIndex[i] = 0u;
        m_expiratoryPidLastErrorsIndex[i] = 0u;
    }
}

void PC_CMV_Controller::initCycle() {

    m_plateauPressureReached = false;
    m_plateauStartTime = mainController.tickPerInhalation();

    m_inspiratoryValveLastAperture = inspiratoryValve.maxAperture();
    m_expiratoryValveLastAperture = expiratoryValve.maxAperture();
    m_inspiratoryValveLastAperture = inspiratoryValve.maxAperture();
    m_expiratoryValveLastAperture = expiratoryValve.maxAperture();
    // Reset PID values
    m_inspiratoryPidIntegral = 0;
    m_expiratoryPidIntegral = 0;
    m_inspiratoryPidLastError = mainController.plateauPressureCommand() - mainController.peepCommand();
    m_expiratoryPidLastError = mainController.peepCommand() - mainController.plateauPressureCommand();
    m_inspiratoryPidFastMode = true;
    m_expiratoryPidFastMode = true;
    for (uint8_t i = 0; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        m_inspiratoryPidLastErrorsIndex[i] = 0;
        m_expiratoryPidLastErrorsIndex[i] =
            mainController.peepCommand() - mainController.plateauPressureCommand();
    }

    // Apply blower ramp-up
    if (m_blowerIncrement >= 0) {
        blower.runSpeed(blower.getSpeed() + static_cast<uint16_t>(abs(m_blowerIncrement)));
    } else {
        // When blower increment is negative, we need to check that it is less than current speed
        // If not, it would result in an overflow
        if (static_cast<uint16_t>(abs(m_blowerIncrement)) < blower.getSpeed()) {
            blower.runSpeed(blower.getSpeed() - static_cast<uint16_t>(abs(m_blowerIncrement)));
        } else {
            blower.runSpeed(MIN_BLOWER_SPEED);
        }
    }
    m_blowerIncrement = 0;
}

void PC_CMV_Controller::inhale(uint16_t p_tick) {
    // Keep the inspiratory valve open using a PID.
    int32_t inspiratoryPidValue =
        PCinspiratoryPID(mainController.pressureCommand(), mainController.pressure(), mainController.dt());

    inspiratoryValve.open(inspiratoryPidValue);
    expiratoryValve.close();

    // m_plateauStartTime is used for blower regulations, -20 is added to help blower convergence
    if (mainController.pressure() > mainController.plateauPressureCommand() - 20u
        && !m_plateauPressureReached) {
        m_plateauStartTime = p_tick;  // TODO make this only dependent of open loop rampup.
        m_plateauPressureReached = true;
    }
}

void PC_CMV_Controller::exhale(uint16_t p_tick) {

    // Close the inspiratory valve
    inspiratoryValve.close();

    // Open the expiratos valve so the patient can exhale outside
    expiratoryValve.open(
        PCexpiratoryPID(mainController.pressureCommand(), mainController.pressure(), mainController.dt()));

    // In case the pressure trigger mode is enabled, check if inspiratory trigger is raised
    if (mainController.triggerModeEnabledCommand() && mainController.isPeepDetected()) {
        // m_peakPressure > CONST_MIN_PEAK_PRESSURE ensure that the patient is plugged on the
        // machine.
        if (static_cast<int32_t>(mainController.pressure())
                < (mainController.pressureCommand()
                   - static_cast<int32_t>(mainController.pressureTriggerOffsetCommand()))
            && (mainController.peakPressureMeasure() > CONST_MIN_PEAK_PRESSURE)) {
            mainController.setTrigger(true);
        }
    }
}

void PC_CMV_Controller::endCycle() { calculateBlowerIncrement(); }

void PC_CMV_Controller::calculateBlowerIncrement() {
    int16_t peakDelta = mainController.peakPressureMeasure() - mainController.plateauPressureCommand();

    // Number of tick for the half ramp (120ms)
    int32_t halfRampNumberfTick = 1000 * 120 / static_cast<int32_t>(mainController_COMPUTE_PERIOD_US);

    // Update blower only if patient is plugged on the machine
    if (mainController.peakPressureMeasure() > 20) {
        if (m_plateauStartTime < ((mainController.tickPerInhalation() * 30u) / 100u)) {
            // Only case for decreasing the blower : ramping is too fast or overshooting is too high
            if ((m_plateauStartTime < static_cast<uint32_t>(abs(halfRampNumberfTick)))
                || ((peakDelta > 15)
                    && (m_plateauStartTime < ((mainController.tickPerInhalation() * 20u) / 100u)))
                || (peakDelta > 25)) {
                m_blowerIncrement = -100;
                DBG_DO(Serial.println("BLOWER -100");)
            } else {
                m_blowerIncrement = 0;
                DBG_DO(Serial.println("BLOWER 0");)
            }
        } else if (m_plateauStartTime < ((mainController.tickPerInhalation() * 40u) / 100u)) {
            DBG_DO(Serial.println("BLOWER +0");)
            m_blowerIncrement = 0;
        } else if (m_plateauStartTime < ((mainController.tickPerInhalation() * 50u) / 100u)) {
            m_blowerIncrement = +25;
            DBG_DO(Serial.println("BLOWER +25"));
        } else if (m_plateauStartTime < ((mainController.tickPerInhalation() * 60u) / 100u)) {
            m_blowerIncrement = +50;
            DBG_DO(Serial.println("BLOWER +50"));
        } else {
            m_blowerIncrement = +100;
            DBG_DO(Serial.println("BLOWER +100"));
        }
    }

    DBG_DO(Serial.print("Plateau Start time:");)
    DBG_DO(Serial.println(m_plateauStartTime);)
}

int32_t
PC_CMV_Controller::PCinspiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt) {

    int32_t minAperture = inspiratoryValve.minAperture();
    int32_t maxAperture = inspiratoryValve.maxAperture();
    int32_t inspiratoryValveAperture;
    int32_t derivative = 0;
    int32_t smoothError = 0;
    int32_t totalValues = 0;
    int32_t proportionnalWeight;
    int32_t derivativeWeight;

    int32_t coefficientP;
    int32_t coefficientI;
    int32_t coefficientD;

    int32_t temporarym_inspiratoryPidIntegral = 0;

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
    m_inspiratoryPidLastErrorsIndex[m_inspiratoryPidLastErrorsIndex] = error;
    m_inspiratoryPidLastErrorsIndex++;
    if (m_inspiratoryPidLastErrorsIndex
        >= static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN)) {
        m_inspiratoryPidLastErrorsIndex = 0;
    }
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        totalValues += m_inspiratoryPidLastErrorsIndex[i];
    }
    smoothError = totalValues / static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN);

    // Fast mode ends at 20mmH20 from target. When changing from fast-mode to PID, Set the
    // integral to the previous value
    if (error < 20) {
        if (m_inspiratoryPidFastMode) {
            proportionnalWeight = (coefficientP * error) / 1000;
            derivativeWeight = (coefficientD * derivative / 1000);
            m_inspiratoryPidIntegral =
                1000 * ((int32_t)m_inspiratoryValveLastAperture - maxAperture)
                    / (minAperture - maxAperture)
                - (proportionnalWeight + derivativeWeight);
        }
        m_inspiratoryPidFastMode = false;
    }

    // In fast mode : everything is openned (Open loop)
    if (m_inspiratoryPidFastMode) {
        // Ramp from 125 to 0 angle during 250ms.
        int32_t increment = 5 * ((int32_t)mainController_COMPUTE_PERIOD_US) / 10000;
        if (m_inspiratoryValveLastAperture >= static_cast<uint32_t>(abs(increment))) {
            inspiratoryValveAperture = max(
                minAperture,
                min(maxAperture, (static_cast<int32_t>(m_inspiratoryValveLastAperture) - increment)));
        } else {
            inspiratoryValveAperture = 0;
        }
    }

    // In not fast mode the PID is used
    else {
        derivative =
            ((dt == 0)) ? 0 : ((1000000 * (m_inspiratoryPidLastError - smoothError)) / dt);

        temporarym_inspiratoryPidIntegral =
            m_inspiratoryPidIntegral + ((coefficientI * error * dt) / 1000000);
        temporarym_inspiratoryPidIntegral =
            max(PID_BLOWER_INTEGRAL_MIN,
                min(PID_BLOWER_INTEGRAL_MAX, temporarym_inspiratoryPidIntegral));

        proportionnalWeight = ((coefficientP * error) / 1000);
        int32_t integralWeight = temporarym_inspiratoryPidIntegral;
        derivativeWeight = coefficientD * derivative / 1000;

        int32_t blowerCommand = proportionnalWeight + integralWeight + derivativeWeight;
        inspiratoryValveAperture =
            max(minAperture,
                min(maxAperture, maxAperture + (minAperture - maxAperture) * blowerCommand / 1000));
    }

    // If the valve is completly open or completly closed, dont update Integral
    if ((inspiratoryValveAperture != static_cast<uint32_t>(minAperture))
        && (inspiratoryValveAperture != static_cast<uint32_t>(maxAperture))) {
        m_inspiratoryPidIntegral = temporarym_inspiratoryPidIntegral;
    }

    m_inspiratoryValveLastAperture = inspiratoryValveAperture;
    m_inspiratoryPidLastError = smoothError;

    return inspiratoryValveAperture;
}

int32_t
PC_CMV_Controller::PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt) {

    int32_t minAperture = expiratoryValve.minAperture();
    int32_t maxAperture = expiratoryValve.maxAperture();
    int32_t expiratoryValveAperture;
    int32_t derivative = 0;
    int32_t smoothError = 0;
    int32_t totalValues = 0;
    int32_t temporarym_expiratoryPidIntegral = 0;
    int32_t proportionnalWeight;
    int32_t derivativeWeight;

    int32_t coefficientP;
    int32_t coefficientI;
    int32_t coefficientD;

    // Compute error
    int32_t error = targetPressure + PID_PATIENT_SAFETY_PEEP_OFFSET - currentPressure;

    // Calculate Derivative part. Include a moving average on error for smoothing purpose
    m_expiratoryPidLastErrorsIndex[m_expiratoryPidLastErrorsIndex] = error;
    m_expiratoryPidLastErrorsIndex++;
    if (m_expiratoryPidLastErrorsIndex
        >= static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN)) {
        m_expiratoryPidLastErrorsIndex = 0;
    }
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        totalValues += m_expiratoryPidLastErrorsIndex[i];
    }
    smoothError = totalValues / static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN);
    derivative = (dt == 0) ? 0 : (1000000 * (smoothError - m_expiratoryPidLastError)) / dt;

    //  Windowing. It overides the parameter.h coefficients
    if (error < 0) {
        coefficientI = 50;
        coefficientP = 2500;
        coefficientD = 0;
    } else {
        // For a high peep, a lower KI is requiered. For Peep = 100mmH2O, KI = 120. For Peep =
        // 50mmH2O, KI = 250.
        if (mainController.peepCommand() > 100) {
            coefficientI = 120;
        } else {
            coefficientI = ((-130 * ((int32_t)mainController.peepCommand())) / 50) + 380;
        }

        coefficientP = 2500;
        coefficientD = 0;
    }

    // Fast mode ends at 30mmH20 from target. When changing from fast-mode to
    // PID, Set the integral to the previous value
    if (error > -30) {
        if (m_expiratoryPidFastMode) {
            proportionnalWeight = (coefficientP * error) / 1000;
            derivativeWeight = (coefficientD * derivative / 1000);
            m_expiratoryPidIntegral = 1000 * ((int32_t)m_expiratoryValveLastAperture - maxAperture)
                                             / (maxAperture - minAperture)
                                         - (proportionnalWeight + derivativeWeight);
        }
        m_expiratoryPidFastMode = false;
    }

    // Fast mode : open loop with ramp
    if (m_expiratoryPidFastMode) {
        // Ramp from 125 to 0 angle during 250ms.
        int32_t increment = 5 * ((int32_t)mainController_COMPUTE_PERIOD_US) / 10000;
        if (m_expiratoryValveLastAperture >= static_cast<uint32_t>(abs(increment))) {
            expiratoryValveAperture = max(
                minAperture,
                min(maxAperture, (static_cast<int32_t>(m_expiratoryValveLastAperture) - increment)));
        } else {
            expiratoryValveAperture = 0;
        }
    }

    // In not fast mode the PID is used
    else {
        temporarym_expiratoryPidIntegral =
            m_expiratoryPidIntegral + ((coefficientI * error * dt) / 1000000);
        temporarym_expiratoryPidIntegral =
            max(PID_PATIENT_INTEGRAL_MIN,
                min(PID_PATIENT_INTEGRAL_MAX, temporarym_expiratoryPidIntegral));

        proportionnalWeight = ((coefficientP * error) / 1000);
        int32_t integralWeight = temporarym_expiratoryPidIntegral;
        derivativeWeight = coefficientD * derivative / 1000;

        int32_t patientCommand = proportionnalWeight + integralWeight + derivativeWeight;

        expiratoryValveAperture = max(
            minAperture,
            min(maxAperture, maxAperture + (maxAperture - minAperture) * patientCommand / 1000));
    }

    // If the valve is completly open or completly closed, dont update Integral
    if ((expiratoryValveAperture != static_cast<uint32_t>(minAperture))
        && (expiratoryValveAperture != static_cast<uint32_t>(maxAperture))) {
        m_expiratoryPidIntegral = temporarym_expiratoryPidIntegral;
    }

    m_expiratoryPidLastError = smoothError;
    m_expiratoryValveLastAperture = expiratoryValveAperture;

    return expiratoryValveAperture;
}
