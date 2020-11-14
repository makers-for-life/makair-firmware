/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file vc_prvc_controller.cpp
 * @brief PID for BIPAP pressure control
 *****************************************************************************/

// INCLUDES ==================================================================

// Associated header
#include "../includes/vc_prvc_controller.h"

// External
#include "Arduino.h"
#include <algorithm>

// Internal
#include "../includes/main_controller.h"
#include "../includes/pressure_valve.h"


// INITIALISATION =============================================================

VC_PRVC_Controller vcPRVCController;

// FUNCTIONS ==================================================================

// cppcheck-suppress misra-c2012-5.2 ; false positive
VC_PRVC_Controller::VC_PRVC_Controller() {
    m_inspiratoryValveLastAperture = inspiratoryValve.maxAperture();
    m_expiratoryValveLastAperture = expiratoryValve.maxAperture();

    m_inspiratoryFlowLastValuesIndex = 0;
    m_expiratoryPidLastErrorsIndex = 0;
    for (uint8_t i = 0u; i < NUMBER_OF_SAMPLE_LAST_VALUES; i++) {
        m_inspiratoryFlowLastValues[i] = 0u;
    }
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        m_expiratoryPidLastErrors[i] = 0u;
    }

    m_blowerSpeed = DEFAULT_BLOWER_SPEED;
    m_blowerIncrement = 0;
    m_expiratoryPidFastMode = true;
    m_expiratoryPidIntegral = 0;
    m_expiratoryPidLastError = 0;
    m_maxInspiratoryFlow = 0;
}

void VC_PRVC_Controller::setup() {
    // No specific setup code
}

void VC_PRVC_Controller::initCycle() {
    m_maxInspiratoryFlow = 0;
    m_expiratoryValveLastAperture = expiratoryValve.maxAperture();
    // Reset PID values
    m_expiratoryPidIntegral = 0;
    m_expiratoryPidLastError =
        mainController.peepCommand() - mainController.plateauPressureCommand();
    m_expiratoryPidFastMode = true;
    for (uint8_t i = 0; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        m_expiratoryPidLastErrors[i] =
            mainController.peepCommand() - mainController.plateauPressureCommand();
    }

    for (uint8_t i = 0u; i < NUMBER_OF_SAMPLE_LAST_VALUES; i++) {
        m_inspiratoryFlowLastValues[i] = 0u;
    }
    m_inspiratoryFlowLastValuesIndex = 0;

    // Apply blower ramp-up
    if (m_blowerIncrement >= 0) {
        blower.runSpeed(m_blowerSpeed + static_cast<uint16_t>(abs(m_blowerIncrement)));
    } else {
        // When blower increment is negative, we need to check that it is less than current speed
        // If not, it would result in an overflow
        if (static_cast<uint16_t>(abs(m_blowerIncrement)) < blower.getSpeed()) {
            blower.runSpeed(m_blowerSpeed - static_cast<uint16_t>(abs(m_blowerIncrement)));
        } else {
            blower.runSpeed(MIN_BLOWER_SPEED);
        }
    }
    m_blowerSpeed = blower.getSpeed();
    m_blowerIncrement = 0;
}

void VC_PRVC_Controller::inhale() {
    m_inspiratoryValveLastAperture = inspiratoryValve.maxAperture();

    expiratoryValve.close();

    if (mainController.tick() < mainController.ticksPerInhalation() * 0.33) {
        inspiratoryValve.open();
    } else {
        inspiratoryValve.close();
    }


    if (mainController.inspiratoryFlow() > m_maxInspiratoryFlow) {
        m_maxInspiratoryFlow = mainController.inspiratoryFlow();
    }
}

void VC_PRVC_Controller::exhale() {
    // Open the expiration valve so the patient can exhale outside
    int32_t expiratoryValveOpenningValue = PCexpiratoryPID(
        mainController.pressureCommand(), mainController.pressure(), mainController.dt());

    (void)expiratoryValve.openLinear(expiratoryValveOpenningValue);

    inspiratoryValve.close();
    // m_inspiratoryValveLastAperture = inspiratoryValveOpenningValue;
}

void VC_PRVC_Controller::endCycle() { calculateBlowerIncrement(); }

void VC_PRVC_Controller::calculateBlowerIncrement() {
    m_blowerIncrement = 0;
    int32_t difference = 300 - mainController.tidalVolumeMeasure(); 

    if (difference > 30) {
        m_blowerIncrement = 40;
    } else if (difference < 0) {
        m_blowerIncrement = -40;
    }

    Serial.println(mainController.tidalVolumeMeasure());
}

int32_t
VC_PRVC_Controller::PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt) {
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

    // Calculate derivative part
    // Include a moving average on error for smoothing purpose
    m_expiratoryPidLastErrors[m_expiratoryPidLastErrorsIndex] = error;
    m_expiratoryPidLastErrorsIndex++;
    if (m_expiratoryPidLastErrorsIndex
        >= static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN)) {
        m_expiratoryPidLastErrorsIndex = 0;
    }
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        totalValues += m_expiratoryPidLastErrors[i];
    }
    smoothError = totalValues / static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN);
    derivative = (dt == 0) ? 0 : (1000000 * (smoothError - m_expiratoryPidLastError)) / dt;

    //  Windowing (it overrides the parameter.h coefficients)
    if (error < 0) {
        coefficientI = 50;
        coefficientP = 2500;
        coefficientD = 0;
    } else {
        // For a high PEEP, a lower KI is required
        // For PEEP = 100 mmH2O, KI = 120
        // For PEEP = 50 mmH2O, KI = 250
        if (mainController.peepCommand() > 100) {
            coefficientI = 120;
        } else {
            coefficientI = ((-130 * ((int32_t)mainController.peepCommand())) / 50) + 380;
        }
        coefficientP = 2500;
        coefficientD = 0;
    }

    // Fast mode ends at 30 mmH20 from target
    // When changing from fast mode to PID, set the integral to the previous value
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

    // Fast mode: open loop with ramp
    if (m_expiratoryPidFastMode) {
        expiratoryValveAperture = 0;
    } else {  // If not in fast mode, the PID is used
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

    // If the valve is completely open or completely closed, don't update integral part
    if ((expiratoryValveAperture != minAperture) && (expiratoryValveAperture != maxAperture)) {
        m_expiratoryPidIntegral = temporarym_expiratoryPidIntegral;
    }

    m_expiratoryPidLastError = smoothError;
    m_expiratoryValveLastAperture = expiratoryValveAperture;

    return expiratoryValveAperture;
}
