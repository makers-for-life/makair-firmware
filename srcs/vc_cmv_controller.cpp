/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file VC_CMV_Controller.cpp
 * @brief PID for VC_CMV control
 *****************************************************************************/

// INCLUDES ==================================================================

// Associated header
#include "../includes/vc_cmv_controller.h"

// External
#include "Arduino.h"
#include <algorithm>

// Internal
#include "../includes/main_controller.h"
#include "../includes/pressure_valve.h"

// INITIALISATION =============================================================

VC_CMV_Controller vcCmvController;

// FUNCTIONS ==================================================================

// cppcheck-suppress misra-c2012-5.2 ; false positive
VC_CMV_Controller::VC_CMV_Controller() {
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
    m_blowerTicks = 0;
    m_expiratoryPidFastMode = true;
    m_expiratoryPidIntegral = 0;
    m_inspiratoryPidIntegral = 0;
    m_expiratoryPidLastError = 0;
    m_maxInspiratoryFlow = 0;
    m_targetFlowMultiplyBy1000 = 0;
}

void VC_CMV_Controller::setup() {
    // No specific setup code
}

void VC_CMV_Controller::initCycle() {
    m_maxInspiratoryFlow = 0;
    m_expiratoryValveLastAperture = expiratoryValve.maxAperture();
    // Reset PID values
    m_expiratoryPidIntegral = 0;
    m_inspiratoryPidIntegral = 0;
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

    calculateBlower();
    blower.runSpeedWithRampUp(m_blowerSpeed);
    m_blowerSpeed = blower.getTargetSpeed();
    mainController.ticksPerInhalationSet(mainController.ticksPerInhalation() + 50);
    m_duringPlateau = false;
}

void VC_CMV_Controller::inhale() {
    m_inspiratoryValveLastAperture = inspiratoryValve.maxAperture();

    expiratoryValve.close();

    int32_t inspirationRemainingDurationMs =
        ((mainController.ticksPerInhalation() - mainController.tick())
             * MAIN_CONTROLLER_COMPUTE_PERIOD_MS
         - mainController.plateauDurationCommand());  // in ms

    if (inspirationRemainingDurationMs > 20) {
        /*m_targetFlowMultiplyBy1000 =
            (60 * 1000
             * (mainController.tidalVolumeCommand() - mainController.currentDeliveredVolume()))
            / inspirationRemainingDurationMs;  // in mL/min
        m_targetFlowMultiplyBy1000 = max(int32_t(0), m_targetFlowMultiplyBy1000);*/
        m_targetFlowMultiplyBy1000 = mainController.targetInspiratoryFlowCommand();
    }

    int32_t safetyVolume = mainController.inspiratoryFlow() * VALVE_RESPONSE_TIME_MS
                           / 60000;  // todo check greater than volume target
    if (!m_duringPlateau
        && mainController.currentDeliveredVolume()
               > mainController.tidalVolumeCommand() - safetyVolume) {
        mainController.ticksPerInhalationSet(mainController.tick()
                                             + mainController.plateauDurationCommand()
                                                   / MAIN_CONTROLLER_COMPUTE_PERIOD_MS);
        m_duringPlateau = true;
    }
    if (mainController.tick()
        < mainController.ticksPerInhalation()
              - mainController.plateauDurationCommand() / MAIN_CONTROLLER_COMPUTE_PERIOD_MS) {
        int32_t flow = mainController.inspiratoryFlow();
        int32_t blowerPressure = blower.getBlowerPressure(flow);
        int32_t patientPressure = mainController.pressure();

        int32_t A1MultiplyBy100 = 3318;
        int32_t rhoMultiplyBy100 = 120;
        int32_t twoA1SquareDotDeltaPressureMultiplyBy100 =
            100 * 2 * (A1MultiplyBy100 * A1MultiplyBy100 / 10000)
            * (98 * (blowerPressure - patientPressure) / 10);
        int32_t divider = (rhoMultiplyBy100 * (m_targetFlowMultiplyBy1000 / 60)
                           * (m_targetFlowMultiplyBy1000 / 60) / 100);
        int32_t tempRatio = (divider == 0) ? 0 : twoA1SquareDotDeltaPressureMultiplyBy100 / divider;
        int32_t sectionToOpen;
        if (m_targetFlowMultiplyBy1000 == 0) {
            sectionToOpen = 0;

        } else {
            int32_t divider2 = (tempRatio + 100 < 0) ? 0 : sqrt(tempRatio + 100);
            sectionToOpen = (divider2 == 0) ? 0 : (A1MultiplyBy100 * 10 / divider2);
        }
        inspiratoryValve.openSection(sectionToOpen);

    } else {
        inspiratoryValve.close();
    }

    if (mainController.inspiratoryFlow() > m_maxInspiratoryFlow) {
        m_maxInspiratoryFlow = mainController.inspiratoryFlow();
    }
}

void VC_CMV_Controller::exhale() {
    // Open the expiration valve so the patient can exhale outside
    int32_t expiratoryValveOpenningValue = PCexpiratoryPID(
        mainController.pressureCommand(), mainController.pressure(), mainController.dt());

    (void)expiratoryValve.openLinear(expiratoryValveOpenningValue);

    inspiratoryValve.close();
    // m_inspiratoryValveLastAperture = inspiratoryValveOpenningValue;
}

void VC_CMV_Controller::endCycle() {}

void VC_CMV_Controller::calculateBlower() {
    int32_t inspirationDurationMs =
        (mainController.ticksPerInhalation() * MAIN_CONTROLLER_COMPUTE_PERIOD_MS
         - mainController.plateauDurationCommand());  // in ms
    m_targetFlowMultiplyBy1000 = (inspirationDurationMs == 0)
                                     ? 0
                                     : (60 * 1000 * mainController.tidalVolumeCommand())
                                           / inspirationDurationMs;  // in mL/min

    // 40L/min -> max blower (1800) ; 6L/min -> min blower (300)
    m_blowerSpeed = 1800;
}

int32_t
VC_CMV_Controller::PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt) {
    int32_t minAperture = expiratoryValve.minAperture();
    int32_t maxAperture = expiratoryValve.maxAperture();
    int32_t expiratoryValveAperture;
    int32_t derivative = 0;
    int32_t smoothError = 0;
    int32_t totalValues = 0;
    int32_t temporaryExpiratoryPidIntegral = 0;
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
        temporaryExpiratoryPidIntegral =
            m_expiratoryPidIntegral + ((coefficientI * error * dt) / 1000000);
        temporaryExpiratoryPidIntegral =
            max(PID_PATIENT_INTEGRAL_MIN,
                min(PID_PATIENT_INTEGRAL_MAX, temporaryExpiratoryPidIntegral));

        proportionnalWeight = ((coefficientP * error) / 1000);
        int32_t integralWeight = temporaryExpiratoryPidIntegral;
        derivativeWeight = coefficientD * derivative / 1000;

        int32_t patientCommand = proportionnalWeight + integralWeight + derivativeWeight;

        expiratoryValveAperture = max(
            minAperture,
            min(maxAperture, maxAperture + (maxAperture - minAperture) * patientCommand / 1000));
    }

    // If the valve is completely open or completely closed, don't update integral part
    if ((expiratoryValveAperture != minAperture) && (expiratoryValveAperture != maxAperture)) {
        m_expiratoryPidIntegral = temporaryExpiratoryPidIntegral;
    }

    m_expiratoryPidLastError = smoothError;
    m_expiratoryValveLastAperture = expiratoryValveAperture;

    return expiratoryValveAperture;
}
