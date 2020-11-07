

/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure.cpp
 * @brief Pressure sensor related functions
 *****************************************************************************/

// INCLUDES ==================================================================

// Associated header
#include "../includes/PC_BIPAP_Controller.h"

// External
#include "Arduino.h"
#include <algorithm>

// Internal
#include "../includes/main_controller.h"
#include "../includes/pressure_valve.h"

PC_BIPAP_Controller pcBipapController;

PC_BIPAP_Controller::PC_BIPAP_Controller() {}

void PC_BIPAP_Controller::setup() {

    m_inspiratoryValveLastAperture = inspiratoryValve.maxAperture();
    m_expiratoryValveLastAperture = expiratoryValve.maxAperture();
    m_plateauPressureReached = false;
    m_triggerWindow = mainController.tickPerInhalation() + 100;  // Possible to trigger 1s before end

    m_inspiratoryFlowLastValuesIndex = 0;
    m_inspiratoryPidLastErrorsIndex = 0;
    m_expiratoryPidLastErrorsIndex = 0;
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        m_inspiratoryPidLastErrors[i] = 0u;
        m_expiratoryPidLastErrors[i] = 0u;
    }
}

void PC_BIPAP_Controller::initCycle() {

    m_plateauPressureReached = false;
    m_triggerWindow = mainController.tickPerInhalation() + 140;  // Possible to trigger 1s before end

    // m_inspiratoryValveLastAperture = inspiratoryValve.maxAperture();
    m_expiratoryValveLastAperture = expiratoryValve.maxAperture();
    // Reset PID values
    m_inspiratoryPidIntegral = 0;
    m_expiratoryPidIntegral = 0;
    m_inspiratoryPidLastError =
        mainController.plateauPressureCommand() - mainController.peepCommand();
    m_expiratoryPidLastError =
        mainController.peepCommand() - mainController.plateauPressureCommand();
    m_inspiratoryPidFastMode = true;
    m_expiratoryPidFastMode = true;
    for (uint8_t i = 0; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        m_inspiratoryPidLastErrors[i] = 0;
        m_expiratoryPidLastErrors[i] =
            mainController.peepCommand() - mainController.plateauPressureCommand();
    }

    for (uint8_t i = 0u; i < NUMBER_OF_SAMPLE_FLOW_LAST_VALUES; i++) {
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

    m_reOpenInspiratoryValve = false;
}

void PC_BIPAP_Controller::inhale() {

    m_expiratoryPidFastMode = false;

    // Keep the inspiratory valve open using a PID.
    int32_t inspiratoryValveOpenningValue = PCinspiratoryPID(
        mainController.pressureCommand(), mainController.pressure(), mainController.dt());

    // Normally inspiratory vavle is open, but at the end of the cycle, it could be closed, and
    // expiratory valve will open
    inspiratoryValve.open(inspiratoryValveOpenningValue);
    expiratoryValve.close();

    m_expiratoryPidFastMode = true;

    // m_inspiratorySlope is used for blower regulations, -20 corresponds to open loop openning
    if (mainController.pressure() > mainController.plateauPressureCommand() - 20u
        && !m_plateauPressureReached) {
        m_inspiratorySlope = (mainController.pressure() - mainController.peepMeasure()) * 100
                             / (mainController.tick() - 0);  // in mmH2O/s
        m_plateauPressureReached = true;
    }
}

void PC_BIPAP_Controller::exhale() {
    blower.runSpeed(m_blowerSpeed - 400);  // todo check min blower speed

    // Open the expiratos valve so the patient can exhale outside
    int32_t expiratoryValveOpenningValue = PCexpiratoryPID(
        mainController.pressureCommand(), mainController.pressure(), mainController.dt());

    expiratoryValve.open(expiratoryValveOpenningValue);

    int32_t inspiratoryValveOpenningValue =
        max((uint32_t)70, 125 - (mainController.tick() - mainController.tickPerInhalation()) / 2);
    inspiratoryValve.open(inspiratoryValveOpenningValue);
    m_inspiratoryValveLastAperture = inspiratoryValveOpenningValue;

    // In case the pressure trigger mode is enabled, check if inspiratory trigger is raised
    // m_peakPressure > CONST_MIN_PEAK_PRESSURE ensure that the patient is plugged on the machine.
    if ( mainController.tick() < m_triggerWindow) {
        m_inspiratoryFlowLastValues[m_inspiratoryFlowLastValuesIndex] =
            mainController.inspiratoryFlow();
        m_inspiratoryFlowLastValuesIndex++;
        if (m_inspiratoryFlowLastValuesIndex >= NUMBER_OF_SAMPLE_FLOW_LAST_VALUES) {
            m_inspiratoryFlowLastValuesIndex = 0;
        }
    }

    if (mainController.triggerModeEnabledCommand() ) {

        // triggering an inspiration is only possible within a time window
        if (mainController.tick() >= m_triggerWindow) {
            int32_t sum = 0;
            for (uint8_t i = 0u; i < NUMBER_OF_SAMPLE_FLOW_LAST_VALUES; i++) {
                sum += m_inspiratoryFlowLastValues[i];
            }
            int32_t meanFLow = sum / NUMBER_OF_SAMPLE_FLOW_LAST_VALUES;

            if (mainController.inspiratoryFlow() > 280*100 + 100*mainController.pressureTriggerOffsetCommand() ){
                mainController.setTrigger(true);
            }

            
        }
    }
}

void PC_BIPAP_Controller::endCycle() { calculateBlowerIncrement(); }

void PC_BIPAP_Controller::calculateBlowerIncrement() {
    int16_t peakDelta =
        mainController.peakPressureMeasure() - mainController.plateauPressureCommand();
    int16_t rebouncePeakDelta =
        mainController.rebouncePeakPressureMeasure() - mainController.plateauPressureCommand();
    DBG_DO(Serial.print("peakPressure:");)
    DBG_DO(Serial.println(mainController.peakPressureMeasure());)
    DBG_DO(Serial.print("plateauPressureCommand:");)
    DBG_DO(Serial.println(mainController.plateauPressureCommand());)

    // Check that pressure didnt rebounced.
    // High rebounces will decrease the blower. Low rebounce will prevent increase (but not
    // decrease)
    bool veryHighRebounce = (peakDelta > 60) || (rebouncePeakDelta < -60 && peakDelta >= 0);
    bool highRebounce = (peakDelta > 40) || (rebouncePeakDelta < -40 && peakDelta >= 0);
    bool lowRebounce = (peakDelta > 20) || (rebouncePeakDelta < -15 && peakDelta >= 0);
    bool veryLowRebounce = (peakDelta > 10) || (rebouncePeakDelta < -10 && peakDelta >= 0);

    // Update blower only if patient is plugged on the machine
    if (mainController.peakPressureMeasure() > 20) {
        // Safety condition : A very too high peak (4cmH2O) should decrease the blower
        if (veryHighRebounce) {
            m_blowerIncrement = -100;
        } else if (highRebounce) {
            m_blowerIncrement = -10;
        }
        // We want the m_inspiratorySlope = 600 mmH2O/s
        else if (m_inspiratorySlope > 650) {
            // Only case for decreasing the blower : ramping is too fast or overshooting is too high
            if (m_inspiratorySlope > 1000 || (lowRebounce && (m_inspiratorySlope > 800))
                || peakDelta > 25) {
                m_blowerIncrement = -100;
            } else {
                m_blowerIncrement = 0;
            }
        } else if (m_inspiratorySlope > 550) {
            m_blowerIncrement = 0;
        } else if (m_inspiratorySlope > 450 && !veryLowRebounce) {
            m_blowerIncrement = +25;
        } else if (m_inspiratorySlope > 350 && !veryLowRebounce) {
            m_blowerIncrement = +50;
        } else if (m_inspiratorySlope > 250 && !veryLowRebounce) {
            m_blowerIncrement = +75;
        } else if (!lowRebounce) {
            m_blowerIncrement = +100;
        }
    }
    DBG_DO(Serial.print("BLOWER"));
    DBG_DO(Serial.println(m_blowerIncrement));
    DBG_DO(Serial.print("m_inspiratorySlope:");)
    DBG_DO(Serial.println(m_inspiratorySlope);)
}

int32_t
PC_BIPAP_Controller::PCinspiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt) {

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
    coefficientP = 2500;

    coefficientD = 0;
    if (error < 0) {
        coefficientI = 200;

    } else {
        coefficientI = 50;
    }

    // Calculate Derivative part. Include a moving average on error for smoothing purpose
    m_inspiratoryPidLastErrors[m_inspiratoryPidLastErrorsIndex] = error;
    m_inspiratoryPidLastErrorsIndex++;
    if (m_inspiratoryPidLastErrorsIndex
        >= static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN)) {
        m_inspiratoryPidLastErrorsIndex = 0;
    }
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        totalValues += m_inspiratoryPidLastErrors[i];
    }
    smoothError = totalValues / static_cast<int32_t>(PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN);

    // Fast mode ends at 20mmH20 from target. When changing from fast-mode to PID, Set the
    // integral to the previous value
    if (error < 20) {
        if (m_inspiratoryPidFastMode) {
            proportionnalWeight = (coefficientP * error) / 1000;
            derivativeWeight = (coefficientD * derivative / 1000);
            m_inspiratoryPidIntegral = 1000
                                           * ((int32_t)m_inspiratoryValveLastAperture - maxAperture)
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
            inspiratoryValveAperture =
                max(minAperture,
                    min(maxAperture,
                        (static_cast<int32_t>(m_inspiratoryValveLastAperture) - increment)));
        } else {
            inspiratoryValveAperture = 0;
        }
    }

    // In not fast mode the PID is used
    else {
        derivative = ((dt == 0)) ? 0 : ((1000000 * (m_inspiratoryPidLastError - smoothError)) / dt);

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
PC_BIPAP_Controller::PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt) {

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
        expiratoryValveAperture = 0;
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
