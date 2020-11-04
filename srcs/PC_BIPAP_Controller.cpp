

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

#include "../includes/pressure_controller.h"
#include "../includes/pressure_valve.h"

PC_BIPAP_Controller pcBIPAPController;

PC_BIPAP_Controller::PC_BIPAP_Controller() {}

void PC_BIPAP_Controller::setup() {

    inspiratoryValveLastAperture = inspiratoryValve.maxAperture();
    expiratoryValveLastAperture = expiratoryValve.maxAperture();
    m_plateauPressureReached = false;
    m_plateauStartTime = pController.tickPerInhalation();

    PC_inspiratory_PID_last_errorsIndex = 0;
    PC_expiratory_PID_last_errorsIndex = 0;
    for (uint8_t i = 0u; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        PC_inspiratory_PID_last_errors[i] = 0u;
        PC_expiratory_PID_last_errors[i] = 0u;
    }
}

void PC_BIPAP_Controller::initCycle() {

    m_plateauPressureReached = false;
    m_plateauStartTime = pController.tickPerInhalation();

    inspiratoryValveLastAperture = inspiratoryValve.maxAperture();
    expiratoryValveLastAperture = expiratoryValve.maxAperture();
    inspiratoryValveLastAperture = inspiratoryValve.maxAperture();
    expiratoryValveLastAperture = expiratoryValve.maxAperture();
    // Reset PID values
    PC_inspiratory_PID_integral = 0;
    PC_expiratory_PID_integral = 0;
    PC_inspiratory_PID_LastError = pController.plateauPressureCommand() - pController.peepCommand();
    PC_expiratory_PID_LastError = pController.peepCommand() - pController.plateauPressureCommand();
    PC_inspiratory_PID_fast_mode = true;
    PC_expiratory_PID_fast_mode = true;
    for (uint8_t i = 0; i < PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN; i++) {
        PC_inspiratory_PID_last_errors[i] = 0;
        PC_expiratory_PID_last_errors[i] =
            pController.peepCommand() - pController.plateauPressureCommand();
    }

    // Apply blower ramp-up
    if (m_blower_increment >= 0) {
        blower.runSpeed(blower.getSpeed() + static_cast<uint16_t>(abs(m_blower_increment)));
    } else {
        // When blower increment is negative, we need to check that it is less than current speed
        // If not, it would result in an overflow
        if (static_cast<uint16_t>(abs(m_blower_increment)) < blower.getSpeed()) {
            blower.runSpeed(blower.getSpeed() - static_cast<uint16_t>(abs(m_blower_increment)));
        } else {
            blower.runSpeed(MIN_BLOWER_SPEED);
        }
    }
    m_blower_increment = 0;

    m_reOpenInspiratoryValve = false;
}

void PC_BIPAP_Controller::inhale(uint16_t p_tick) {

    PC_expiratory_PID_fast_mode = false;

    // Keep the inspiratory valve open using a PID.
    int32_t inspiratoryValveOpenningValue =
        PCinspiratoryPID(pController.pressureCommand(), pController.pressure(), pController.dt());

    // Normally inspiratory vavle is open, but at the end of the cycle, it could be closed, and
    // expiratory valve will open
    inspiratoryValve.open(inspiratoryValveOpenningValue);
    expiratoryValve.close();
    /*if (inspiratoryPidValue >= 0 || !pController.isPlateauComputationStarted()) {
        inspiratoryValve.open(inspiratoryPidValue);
        expiratoryValve.close();
    } else {
        inspiratoryValve.close();
        expiratoryValve.open(PCexpiratoryPID(pController.pressureCommand(), pController.pressure(),
                                             pController.dt()));
    }*/

    PC_expiratory_PID_fast_mode = true;

    // m_plateauStartTime is used for blower regulations, -20 corresponds to open loop openning
    if (pController.pressure() > pController.plateauPressureCommand() - 20u
        && !m_plateauPressureReached) {
        m_plateauStartTime = p_tick;
        m_inspiratorySlope = (pController.pressure() - pController.peepMeasure()) * 100
                             / (p_tick - 0);  // in mmH2O/s
        m_plateauPressureReached = true;
    }
}

void PC_BIPAP_Controller::exhale() {

    // Open the expiratos valve so the patient can exhale outside
    int32_t inspiratoryValveOpenningValue =
        PCexpiratoryPID(pController.pressureCommand(), pController.pressure(), pController.dt());

    if (inspiratoryValveOpenningValue > 90) {
        m_reOpenInspiratoryValve = true;
    }
    expiratoryValve.open(inspiratoryValveOpenningValue);
    if (m_reOpenInspiratoryValve) {
        // slightly reopen inspiratory valve. This create a circulatory flow and make it possible to
        // detect an inspiratory trigger
        inspiratoryValve.open(100);
    } else {
        // Close the inspiratory valve
        inspiratoryValve.close();
    }

    // In case the pressure trigger mode is enabled, check if inspiratory trigger is raised
    if (pController.triggerModeEnabledCommand() && pController.isPeepDetected()) {
        // m_peakPressure > CONST_MIN_PEAK_PRESSURE ensure that the patient is plugged on the
        // machine.
        if (static_cast<int32_t>(pController.pressure())
                < (pController.pressureCommand()
                   - static_cast<int32_t>(pController.pressureTriggerOffsetCommand()))
            && (pController.peakPressureMeasure() > CONST_MIN_PEAK_PRESSURE)) {
            pController.setTrigger(true);
        }
    }
}

void PC_BIPAP_Controller::endCycle() { calculateBlowerIncrement(); }

void PC_BIPAP_Controller::calculateBlowerIncrement() {
    int16_t peakDelta = pController.peakPressureMeasure() - pController.plateauPressureCommand();
    int16_t rebouncePeakDelta =
        pController.rebouncePeakPressureMeasure() - pController.plateauPressureCommand();
    DBG_DO(Serial.print("peakPressure:");)
    DBG_DO(Serial.println(pController.peakPressureMeasure());)
    DBG_DO(Serial.print("plateauPressureCommand:");)
    DBG_DO(Serial.println(pController.plateauPressureCommand());)

    // Check that pressure didnt rebounced.
    // High rebounces will decrease the blower. Low rebounce will prevent increase (but not
    // decrease)
    bool veryHighRebounce = (peakDelta > 60) || (rebouncePeakDelta < -60 && peakDelta >= 0);
    bool highRebounce = (peakDelta > 40) || (rebouncePeakDelta < -40 && peakDelta >= 0);
    bool lowRebounce = (peakDelta > 20) || (rebouncePeakDelta < -15 && peakDelta >= 0);
    bool veryLowRebounce = (peakDelta > 10) || (rebouncePeakDelta < -10 && peakDelta >= 0);

    // Update blower only if patient is plugged on the machine
    if (pController.peakPressureMeasure() > 20) {
        // Safety condition : A very too high peak (4cmH2O) should decrease the blower
        if (veryHighRebounce) {
            m_blower_increment = -100;
        } else if (highRebounce) {
            m_blower_increment = -10;
        }
        // We want the m_inspiratorySlope = 600 mmH2O/s
        else if (m_inspiratorySlope > 650) {
            // Only case for decreasing the blower : ramping is too fast or overshooting is too high
            if (m_inspiratorySlope > 1000 || (lowRebounce && (m_inspiratorySlope > 800))
                || peakDelta > 25) {
                m_blower_increment = -100;
            } else {
                m_blower_increment = 0;
            }
        } else if (m_inspiratorySlope > 550) {
            m_blower_increment = 0;
        } else if (m_inspiratorySlope > 450 && !veryLowRebounce) {
            m_blower_increment = +15;
        } else if (m_inspiratorySlope > 350 && !veryLowRebounce) {
            m_blower_increment = +30;
        } else if (m_inspiratorySlope > 250 && !veryLowRebounce) {
            m_blower_increment = +50;
        } else if (!lowRebounce) {
            m_blower_increment = +100;
        }
    }
    DBG_DO(Serial.print("BLOWER"));
    DBG_DO(Serial.println(m_blower_increment));
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

    int32_t temporaryPC_inspiratory_PID_integral = 0;

    // Compute error
    int32_t error = targetPressure - currentPressure;

    // Windowing. It overides the parameter.h coefficients
    // For a high plateau pressure, a lower KP is requiered. For  = 100mmH2O, KP = 120. For  =
    // 50mmH2O, KP = 250.
    /*if (pController.plateauPressureCommand() > 300) {
        coefficientP = 1000;
    } else {
        */
    coefficientP = 2500;
    //}
    coefficientD = 0;
    if (error < 0) {
        coefficientI = 200;

    } else {
        coefficientI = 50;
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
PC_BIPAP_Controller::PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt) {

    int32_t minAperture = expiratoryValve.minAperture();
    int32_t maxAperture = expiratoryValve.maxAperture();
    int32_t expiratoryValveAperture;
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
        coefficientI = 250;
        coefficientP = 2500;
        coefficientD = 0;
    } else {
        // For a high peep, a lower KI is requiered. For Peep = 100mmH2O, KI = 120. For Peep =
        // 50mmH2O, KI = 250.
        if (pController.peepCommand() > 100) {
            coefficientI = 120;
        } else {
            coefficientI = ((-130 * ((int32_t)pController.peepCommand())) / 50) + 380;
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
