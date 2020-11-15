/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file VC_CMV_Controller.h
 * @brief PID for Volume control
 *****************************************************************************/

#pragma once

#include "../includes/parameters.h"
#include "../includes/ventilation_controller.h"

/// Controller for the Volume Controled mode
class VC_CMV_Controller : public VentilationController {
 public:
    /// Default constructor
    VC_CMV_Controller();

    /// Initialize controller
    void setup() override;

    /// Begin a new breathing cycle
    void initCycle() override;

    /// Control the inhalation
    void inhale() override;

    /// Control the exhalation
    void exhale() override;

    /// End the current breathing cycle
    void endCycle() override;

 private:
    /// Determine the blower speed to adopt for next cycle
    void calculateBlower();

    /**
     * PID to control the patient valve during some specific steps of the cycle
     *
     * @param targetPressure The pressure we want (in mmH2O)
     * @param currentPressure The pressure measured by the sensor (in mmH2O)
     * @param dt Time since the last computation (in microsecond)
     */
    int32_t PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt);

    int32_t VCinspiratoryPID(int32_t targetFlow, int32_t currentFlow, int32_t dt);

    int32_t m_volume;
    /// Current blower speed
    uint16_t m_blowerSpeed;

    int32_t m_targetFlowMultiplyBy1000;

    /// Slope of the inspiration open loop (in mmH2O/s)
    int32_t m_inspiratorySlope;

    /// Fast mode at start of expiration
    bool m_expiratoryPidFastMode;

    /// Integral gain of the expiratory PID
    int32_t m_expiratoryPidIntegral;

    /// Integral gain of the inspiratory PID
    int32_t m_inspiratoryPidIntegral;

    /// Last aperture of the inspiratory valve
    int32_t m_inspiratoryValveLastAperture;

    /// Last aperture of the blower valve
    int32_t m_expiratoryValveLastAperture;

    /// Error of the last computation of the PID
    int32_t m_expiratoryPidLastError;
    /// Last errors in expiratory PID
    int32_t m_expiratoryPidLastErrors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    /// Last error index in expiratory PID
    int32_t m_expiratoryPidLastErrorsIndex;

    /// Error of the last computation of the expiratory PID
    int32_t m_inspiratoryPidLastError;
    /// Last errors in inspiratory PID
    int32_t m_inspiratoryPidLastErrors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    /// Last error index in inspiratory PID
    int32_t m_inspiratoryPidLastErrorsIndex;

    /// Last flow values
    int32_t m_inspiratoryFlowLastValues[NUMBER_OF_SAMPLE_LAST_VALUES];

    /// Last pressure values
    int32_t m_inspiratoryPressureLastValues[NUMBER_OF_SAMPLE_LAST_VALUES];

    /// Last flow index
    int32_t m_inspiratoryFlowLastValuesIndex;

    /// Last pressure index
    int32_t m_inspiratoryPressureLastValuesIndex;

    /// Max flow during inspiration
    int32_t m_maxInspiratoryFlow;

    /// Blower ticks
    int32_t m_blowerTicks;
};

extern VC_CMV_Controller vcCmvController;