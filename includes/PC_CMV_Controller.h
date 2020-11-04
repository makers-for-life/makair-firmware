/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure.h
 * @brief PID pour pressure control
 *****************************************************************************/

#pragma once

#include "../includes/VentilationController.h"
#include "../includes/parameters.h"

class PC_CMV_Controller final : public VentilationController {
 public:
    PC_CMV_Controller();
    void setup();
    void initCycle();
    void inhale(uint16_t p_tick);
    void exhale();
    void endCycle();

 private:
    void calculateBlowerIncrement();
    /// Number of tick when plateau is reached for the first time
    uint16_t m_plateauStartTime;

    /// true if plateau pressure has been reached (but not necessarily converged.)
    bool m_plateauPressureReached;
    /**
     * PID to controller the blower valve during some specific steps of the cycle
     *
     * @param targetPressure The pressure we want (in mmH2O)
     * @param currentPressure The pressure measured by the sensor (in mmH2O)
     * @param dt Time since the last computation (in microsecond)
     */
    int32_t PCinspiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt);

    /**
     * PID to controller the patient valve during some specific steps of the cycle
     *
     * @param targetPressure The pressure we want (in mmH2O)
     * @param currentPressure The pressure measured by the sensor (in mmH2O)
     * @param dt Time since the last computation (in microsecond)
     */
    int32_t PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt);

    int32_t m_blower_increment;
    /// Error of the last computation of the blower PID
    int32_t PC_inspiratory_PID_integral;

    /// Error of the last computation of the blower PID
    int32_t PC_inspiratory_PID_LastError;

    /// Fast mode at start of expiration
    bool PC_expiratory_PID_fast_mode;

    /// Fast mode at start of inspiration
    bool PC_inspiratory_PID_fast_mode;

    /// Integral gain of the patient PID
    int32_t PC_expiratory_PID_integral;

    /// Last aperture of the blower valve
    int32_t inspiratoryValveLastAperture;

    /// Last aperture of the blower valve
    int32_t expiratoryValveLastAperture;

    /// Error of the last computation of the patient PID
    int32_t PC_expiratory_PID_LastError;
    /// Last error in inspiratory PID
    int32_t PC_inspiratory_PID_last_errors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    int32_t PC_inspiratory_PID_last_errorsIndex;

    /// Last error in expiratory PID
    int32_t PC_expiratory_PID_last_errors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    int32_t PC_expiratory_PID_last_errorsIndex;
};

extern PC_CMV_Controller pcCmvController;