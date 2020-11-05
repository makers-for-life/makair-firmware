/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure.h
 * @brief PID pour pressure control
 *****************************************************************************/

#pragma once

#include "../includes/parameters.h"
#include "../includes/VentilationController.h"

class PC_BIPAP_Controller final : public VentilationController{
 public:
    PC_BIPAP_Controller();
    void setup();
    void initCycle();
    void inhale(uint16_t p_tick);
    void exhale(uint16_t p_tick);
    void endCycle();
    

 private:
    void calculateBlowerIncrement();

    /// Number of tick from which it is possible to trigger a new inspiration
    uint16_t m_triggerWindow;

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

    int32_t m_blowerSpeed;

    /// True if we want to reopen the inspiratory vavle to create a flow circulation able to detecte inspiratory flow trigger
    bool m_reOpenInspiratoryValve;

    /// Slope of the inspiration open loop. In mmH2O/s
    int32_t m_inspiratorySlope;

    int32_t m_blowerIncrement;
    /// Error of the last computation of the blower PID
    int32_t m_inspiratoryPidIntegral;

    ///Error of the last computation of the blower PID
    int32_t m_inspiratoryPidLastError;

    ///Fast mode at start of expiration
    bool m_expiratoryPidFastMode;

    /// Fast mode at start of inspiration
    bool m_inspiratoryPidFastMode;

    /// Integral gain of the patient PID
    int32_t m_expiratoryPidIntegral;

    /// Last aperture of the blower valve
    int32_t m_inspiratoryValveLastAperture;

    /// Last aperture of the blower valve
    int32_t m_expiratoryValveLastAperture;

    /// Error of the last computation of the  PID
    int32_t m_expiratoryPidLastError;
    /// Last error in inspiratory PID
    int32_t m_inspiratoryPidLastErrors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    int32_t m_inspiratoryPidLastErrorsIndex;

    /// Last error in expiratory PID
    int32_t m_expiratoryPidLastErrors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    int32_t m_expiratoryPidLastErrorsIndex;
};

extern PC_BIPAP_Controller pcBipapController;