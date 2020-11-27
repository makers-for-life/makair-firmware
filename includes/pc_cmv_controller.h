/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pc_cmv_controller.h
 * @brief PID for CMV pressure control
 *****************************************************************************/

#pragma once

#include "../includes/parameters.h"
#include "../includes/ventilation_controller.h"

/// Controller for the CMV mode
class PC_CMV_Controller : public VentilationController {
 public:
    /// Default constructor
    PC_CMV_Controller();

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

    /// List of alarms that must be enabled for this mode
    struct Alarms enabledAlarms() const {
        struct Alarms a = {RCM_SW_1,  RCM_SW_2,  RCM_SW_3,  RCM_SW_11, RCM_SW_12,
                           RCM_SW_14, RCM_SW_15, RCM_SW_16, RCM_SW_18, RCM_SW_19};
        return a;
    }

 private:
    /// Determine the blower speed to adopt for next cycle
    void calculateBlowerIncrement();

    /// Number of ticks when plateau is reached for the first time
    uint16_t m_plateauStartTime;

    /// True if plateau pressure has been reached (but not necessarily converged)
    bool m_plateauPressureReached;

    /**
     * PID to control the blower valve during some specific steps of the cycle
     *
     * @param targetPressure The pressure we want (in mmH2O)
     * @param currentPressure The pressure measured by the sensor (in mmH2O)
     * @param dt Time since the last computation (in microsecond)
     */
    int32_t PCinspiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt);

    /**
     * PID to control the patient valve during some specific steps of the cycle
     *
     * @param targetPressure The pressure we want (in mmH2O)
     * @param currentPressure The pressure measured by the sensor (in mmH2O)
     * @param dt Time since the last computation (in microsecond)
     */
    int32_t PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt);

    /// Current blower speed
    uint16_t m_blowerSpeed;

    /// Current blower speed increment (to apply at the beginning of the next cycle)
    int32_t m_blowerIncrement;

    /// Error of the last computation of the blower PID
    int32_t m_inspiratoryPidIntegral;

    /// Fast mode at start of expiration
    bool m_expiratoryPidFastMode;

    /// Fast mode at start of inspiration
    bool m_inspiratoryPidFastMode;

    /// Integral gain of the patient PID
    int32_t m_expiratoryPidIntegral;

    /// Last aperture of the blower valve
    int32_t m_inspiratoryValveLastAperture;

    /// Last aperture of the blower valve
    int32_t m_expiratoryValveLastAperture;

    /// Error of the last computation of the expiratory PID
    int32_t m_expiratoryPidLastError;
    /// Last errors in expiratory PID
    int32_t m_expiratoryPidLastErrors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    /// Last error index in inspiratory PID
    int32_t m_expiratoryPidLastErrorsIndex;

    /// Error of the last computation of the expiratory PID
    int32_t m_inspiratoryPidLastError;
    /// Last errors in inspiratory PID
    int32_t m_inspiratoryPidLastErrors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    /// Last error index in inspiratory PID
    int32_t m_inspiratoryPidLastErrorsIndex;
};

extern PC_CMV_Controller pcCmvController;
