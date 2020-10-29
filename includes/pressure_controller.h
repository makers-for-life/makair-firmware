/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure_controller.h
 * @brief Core logic to control the breathing cycle
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Internal
#include "../includes/alarm_controller.h"
#include "../includes/blower.h"
#include "../includes/cycle.h"
#include "../includes/parameters.h"
#include "../includes/pressure_valve.h"

/// Number of values to aggregate when computing plateau pressure
#define MAX_PRESSURE_SAMPLES 10u

/// Max value for increment / decrement of peak command
static const uint16_t MAX_PEAK_INCREMENT = 30u;

// CLASS ======================================================================

/// Controls breathing cycle
class PressureController {
 public:
    /// Default constructor
    PressureController();

    /**
     * Parameterized constructor
     *
     * @param p_cyclesPerMinute     Initial number of breathing cycles per minute
     * @param p_minPeepCommand      Initial minimum PEEP pressure (in mmH2O)
     * @param p_maxPlateauPressure  Initial maximum plateau pressure (in mmH2O)
     * @param p_maxPeakPressure     Initial maximum peak pressure (in mmH2O)
     * @param p_blower_valve        Pressure Valve between blower and patient
     * @param p_patient_valve       Pressure Valve between patient and atmosphere
     * @param p_alarmController     Alarm controller
     * @param p_blower              Blower
     */
    // cppcheck-suppress misra-c2012-2.7
    PressureController(int16_t p_cyclesPerMinute,
                       int16_t p_minPeepCommand,
                       int16_t p_maxPlateauPressure,
                       int16_t p_maxPeakPressure,
                       const PressureValve& p_blower_valve,
                       const PressureValve& p_patient_valve,
                       AlarmController* p_alarmController,
                       Blower* p_blower);

    /// Initialize actuators
    void setup();

    /// Begin a respiratory cycle
    void initRespiratoryCycle();

    /// End a respiratory cycle
    void endRespiratoryCycle();

    /**
     * Input a pressure reading
     * @param p_currentPressure  Measured pressure
     */
    void updatePressure(int16_t p_currentPressure);

    /**
     * Perform the pressure control
     *
     * @param p_tick  Duration in hundredth of second from the begining of the cycle
     */
    void compute(uint16_t p_tick);

    /// Decrease the desired number of cycles per minute
    void onCycleDecrease();

    /// Increase the desired number of cycles per minute
    void onCycleIncrease();

    /**
     * Set the desired number of cycles per minute
     *
     * @param cpm Desired number of cycle per minute
     */
    void onCycleSet(uint16_t cpm);

    /// Decrease the minimal PEEP desired
    void onPeepPressureDecrease();

    /// Increase the minimal PEEP desired
    void onPeepPressureIncrease();

    /**
     * Set the desired PEEP
     *
     * @param peep Desired PEEP in mmH2O
     */
    void onPeepSet(uint16_t peep);

    /// Decrease the desired plateau pressure
    void onPlateauPressureDecrease();

    /// Increase the desired plateau pressure
    void onPlateauPressureIncrease();

    /**
     * Set the desired plateau pressure
     *
     * @param plateauPressure Desired plateau pressure in mmH2O
     */
    void onPlateauPressureSet(uint16_t plateauPressure);

    /**
     * Decrease the desired peak pressure
     *
     * @param p_decrement Positive value of decrement
     */
    void onPeakPressureDecrease(uint8_t p_decrement);

    /**
     * Increase the desired peak pressure
     *
     * @param p_increment Positive value of increment
     */
    void onPeakPressureIncrease(uint8_t p_increment);

    /**
     * Set the desired peak pressure
     *
     * @param peakPressure Desired peak pressure in mmH2O
     */
    void onPeakPressureSet(uint16_t peakPressure);

    /**
     * Set the desired Expiratory term
     *
     * @param ExpiratoryTerm : Expiration term in the "Inspiration/Expiration" ratio given that
     * Inspiration = 10
     */
    void onExpiratoryTermSet(uint16_t ExpiratoryTerm);

    /**
     * 0: trigger mode disable, 1: trigger mode enable
     *
     * @param TriggerEnabled
     */
    void onTriggerEnabledSet(uint16_t TriggerEnabled);

    /**
     * Set the desired Trigger Offset
     *
     * @param TriggerOffset Desired trigger offset in mmH2O
     */
    void onTriggerOffsetSet(uint16_t TriggerOffset);

    /// Get the desired number of cycles per minute
    inline uint16_t cyclesPerMinuteCommand() const { return m_cyclesPerMinuteCommand; }

    /// Get the desired max peak
    inline uint16_t maxPeakPressureCommand() const { return m_maxPeakPressureCommand; }

    /// Get the desired minimal PEEP
    inline uint16_t minPeepCommand() const { return m_minPeepCommand; }

    /// Get the desired maximal plateau pressure
    inline uint16_t maxPlateauPressureCommand() const { return m_maxPlateauPressureCommand; }

    /// Get the number of cycles per minute
    inline uint16_t cyclesPerMinute() const { return m_cyclesPerMinute; }

    /// Get the number of past cycles since the beginning
    inline uint32_t cycleNumber() const { return m_cycleNb; }

    /// Get the duration of a cycle in ticks
    inline uint16_t tickPerCycle() const { return m_ticksPerCycle; }

    /// Get the duration of an inhalation in ticks
    inline uint32_t tickPerInhalation() const { return m_tickPerInhalation; }

    /// Get the current measured pressure
    inline int16_t pressure() const { return m_pressure; }

    /// Get the peak pressure
    inline int16_t peakPressure() const { return m_peakPressure; }

    /// Get the plateau pressure
    inline int16_t plateauPressure() const { return m_plateauPressure; }

    /// Get the PEEP
    inline int16_t peep() const { return m_peep; }

    /// Get the measured number of cycles per minute
    inline uint32_t measuredCyclesPerMinute() const { return m_measuredCyclesPerMinute; }

    /// Get the current cycle phase
    inline CyclePhases phase() const { return m_phase; }

    /// Get the current cycle subphase
    inline CycleSubPhases subPhase() const { return m_subPhase; }

    /// Get the blower's Pressure Valve instance
    inline const PressureValve& blower_valve() const { return m_blower_valve; }

    /// Get the patient's Pressure Valve instance
    inline const PressureValve& patient_valve() const { return m_patient_valve; }

    /// Get the state of the inspiratory trigger
    inline const bool triggered() const { return m_triggered; }

    /// Reset the trigger to false
    inline const void reset_trigger() { m_triggered = false; }

    /// Get the value of the inspiratory trigger pressure
    inline const uint16_t pressureTrigger() const { return m_pressureTrigger; }

    /// Get the enable state of the trigger mode
    inline const uint16_t isTriggerModeEnabled() const { return m_triggerModeEnabled; }

    /**
     * Input the real duration since the last pressure controller computation
     *
     * @param p_dt Duration in microsecond
     */
    void updateDt(int32_t p_dt);

    void reachSafetyPosition();

 private:
    /**
     * Update the cycle phase
     *
     * @param p_tick  Duration from the begining of the cycle in hundredth of second
     */
    void updatePhase(uint16_t p_tick);

    /// Update peak pressure and blower ramp up
    // cppcheck-suppress unusedPrivateFunction
    void updatePeakPressure();

    /// Perform the pressure control and compute the transistors commands during the inhalation
    /// phase
    void inhale();

    /// Perform the pressure control and compute the transistors commands during the plateau phase
    void plateau();

    /// Perform the pressure control and compute the transistors commands during the exhalation
    /// phase
    void exhale();

    /**
     * Compute various cycle durations given the desired number of cycles per minute
     *
     * - duration of a cycle in hundredth of second
     * - duration of the inhalation phase in hundredth of second
     *
     *  N.B.: Inhalation lasts 1/3 of a cycle while exhalation lasts 2/3 of a cycle
     */
    void computeTickParameters();

    /**
     * Compute plateau pressure
     *
     * @param p_tick  Duration from the begining of the cycle in hundredth of second
     */
    // cppcheck-suppress unusedPrivateFunction
    void computePlateau(uint16_t p_tick);

    /// Give the computed commands to actuators
    void executeCommands();

    /**
     * Make a transition toward another subphase
     *
     * @param p_subPhase  Next cycle step
     * @param p_tick  Duration from the begining of the cycle in hundredth of second
     */
    void setSubPhase(CycleSubPhases p_subPhase, uint16_t p_tick);

    /**
     * PID to controller the blower valve during some specific steps of the cycle
     *
     * @param targetPressure The pressure we want (in mmH2O)
     * @param currentPressure The pressure measured by the sensor (in mmH2O)
     * @param dt Time since the last computation (in microsecond)
     */
    int32_t pidBlower(int32_t targetPressure, int32_t currentPressure, int32_t dt);

    /**
     * PID to controller the patient valve during some specific steps of the cycle
     *
     * @param targetPressure The pressure we want (in mmH2O)
     * @param currentPressure The pressure measured by the sensor (in mmH2O)
     * @param dt Time since the last computation (in microsecond)
     */
    int32_t pidPatient(int32_t targetPressure, int32_t currentPressure, int32_t dt);

    /// At the end of a respiratory cycle, check if some alarms are triggered
    void checkCycleAlarm();

#if VALVE_TYPE == VT_FAULHABER
    /// Update only blower speed
    void updateOnlyBlower();
#endif

 private:
    /// Number of cycles per minute desired by the operator
    uint16_t m_cyclesPerMinuteCommand;

    /// Maximal peak pressure desired by the operator
    uint16_t m_maxPeakPressureCommand;

    /// Maximal plateau pressure desired by the operator
    uint16_t m_maxPlateauPressureCommand;

    /// Minimal PEEP desired by the operator
    uint16_t m_minPeepCommand;

    /// Number of cycles per minute
    uint16_t m_cyclesPerMinute;

    /// Number of hundredth of second per cycle
    uint16_t m_ticksPerCycle;

    /// Number of hundredth of second per inhalation
    uint32_t m_tickPerInhalation;

    /// Maximal peak pressure
    uint16_t m_maxPeakPressure;

    /// Maximal plateau pressure
    uint16_t m_maxPlateauPressure;

    /// Minimal PEEP
    uint16_t m_minPeep;

    /// Measured pressure
    uint16_t m_pressure;

    /// Inhalation last Pressre
    uint16_t m_inhalationLastPressure;

    /// Sum for calulating square plateau value
    uint64_t m_squarePlateauSum;

    /// Count for calulating square plateau value
    uint16_t m_squarePlateauCount;

    /// Peak pressure
    uint16_t m_peakPressure;

    /// Plateau pressure
    uint16_t m_plateauPressure;

    /// Positive End Expiratory Pressure
    uint16_t m_peep;

    /// Blower valve angle at peak
    uint32_t m_peakBlowerValveAngle;

    /// Current respiratory cycle phase
    CyclePhases m_phase;

    /// Current respiratory cycle phase
    CycleSubPhases m_subPhase;

    /// Blower's transistor
    PressureValve m_blower_valve;

    /// Patient's transistor
    PressureValve m_patient_valve;

    /// Blower
    Blower* m_blower;

    /// Blower increment
    int32_t m_blower_increment;

    /// Number of passed cycles
    uint32_t m_cycleNb;

    /// Time since the last computation (in microsecond)
    int32_t m_dt;

    /// Requested pressure at a given point in time
    int32_t m_pressureCommand;

    /**
     * Integral gain of the blower PID
     *
     * @note This must be persisted between computations
     */
    int32_t blowerIntegral;

    /**
     * Error of the last computation of the blower PID
     *
     * @note This must be persisted between computation in order to compute derivative gain
     */
    int32_t blowerLastError;

    /**
     * Fast mode at start of expiration
     *
     * @note This must be persisted between computations
     */
    bool patientPIDFastMode;

    /**
     * Fast mode at start of inspiration
     *
     * @note This must be persisted between computations
     */
    bool blowerPIDFastMode;

    /**
     * Integral gain of the patient PID
     *
     * @note This must be persisted between computations
     */
    int32_t patientIntegral;

    /**
     * Last aperture of the blower valve
     *
     * @note This must be persisted between computations
     */
    uint32_t lastBlowerAperture;

    /**
     * Last aperture of the blower valve
     *
     * @note This must be persisted between computations
     */
    uint32_t lastPatientAperture;

    /**
     * Error of the last computation of the patient PID
     *
     * @note This must be persisted between computation in order to compute derivative gain
     */
    int32_t patientLastError;

    /// Alarm controller
    AlarmController* m_alarmController;

    /// True if we started to compute plateau measure, false otherwise
    bool m_startPlateauComputation;

    /// True if plateau is computed, false otherwise
    bool m_plateauComputed;

    /// Last pressure values
    uint16_t m_lastPressureValues[MAX_PRESSURE_SAMPLES];

    /// Last error in blower PID
    int32_t m_lastBlowerPIDError[NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN];
    int32_t m_lastBlowerPIDErrorIndex;

    /// Last error in Patient PID
    int32_t m_lastPatientPIDError[NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN];
    int32_t m_lastPatientPIDErrorIndex;

    /// Index of array for last pressure storage
    uint16_t m_lastPressureValuesIndex;

    /// Sum of the current cycle's pressures
    uint32_t m_sumOfPressures;

    /// Number of the current cycle's pressures
    uint16_t m_numberOfPressures;

    /// Number of hundredth of second from the begining of the cycle till the plateau phase
    uint16_t m_plateauStartTime;

    // Tick index, given by the main loop
    uint16_t m_tick;

    // Pressure trigger value
    uint16_t m_pressureTrigger;

    /// Is trigger mode enabled
    bool m_triggerModeEnabled;

    /// Is inspiratory triggered or not
    bool m_triggered;

    /// Is PEEP pressure detected in the cycle
    bool m_isPeepDetected;

    /// Duration of the plateau. Use this setting in trigger mode
    uint32_t m_plateauDurationMs;

    /**
     * The last cycle periods in ms
     *
     * @note Used to compute m_measuredCyclesPerMinute
     */
    uint32_t m_lastBreathPeriodsMs[NUMBER_OF_BREATH_PERIOD];

    /// Index for the m_lastBreathPeriodsMs array
    uint32_t m_lastBreathPeriodsMsIndex;

    /// Measured number of cycles per minute
    uint32_t m_measuredCyclesPerMinute;

    /// Date of the last end of a respiration
    uint32_t m_lastEndOfRespirationDateMs;

    uint16_t m_ExpiratoryTerm;
};

// INITIALISATION =============================================================

/// Instance of the pressure controller
extern PressureController pController;
