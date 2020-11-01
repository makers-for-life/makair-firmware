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
     * @param p_inspiratory_valve        Pressure Valve between blower and patient
     * @param p_expiratory_valve       Pressure Valve between patient and atmosphere
     * @param p_alarmController     Alarm controller
     * @param p_blower              Blower
     */
    // cppcheck-suppress misra-c2012-2.7
    PressureController(const PressureValve& p_inspiratory_valve,
                       const PressureValve& p_expiratory_valve,
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
     * Input a flow reading
     * @param p_currentInspiratoryFlow  Measured inspiratory flow
     */
    void updateInspiratoryFlow(int16_t p_currentInspiratoryFlow);

    /**
     * Input a flow reading
     * @param p_currentInspiratoryFlow  Measured inspiratory flow
     */
    void updateExpiratoryFlow(int16_t p_currentExpiratoryFlow);

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
     * DEPRECATED
     */
    void onPeakPressureDecrease();

    /**
     * Increase the desired peak pressure
     *
     * DEPRECATED
     */
    void onPeakPressureIncrease();

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
    inline uint16_t cyclesPerMinuteCommand() const { return m_cyclesPerMinuteNextCommand; }

    /// Get the desired max peak
    inline uint16_t PeakPressureCommand() const { return m_peakPressureNextCommand; }

    /// Get the desired PEEP
    inline uint16_t PeepCommand() const { return m_peepNextCommand; }

    /// Get the desired maximal plateau pressure
    inline uint16_t PlateauPressureCommand() const { return m_plateauPressureNextCommand; }

    /// Get the desired number of cycles per minute
    inline uint16_t cyclesPerMinute() const { return m_cyclesPerMinuteNextCommand; }

    /// Get the number of past cycles since the beginning
    inline uint32_t cycleNumber() const { return m_cycleNb; }

    /// Get the duration of a cycle in ticks
    inline uint16_t tickPerCycle() const { return m_ticksPerCycle; }

    /// Get the duration of an inhalation in ticks
    inline uint32_t tickPerInhalation() const { return m_tickPerInhalation; }

    /// Get the current measured pressure
    inline int16_t pressure() const { return m_pressure; }

    /// Get the measured peak pressure
    inline int16_t peakPressure() const { return m_peakPressureMeasure; }

    /// Get the measured plateau pressure
    inline int16_t plateauPressure() const { return m_plateauPressureMeasure; }

    /// Get the measured PEEP
    inline int16_t peep() const { return m_peepMeasure; }

    /// Get the measured number of cycles per minute
    inline uint32_t measuredCyclesPerMinute() const { return m_CyclesPerMinuteMeasure; }

    /// Get the current cycle phase
    inline CyclePhases phase() const { return m_phase; }

    /// Get the current cycle subphase
    inline CycleSubPhases subPhase() const { return m_subPhase; }

    /// Get the blower's Pressure Valve instance
    inline const PressureValve& inspiratory_valve() const { return m_inspiratoryValve; }

    /// Get the patient's Pressure Valve instance
    inline const PressureValve& expiratory_valve() const { return m_expiratoryValve; }

    /// Get the state of the inspiratory trigger
    inline const bool triggered() const { return m_triggered; }

    /// Get the value of the inspiratory trigger pressure command
    inline const uint16_t pressureTrigger() const { return m_pressureTriggerOffsetNextCommand; }

    /// Reset the trigger to false
    inline const void reset_trigger() { m_triggered = false; }

    /**
     * Input the real duration since the last pressure controller computation
     *
     * @param p_dt Duration in microsecond
     */
    void updateDt(int32_t p_dt);

    void reachSafetyPosition();

    void stop();

    void sendSnapshot(bool isRunning);

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
    void inhale(uint16_t p_tick);

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
    int32_t PCinspiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt);

    /**
     * PID to controller the patient valve during some specific steps of the cycle
     *
     * @param targetPressure The pressure we want (in mmH2O)
     * @param currentPressure The pressure measured by the sensor (in mmH2O)
     * @param dt Time since the last computation (in microsecond)
     */
    int32_t PCexpiratoryPID(int32_t targetPressure, int32_t currentPressure, int32_t dt);

    /// At the end of a respiratory cycle, check if some alarms are triggered
    void checkCycleAlarm();

    void calculateBlowerIncrement();

 private:
    /// Actual desired number of cycles per minute
    uint16_t m_cyclesPerMinuteCommand;
    /// Number of cycles per minute desired by the operator for next cycle
    uint16_t m_cyclesPerMinuteNextCommand;
    /// Measured number of cycles per minute
    uint32_t m_CyclesPerMinuteMeasure;
    /// Used to compute cpm with a moving mean on some cycles.
    uint32_t m_lastBreathPeriodsMs[NUMBER_OF_BREATH_PERIOD];
    /// Index for the m_lastBreathPeriodsMs array
    uint32_t m_lastBreathPeriodsMsIndex;
    /// Date of the last end of a respiration
    uint32_t m_lastEndOfRespirationDateMs;

    /// Actual desired peak pressure
    uint16_t m_peakPressureCommand;
    /// Measure the value of peak pressure
    uint16_t m_peakPressureMeasure;
    /// Peak pressure desired by the operator for next cycle
    uint16_t m_peakPressureNextCommand;

    /// Actual desired plateau pressure
    uint16_t m_plateauPressureCommand;
    /// Plateau pressure desired by the operator for next cycle
    uint16_t m_plateauPressureNextCommand;
    /// Measured value of the plateau pressure
    uint16_t m_plateauPressureMeasure;
    /// This value is sometimes MAXINT and we dont want to display MAXINT
    uint16_t m_plateauPressureToDisplay;
    /// True if we started to compute plateau measure, false otherwise
    bool m_startPlateauComputation;
    /// True if plateau is computed, false otherwise
    bool m_plateauComputed;
    /// Sum for calulating square plateau value
    uint64_t m_squarePlateauSum;
    /// Count for calulating square plateau value
    uint16_t m_squarePlateauCount;
    /// Number of hundredth of second from the begining of the cycle till the plateau phase
    uint16_t m_plateauStartTime;
    /// Duration of the plateau. Use this setting in trigger mode
    uint32_t m_plateauDurationMs;
    /// true if plateau pressure has been reached (but not necessarily converged.)
    bool m_plateauPressureReached;

    /// Actual desired PEEP
    uint16_t m_peepCommand;
    /// Desired PEEP for next cycle
    uint16_t m_peepNextCommand;
    /// Measured value of the PEEP
    uint16_t m_peepMeasure;
    /// Is PEEP pressure detected in the cycle
    bool m_isPeepDetected;

    // Actual Pressure trigger offset
    uint16_t m_pressureTriggerOffset;
    // Desired Pressure trigger offset for next cycle
    uint16_t m_pressureTriggerOffsetNextCommand;
    /// Is inspiratory triggered or not
    bool m_triggered;

    /// Actual state of enabling of trigger mode
    bool m_triggerModeEnabled;
    /// Desired state of enabling of trigger mode for next cycle
    bool m_triggerModeEnabledNextCommand;

    // E term of the I:E ratio. I = 10, and E is in [10;60]
    /// Actual expiratory term
    uint16_t m_expiratoryTerm;
    /// Desired expiratory term for next cycle
    uint16_t m_expiratoryTermNextCommand;

    /// Measured value of the tidal volume (volume of air pushed in patient lungs in last
    /// inspiration)
    uint16_t m_tidalVolumeMeasure;

    /// Number of hundredth of second per cycle
    uint16_t m_ticksPerCycle;

    /// Number of hundredth of second per inhalation
    uint32_t m_tickPerInhalation;

    /// Measured pressure
    uint16_t m_pressure;

    /// Measured expiratory flow
    int16_t m_inspiratoryFlow;

    /// Measured inspiratory flow
    int16_t m_expiratoryFlow;

    /// Inhalation last Pressre
    uint16_t m_inhalationLastPressure;

    /// Blower valve angle at peak
    uint32_t m_inspiratoryValveAngle;

    /// Current respiratory cycle phase
    CyclePhases m_phase;

    /// Current respiratory cycle phase
    CycleSubPhases m_subPhase;

    /// Inspiratory valve
    PressureValve m_inspiratoryValve;

    /// Expiratory valve
    PressureValve m_expiratoryValve;

    /// Blower
    Blower* m_blower;

    /// Blower increment
    int32_t m_blower_increment;

    /// Number of passed cycles since beginning
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
    int32_t PC_inspiratory_PID_integral;

    /**
     * Error of the last computation of the blower PID
     *
     * @note This must be persisted between computation in order to compute derivative gain
     */
    int32_t PC_inspiratory_PID_LastError;

    /**
     * Fast mode at start of expiration
     *
     * @note This must be persisted between computations
     */
    bool PC_expiratory_PID_fast_mode;

    /**
     * Fast mode at start of inspiration
     *
     * @note This must be persisted between computations
     */
    bool PC_inspiratory_PID_fast_mode;

    /**
     * Integral gain of the patient PID
     *
     * @note This must be persisted between computations
     */
    int32_t PC_expiratory_PID_integral;

    /**
     * Last aperture of the blower valve
     *
     * @note This must be persisted between computations
     */
    uint32_t inspiratoryValveLastAperture;

    /**
     * Last aperture of the blower valve
     *
     * @note This must be persisted between computations
     */
    uint32_t expiratoryValveLastAperture;

    /**
     * Error of the last computation of the patient PID
     *
     * @note This must be persisted between computation in order to compute derivative gain
     */
    int32_t PC_expiratory_PID_LastError;

    /// Alarm controller
    AlarmController* m_alarmController;

    /// Last pressure values
    uint16_t m_lastPressureValues[MAX_PRESSURE_SAMPLES];
    uint16_t m_lastPressureValuesIndex;

    /// Last error in inspiratory PID
    int32_t PC_inspiratory_PID_last_errors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    int32_t PC_inspiratory_PID_last_errorsIndex;

    /// Last error in expiratory PID
    int32_t PC_expiratory_PID_last_errors[PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN];
    int32_t PC_expiratory_PID_last_errorsIndex;

    /// Sum of the current cycle's pressures
    uint32_t m_sumOfPressures;  // TODO : be carefull for this parameter overflow !!
    /// Number of the current cycle's pressures
    uint16_t m_numberOfPressures;

    // Tick index, given by the main loop
    uint16_t m_tick;
};

// INITIALISATION =============================================================

/// Instance of the pressure controller
extern PressureController pController;
