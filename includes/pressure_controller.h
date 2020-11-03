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
#include "../includes/PC_CMV_Controller.h"
#include "../includes/PC_BIPAP_Controller.h"

/// Number of values to aggregate when computing plateau pressure
#define MAX_PRESSURE_SAMPLES 10u

/// Max value for increment / decrement of peak command
static const uint16_t MAX_PEAK_INCREMENT = 30u;

// CLASS ======================================================================

/// Controls breathing cycle
class PressureController {
 public:
    // cppcheck-suppress misra-c2012-2.7
    PressureController();

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

    /// Get the desired max peak
    inline uint16_t peakPressureCommand() const { return m_peakPressureCommand; }
    /// Get the desired plateau pressure
    inline uint16_t plateauPressureCommand() const { return m_plateauPressureCommand; }
    /// Get the desired PEEP
    inline uint16_t peepCommand() const { return m_peepNextCommand; }
    /// Get the desired number of cycles per minute
    inline uint16_t cyclesPerMinuteCommand() const { return m_cyclesPerMinuteCommand; }
    /// Get the value of the inspiratory trigger pressure command
    inline const uint16_t pressureTriggerOffsetCommand() const {
        return m_pressureTriggerOffsetCommand;
    }
    // Get the enabling state of trigger mode
    inline const bool triggerModeEnabledCommand() { return m_triggerModeEnabledCommand; }

    /// Get the desired max peak for next cycle
    inline uint16_t peakPressureNextCommand() const { return m_peakPressureNextCommand; }
    /// Get the desired plateau pressure for next cycle
    inline uint16_t plateauPressureNextCommand() const { return m_plateauPressureNextCommand; }
    /// Get the desired PEEP for next cycle
    inline uint16_t peepNextCommand() const { return m_peepNextCommand; }
    /// Get the desired number of cycles per minute for next cycle
    inline uint16_t cyclesPerMinuteNextCommand() const { return m_cyclesPerMinuteNextCommand; }
    /// Get the value of the inspiratory trigger pressure command for next cycle
    inline const uint16_t pressureTriggerOffsetNextCommand() const {
        return m_pressureTriggerOffsetNextCommand;
    }
    // Get the enabling state of trigger mode for next cycle
    inline const bool triggerModeEnabledNextCommand() { return m_triggerModeEnabledNextCommand; }

    /// Get the measured peak pressure
    inline int16_t peakPressureMeasure() const { return m_peakPressureMeasure; }
    /// Get the measured plateau pressure
    inline int16_t plateauPressureMeasure() const { return m_plateauPressureMeasure; }
    /// Get the measured PEEP
    inline int16_t peepMeasure() const { return m_peepMeasure; }
    /// Get the desired number of cycles per minute
    inline uint16_t cyclesPerMinuteMeasure() const { return m_cyclesPerMinuteMeasure; }
        /// Get the measured tidal Volume
    inline uint16_t tidalVolumeMeasure() const { return m_tidalVolumeMeasure; }

    /// Get the number of past cycles since the beginning
    inline uint32_t cycleNumber() const { return m_cycleNb; }

    /// Get the duration of a cycle in ticks
    inline uint16_t tickPerCycle() const { return m_ticksPerCycle; }

    /// Get the duration of an inhalation in ticks
    inline uint32_t tickPerInhalation() const { return m_tickPerInhalation; }

    /// Get the current measured pressure
    inline int16_t pressure() const { return m_pressure; }

    /// Get the delta of time since the last cycle
    inline int32_t dt() const { return m_dt; }

    /// Get the pressure command
    inline int32_t pressureCommand() const { return m_pressureCommand; }

    /// Get the current cycle phase
    inline CyclePhases phase() const { return m_phase; }

    /// Get the current cycle subphase
    inline CycleSubPhases subPhase() const { return m_subPhase; }

    /// Get the state of the inspiratory trigger
    inline const bool triggered() const { return m_triggered; }

    /// Reset the trigger to false
    inline const void setTrigger(bool triggerValue) { m_triggered = triggerValue; }

    // Get if the peep has been detected during this cycle
    inline const bool isPeepDetected() { return m_isPeepDetected; }

    /**
     * Input the real duration since the last pressure controller computation
     *
     * @param p_dt Duration in microsecond
     */
    void updateDt(int32_t p_dt);

    void reachSafetyPosition();

    void stop();

    void sendSnapshot();

 private:
    /**
     * Update the cycle phase
     *
     * @param p_tick  Duration from the begining of the cycle in hundredth of second
     */
    void updatePhase(uint16_t p_tick);

    /// Update peak pressure and blower ramp up
    // cppcheck-suppress unusedPrivateFunction
    void updatepeakPressureMeasure();

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

    /// Give the computed commands to actuators
    void executeCommands();

    /// At the end of a respiratory cycle, check if some alarms are triggered
    void checkCycleAlarm();

    void calculateBlowerIncrement();

 private:
    PC_BIPAP_Controller *ventilationController;
    /// Actual desired number of cycles per minute
    uint16_t m_cyclesPerMinuteCommand;
    /// Number of cycles per minute desired by the operator for next cycle
    uint16_t m_cyclesPerMinuteNextCommand;
    /// Measured number of cycles per minute
    uint32_t m_cyclesPerMinuteMeasure;
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
    /// Sum for calulating plateau value
    uint64_t m_PlateauMeasureSum;
    /// Count for calulating plateau value
    uint16_t m_PlateauMeasureCount;
    /// Duration of the plateau. Use this setting in trigger mode
    uint32_t m_plateauDurationMs;

    /// Actual desired PEEP
    uint16_t m_peepCommand;
    /// Desired PEEP for next cycle
    uint16_t m_peepNextCommand;
    /// Measured value of the PEEP
    uint16_t m_peepMeasure;
    /// Is PEEP pressure detected in the cycle
    bool m_isPeepDetected;

    // Actual Pressure trigger offset
    uint16_t m_pressureTriggerOffsetCommand;
    // Desired Pressure trigger offset for next cycle
    uint16_t m_pressureTriggerOffsetNextCommand;
    /// Is inspiratory triggered or not
    bool m_triggered;

    /// Actual state of enabling of trigger mode
    bool m_triggerModeEnabledCommand;
    /// Desired state of enabling of trigger mode for next cycle
    bool m_triggerModeEnabledNextCommand;

    // E term of the I:E ratio. I = 10, and E is in [10;60]
    /// Actual expiratory term
    uint16_t m_expiratoryTermCommand;
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

    /// Number of passed cycles since beginning
    uint32_t m_cycleNb;

    /// Time since the last computation (in microsecond)
    int32_t m_dt;

    /// Requested pressure at a given point in time
    int32_t m_pressureCommand;

    /// Last pressure values
    uint16_t m_lastPressureValues[MAX_PRESSURE_SAMPLES];
    uint16_t m_lastPressureValuesIndex;

    /// Sum of the current cycle's pressures
    uint32_t m_sumOfPressures;  // TODO : be carefull for this parameter overflow !!
    /// Number of the current cycle's pressures
    uint16_t m_numberOfPressures;

    // Tick index, given by the main loop
    uint16_t m_tick;
};

extern PressureController pController;