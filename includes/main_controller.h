/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file main_controller.h
 * @brief Core logic to control the breathing cycle
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Internal

#if !SIMULATION
#include "../includes/alarm_controller.h"
#include "../includes/battery.h"
#include "../includes/telemetry.h"
#endif

#include "../includes/blower.h"
#include "../includes/config.h"
#include "../includes/cycle.h"
#include "../includes/debug.h"
#include "../includes/parameters.h"
#include "../includes/pc_bipap_controller.h"
#include "../includes/pc_cmv_controller.h"
#include "../includes/pressure_valve.h"

/// Number of values to aggregate when computing plateau pressure
#define MAX_PRESSURE_SAMPLES 10

// CLASS ======================================================================

/// Controls breathing cycle
// cppcheck-suppress misra-c2012-5.2 ; false positive
class MainController {
 public:
    /// Default constructor
    MainController();

    /// Initialize actuators
    void setup();

    /// Begin a respiratory cycle
    void initRespiratoryCycle();

    /// End a respiratory cycle
    void endRespiratoryCycle(uint32_t p_currentMillis);

    /**
     * Input a tick number
     *
     * @param p_tick Tick number
     */
    void updateTick(uint32_t p_tick);

    /**
     * Input a pressure reading
     *
     * @param p_currentPressure  Measured pressure
     */
    void updatePressure(int16_t p_currentPressure);

    /**
     * Input an inspiratory flow reading
     *
     * @param p_currentInspiratoryFlow  Measured inspiratory flow
     */
    void updateInspiratoryFlow(int32_t p_currentInspiratoryFlow);

    /**
     * Input an expiratory flow reading
     *
     * @param p_currentExpiratoryFlow  Measured expiratory flow
     */
    void updateExpiratoryFlow(int32_t p_currentExpiratoryFlow);

    /// Calculate expiratory flow from pressure and valve angle
    void updateFakeExpiratoryFlow();

    /**
     * Perform the pressure control
     *
     * @param p_tick  Duration in hundredth of second from the begining of the cycle
     */
    void compute();

    /// Decrease the desired number of cycles per minute
    void onCycleDecrease();

    /// Increase the desired number of cycles per minute
    void onCycleIncrease();

    /**
     * Set the desired number of cycles per minute
     *
     * @param p_cpm Desired number of cycles per minute
     */
    void onCycleSet(uint16_t p_cpm);

    /// Decrease the minimal PEEP desired
    void onPeepPressureDecrease();

    /// Increase the minimal PEEP desired
    void onPeepPressureIncrease();

    /**
     * Set the desired PEEP
     *
     * @param p_peep Desired PEEP in mmH2O
     */
    void onPeepSet(int16_t p_peep);

    /// Decrease the desired plateau pressure
    void onPlateauPressureDecrease();

    /// Increase the desired plateau pressure
    void onPlateauPressureIncrease();

    /**
     * Set the desired plateau pressure
     *
     * @param p_plateauPressure Desired plateau pressure in mmH2O
     */
    void onPlateauPressureSet(int16_t p_plateauPressure);

    /**
     * Decrease the desired peak pressure
     *
     * @deprecated Peak pressure is now based on plateau pressure
     */
    void onPeakPressureDecrease();

    /**
     * Increase the desired peak pressure
     *
     * @deprecated Peak pressure is now based on plateau pressure
     */
    void onPeakPressureIncrease();

    /**
     * Set the desired expiratory term
     *
     * @param p_expiratoryTerm  Expiration term in the "Inspiration/Expiration" ratio given that
     * Inspiration = 10
     */
    void onExpiratoryTermSet(uint16_t p_expiratoryTerm);

    /**
     * Enable or disable expiratory trigger mode
     *
     * @param p_triggerEnabled  0: disable trigger mode, 1: enable trigger mode
     */
    void onTriggerModeEnabledSet(uint16_t p_triggerEnabled);

    /**
     * Set the desired offset for expiratory trigger
     *
     * @param p_triggerOffset Desired trigger offset in mmH2O
     */
    void onTriggerOffsetSet(uint16_t p_triggerOffset);

    /// Get the desired max peak
    inline int16_t peakPressureCommand() const { return m_peakPressureCommand; }
    /// Get the desired plateau pressure
    inline int16_t plateauPressureCommand() const { return m_plateauPressureCommand; }
    /// Get the desired PEEP
    inline int16_t peepCommand() const { return m_peepNextCommand; }
    /// Get the desired number of cycles per minute
    inline uint16_t cyclesPerMinuteCommand() const { return m_cyclesPerMinuteCommand; }
    /// Get the value of the inspiratory trigger pressure command
    inline const int16_t pressureTriggerOffsetCommand() const {
        return m_pressureTriggerOffsetCommand;
    }
    /// Get the enabling state of trigger mode
    inline const bool triggerModeEnabledCommand() { return m_triggerModeEnabledCommand; }

    /// Get the desired max peak for the next cycle
    inline int16_t peakPressureNextCommand() const { return m_peakPressureNextCommand; }
    /// Get the desired plateau pressure for the next cycle
    inline int16_t plateauPressureNextCommand() const { return m_plateauPressureNextCommand; }
    /// Get the desired PEEP for the next cycle
    inline int16_t peepNextCommand() const { return m_peepNextCommand; }
    /// Get the desired number of cycles per minute for the next cycle
    inline uint16_t cyclesPerMinuteNextCommand() const { return m_cyclesPerMinuteNextCommand; }
    /// Get the value of the inspiratory trigger pressure command for the next cycle
    inline const int16_t pressureTriggerOffsetNextCommand() const {
        return m_pressureTriggerOffsetNextCommand;
    }
    /// Get the enabling state of trigger mode for the next cycle
    inline const bool triggerModeEnabledNextCommand() { return m_triggerModeEnabledNextCommand; }

    /// Get the measured peak pressure
    inline int16_t peakPressureMeasure() const { return m_peakPressureMeasure; }
    /// Get the measured rebounce peak pressure
    inline int16_t rebouncePeakPressureMeasure() const { return m_rebouncePeakPressureMeasure; }
    /// Get the measured plateau pressure
    inline int16_t plateauPressureMeasure() const { return m_plateauPressureMeasure; }
    /// Get the measured PEEP
    inline int16_t peepMeasure() const { return m_peepMeasure; }
    /// Get the desired number of cycles per minute
    inline uint16_t cyclesPerMinuteMeasure() const { return m_cyclesPerMinuteMeasure; }
    /// Get the measured Tidal Volume
    inline uint16_t tidalVolumeMeasure() const { return m_tidalVolumeMeasure; }

    /// Get the number of past cycles since the beginning
    inline uint32_t cycleNumber() const { return m_cycleNb; }

    /// Get the duration of a cycle in ticks
    inline uint16_t ticksPerCycle() const { return m_ticksPerCycle; }

    /// Get the duration of an inhalation in ticks
    inline uint32_t ticksPerInhalation() const { return m_ticksPerInhalation; }

    /// Get the duration of an inhalation in ticks
    inline void ticksPerInhalationSet(uint32_t p_ticksPerInhalation) {
        m_ticksPerInhalation = p_ticksPerInhalation;
    }

    /// Get the current measured pressure
    inline int16_t pressure() const { return m_pressure; }

    /// Get the current inspiratoryFlow
    inline int32_t inspiratoryFlow() const { return m_inspiratoryFlow; }

    /// Get the current expiratoryFlow
    inline int32_t expiratoryFlow() const { return m_expiratoryFlow; }

    /// Get the delta of time since the last cycle (in ms)
    inline int32_t dt() const { return m_dt; }

    /// Get the tick number of the current cycle
    inline uint32_t tick() const { return m_tick; }

    /// Get the pressure command
    inline int32_t pressureCommand() const { return m_pressureCommand; }

    /// Get the current cycle phase
    inline CyclePhases phase() const { return m_phase; }

    /// Get the state of the inspiratory trigger
    inline const bool triggered() const { return m_triggered; }

    /// Reset the trigger to false
    inline const void setTrigger(bool triggerValue) { m_triggered = triggerValue; }

    /// Get if the PEEP has been detected during this cycle
    inline const bool isPeepDetected() { return m_isPeepDetected; }

    /**
     * Input the real duration since the last pressure controller computation
     *
     * @param p_dt Duration in microsecond
     */
    void updateDt(int32_t p_dt);

    /**
     * Input the current delivered volume in inspiratory branch since beginning of the respiratory
     * cycle
     *
     * @param p_currentDeliveredVolume Delivered Volume in mL/s
     */
    void updateCurrentDeliveredVolume(int32_t p_currentDeliveredVolume);

    /// Put actuators in safety position
    void reachSafetyPosition();

    /// Stop the breathing
    void stop(uint32_t p_currentMillis);

    /// Send a "stopped" telemetry message
    void sendStopMessageToUi();

    /// Send a "machine state snapshot" telemetry message
    void sendMachineState();

 private:
    /**
     * Update the cycle phase
     *
     * @param p_tick  Duration from the begining of the cycle in hundredth of second
     */
    void updatePhase();

    /// Perform the pressure control and compute the actuators commands during the inhalation
    /// phase
    void inhale();

    /// Perform the pressure control and compute the actuators commands during the exhalation
    /// phase
    void exhale();

    /**
     * Compute various cycle durations given the desired number of cycles per minute
     *
     * - duration of a cycle in hundredth of second
     * - duration of the inhalation phase in hundredth of second
     *
     *  @note Inhalation lasts 1/3 of a cycle while exhalation lasts 2/3 of a cycle
     */
    void computeTickParameters();

    /// Send the computed commands to actuators
    void executeCommands();

    /// At the end of a respiratory cycle, check if some alarms are triggered
    void checkCycleAlarm();

    void calculateBlowerIncrement();

    /// Print debug values, used to tweak PID, and triggers
    void printDebugValues();

 private:
    int32_t m_expiratoryVolume;
    /// Actual tick number (given by the main state machine)
    uint32_t m_tick;

    /// Actual desired number of cycles per minute
    uint16_t m_cyclesPerMinuteCommand;
    /// Number of cycles per minute desired by the operator for the next cycle
    uint16_t m_cyclesPerMinuteNextCommand;
    /// Measured number of cycles per minute
    uint32_t m_cyclesPerMinuteMeasure;
    /**
     * Period durations of last beathings
     *
     * @note Used to compute cpm with a moving mean on several cycles
     */
    uint32_t m_lastBreathPeriodsMs[NUMBER_OF_BREATH_PERIOD];
    /// Index for the m_lastBreathPeriodsMs array
    uint32_t m_lastBreathPeriodsMsIndex;
    /// Date of the last ending of a respiration
    uint32_t m_lastEndOfRespirationDateMs;

    /// Actual desired peak pressure
    int16_t m_peakPressureCommand;
    /// Measured value of peak pressure
    int16_t m_peakPressureMeasure;
    /// Measured value of rebounce peak pressure
    int16_t m_rebouncePeakPressureMeasure;
    /// Peak pressure desired by the operator for the next cycle
    int16_t m_peakPressureNextCommand;

    /// Actual desired plateau pressure
    int16_t m_plateauPressureCommand;
    /// Plateau pressure desired by the operator for the next cycle
    int16_t m_plateauPressureNextCommand;
    /**
     * Measured value of the plateau pressure
     *
     * @note Can sometimes have the special value of MAXINT
     */
    int16_t m_plateauPressureMeasure;
    /**
     * Measured value of the plateau pressure for display
     *
     * @note Special value of MAXINT in m_plateauPressureMeasure is replaced by 0
     */
    int16_t m_plateauPressureToDisplay;
    /// Sum for calulating plateau value
    int64_t m_PlateauMeasureSum;
    /// Count for calulating plateau value
    uint16_t m_PlateauMeasureCount;
    /**
     * Duration of the plateau
     *
     * @note This setting is used in trigger mode
     */
    uint32_t m_plateauDurationMs;

    /// Actual desired PEEP
    int16_t m_peepCommand;
    /// Desired PEEP for the next cycle
    int16_t m_peepNextCommand;
    /// Measured value of the PEEP
    int16_t m_peepMeasure;
    /// Is PEEP pressure detected in the cycle?
    bool m_isPeepDetected;

    /// Actual pressure trigger offset
    int16_t m_pressureTriggerOffsetCommand;
    /// Desired pressure trigger offset for the next cycle
    int16_t m_pressureTriggerOffsetNextCommand;
    /// Is inspiratory triggered or not?
    bool m_triggered;

    /// Actual state of enabling of trigger mode
    bool m_triggerModeEnabledCommand;
    /// Desired state of enabling of trigger mode for the next cycle
    bool m_triggerModeEnabledNextCommand;

    /// True if Tidal volume has already been read during cycle
    bool m_tidalVolumeAlreadyRead;

    /** Actual expiratory term
     *
     * E term of the I:E ratio. I = 10, and E is in [10;60]
     */
    uint16_t m_expiratoryTermCommand;
    /// Desired expiratory term for the next cycle
    uint16_t m_expiratoryTermNextCommand;

    /// Ventilation controller in use (for everything related to breathing control)
    VentilationController* m_ventilationController;
    /// Ventilation controller for the next cycle
    VentilationController* m_ventilationControllerNextCommand;

    /// Measured value of the Tidal volume (volume of air pushed in patient lungs in last
    /// inspiration)
    uint16_t m_tidalVolumeMeasure;

    /// Number of hundredth of second per cycle
    uint16_t m_ticksPerCycle;

    /// Number of hundredth of second per inhalation
    uint32_t m_ticksPerInhalation;

    /// Measured pressure
    int16_t m_pressure;

    /// Measured expiratory flow
    int32_t m_inspiratoryFlow;

    /// Measured inspiratory flow
    int32_t m_expiratoryFlow;

    /**
     * Current delivered volume by the blower
     *
     * @note This is not equal to Vt, because of recirculation during exhale
     */
    int32_t m_currentDeliveredVolume;

    /// Last pressure of inhalation
    int16_t m_inhalationLastPressure;

    /// Blower valve angle at peak
    uint32_t m_inspiratoryValveAngle;

    /// Current respiratory cycle phase
    CyclePhases m_phase;

    /// Number of elapsed cycles since beginning
    uint32_t m_cycleNb;

    /// Time since the last computation (in microsecond)
    int32_t m_dt;

    /// Requested pressure at a given point in time
    int32_t m_pressureCommand;

    /// Last pressure values
    int16_t m_lastPressureValues[MAX_PRESSURE_SAMPLES];

    /// Last pressure index
    uint16_t m_lastPressureValuesIndex;

    /// Sum of the current cycle's pressures
    uint32_t m_sumOfPressures;

    /// Number of the current cycle's pressures
    uint16_t m_numberOfPressures;
};

extern MainController mainController;
