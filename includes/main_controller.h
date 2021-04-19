/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file main_controller.h
 * @brief Core logic to control the breathing cycle
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Internal

#include "../includes/alarm_controller.h"
#include "../includes/battery.h"
#include "../includes/blower.h"
#include "../includes/config.h"
#include "../includes/cycle.h"
#include "../includes/debug.h"
#include "../includes/parameters.h"
#include "../includes/pc_ac_controller.h"
#include "../includes/pc_cmv_controller.h"
#include "../includes/pc_vsai_controller.h"
#include "../includes/pressure_valve.h"
#include "../includes/telemetry.h"
#include "../includes/vc_ac_controller.h"
#include "../includes/vc_cmv_controller.h"

/// Number of values to aggregate when computing plateau pressure
#define MAX_PRESSURE_SAMPLES 10u

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

    /// Set ventilation mode
    void onVentilationModeSet(uint16_t p_ventilationControllerMode);

    /// Set inspiratory trigger flow
    void onInspiratoryTriggerFlowSet(uint16_t p_inspiratoryTriggerFlow);

    /// Set expiratory trigger flow
    void onExpiratoryTriggerFlowSet(uint16_t p_expiratoryTriggerFlow);

    /// Set min inspiratory time
    void onTiMinSet(uint16_t p_tiMin);

    /// Set max inspiratory time
    void onTiMaxSet(uint16_t p_tiMax);

    /// Set alarm threshold for low inspiratory minute volume
    void onLowInspiratoryMinuteVolumeAlarmThresholdSet(
        uint16_t p_lowInspiratoryMinuteVolumeAlarmThreshold);

    /// Set alarm threshold for high inspiratory minute volume
    void onHighInspiratoryMinuteVolumeAlarmThresholdSet(
        uint16_t p_highInspiratoryMinuteVolumeAlarmThreshold);

    /// Set alarm threshold for low expiratory minute volume
    void onLowExpiratoryMinuteVolumeAlarmThresholdSet(
        uint16_t p_lowExpiratoryMinuteVolumeAlarmThreshold);

    /// Set alarm threshold for high expiratory minute volume
    void onHighExpiratoryMinuteVolumeAlarmThresholdSet(
        uint16_t p_highExpiratoryMinuteVolumeAlarmThreshold);

    /// Set alarm threshold for low respiratory rate
    void onlowRespiratoryRateAlarmThresholdSet(uint16_t p_lowRespiratoryRateAlarmThreshold);

    /// Set alarm threshold for high respiratory rate
    void onhighRespiratoryRateAlarmThresholdSet(uint16_t p_highRespiratoryRateAlarmThreshold);

    /// Set target tidal volume (used in VC modes)
    void onTargetTidalVolumeSet(uint16_t p_targetTidalVolume);

    /// Set threshold on tidal volume below which an alarm is raised
    void onLowTidalVolumeAlarmThresholdSet(uint16_t p_lowTidalVolumeAlarmThreshold);

    /// Set threshold on tidal volume for which an alarm is raised
    void onHighTidalVolumeAlarmThresholdSet(uint16_t p_highTidalVolumeAlarmThreshold);

    /// Set the duration of Pause at the end of expiration in VC modes
    void onPlateauDurationSet(uint16_t p_plateauDuration);

    /// Set the threshold for leak that raise the alarm
    void onLeakAlarmThresholdSet(uint16_t p_leakAlarmThreshold);

    /// Set the inspiratory flow target
    void onTargetInspiratoryFlow(uint16_t p_targetInspiratoryFlow);

    /// Set the inspiration duration
    void onInspiratoryDuration(uint16_t p_inspiratoryDuration);

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

    /**
     * Set the desired patient height
     *
     * @param p_patientHeight Desired patient height in cm
     */
    void onPatientHeight(int16_t p_patientHeight);

    /**
     * Set the desired threshold for max peak pressure
     *
     * @param p_peakPressureAlarmThreshold Desired threshold in mmH2O
     */
    void onPeakPressureAlarmThreshold(int16_t p_peakPressureAlarmThreshold);

    /**
     * Set the desired patient gender
     *
     * @param p_patientGender patient gender 0 = male, 1 = female
     */
    void onPatientGender(int16_t p_patientGender);

    /**
     * Updates patient computed params
     *
     */
    void onPatientComputePreset();

    // Get the patient Height in cm
    inline int32_t patientHeight() const { return m_patientHeight; }
    // Get the desired tidalVolue
    inline int16_t tidalVolumeCommand() const { return m_tidalVolumeCommand; }
    // Get the desired plateau duration (in VC modes)
    inline int16_t plateauDurationCommand() const { return m_plateauDurationCommand; }
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
    /// Get the value of the inspiratory trigger flow command
    inline const int16_t inspiratoryTriggerFlowCommand() const {
        return m_inspiratoryTriggerFlowCommand;
    }
    /// Get the value of the expiratory trigger flow command
    inline const int16_t expiratoryTriggerFlowCommand() const {
        return m_expiratoryTriggerFlowCommand;
    }
    /// Get the value of the minimum duration of inspiration in ms
    inline const int16_t tiMinCommand() const { return m_tiMinCommand; }
    /// Get the value of the max duration of inspiration in ms
    inline const int16_t tiMaxCommand() const { return m_tiMaxCommand; }
    /// get target inspiratory flow in mL/min (used in VC modes)
    inline const int32_t targetInspiratoryFlowCommand() const {
        return m_targetInspiratoryFlowCommand;
    }
    /// Get duration of inspiration command
    inline const int16_t inspiratoryDurationCommand() const { return m_inspiratoryDurationCommand; }

    /// Get the desired tidal Volume for the next cycle (used in VC modes)
    inline int16_t tidalVolumeNextCommand() const { return m_tidalVolumeNextCommand; }
    // Get the desired plateau duration for the next cycle (used in VC modes)
    inline int16_t plateauDurationNextCommand() const { return m_plateauDurationNextCommand; }
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
    /// Get the value of the inspiratory trigger flow command for the next cycle
    inline const int16_t inspiratoryTriggerFlowNextCommand() const {
        return m_inspiratoryTriggerFlowNextCommand;
    }
    /// Get the value of the expiratory trigger flow command for the next cycle
    inline const int16_t expiratoryTriggerFlowNextCommand() const {
        return m_expiratoryTriggerFlowNextCommand;
    }
    /// Get the value of the minimum duration of inspiration in ms for the next cycle
    inline const int16_t tiMinNextCommand() const { return m_tiMinNextCommand; }
    /// Get the value of the max duration of inspiration in ms for the next cycle
    inline const int16_t tiMaxNextCommand() const { return m_tiMaxNextCommand; }
    /// get target inspiratory flow in mL/min (used in VC modes) for next cycle
    inline const int32_t targetInspiratoryFlowNextCommand() const {
        return m_targetInspiratoryFlowNextCommand;
    }
    /// Get duration of inspiration command fo next cycle
    inline const int16_t inspiratoryDurationNextCommand() const {
        return m_inspiratoryDurationNextCommand;
    }
    /// Ventilation controller pointer for the next cycle
    inline const VentilationController* ventilationControllerNextCommand() const {
        return m_ventilationControllerNextCommand;
    }

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
    /// Get the measured Tidal Volume. Updated only at the end of inspiration
    inline uint16_t tidalVolumeMeasure() const { return m_tidalVolumeMeasure; }
    /// Get the measured Tidal Volume. Updated in real time
    inline int32_t currentDeliveredVolume() const { return m_currentDeliveredVolume; }

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

    /// Get last pressure values
    inline int16_t* lastPressureValues() { return m_lastPressureValues; }

    /// Get last pressure values index
    inline uint16_t lastPressureValuesIndex() { return m_lastPressureValuesIndex; }

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

    // current expiratory Volume
    void updateCurrentExpiratoryVolume(int32_t p_expiratoryVolume);

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

    /// Desired inspiratory trigger flow
    int16_t m_inspiratoryTriggerFlowCommand;
    /// Desired inspiratory trigger flow for next cycle
    int16_t m_inspiratoryTriggerFlowNextCommand;

    /// Desired expiratory trigger flow (in percent of max flow)
    int16_t m_expiratoryTriggerFlowCommand;
    /// Desired expiratory trigger flow for next cycle (in percent of max flow)
    int16_t m_expiratoryTriggerFlowNextCommand;

    /// Minimum duration of inspiration in ms
    int16_t m_tiMinCommand;
    /// Max duration of inspiration in ms
    int16_t m_tiMaxCommand;
    /// Minimum duration of inspiration in ms for next cycle
    int16_t m_tiMinNextCommand;
    /// Max duration of inspiration in ms for next cycle
    int16_t m_tiMaxNextCommand;

    /// Ventilation controller in use (for everything related to breathing control)
    VentilationController* m_ventilationController;
    /// Ventilation controller for the next cycle
    VentilationController* m_ventilationControllerNextCommand;

    /// Array containing pointers to different ventilation controllers
    VentilationController* m_ventilationControllersTable[NUMBER_OF_VENTILATION_MODES + 1u];

    VentilationModes m_ventilationControllerMode;

    /// Measured value of the Tidal volume (volume of air pushed in patient lungs in last
    /// inspiration)
    int16_t m_tidalVolumeMeasure;

    /// Tidal volume command (used in VC modes)
    int16_t m_tidalVolumeCommand;
    /// Tidal volume command for next cycle
    int16_t m_tidalVolumeNextCommand;

    /// Plateau duration command (used in VC modes)
    int16_t m_plateauDurationCommand;
    /// Plateau duration command for next cycle
    int16_t m_plateauDurationNextCommand;

    /// inspiratory flow required (used in VC modes)
    int32_t m_targetInspiratoryFlowCommand;
    /// inspiratory flow required (used in VC modes) for next cycle
    int32_t m_targetInspiratoryFlowNextCommand;
    /// Duration of inspiration
    int16_t m_inspiratoryDurationCommand;
    /// Duration of inspiration
    int16_t m_inspiratoryDurationNextCommand;

    /// Threshold for low inspiratory minute volume alarm
    int32_t m_lowInspiratoryMinuteVolumeAlarmThresholdCommand;
    /// Threshold for low inspiratory minute volume alarm for next cycle
    // cppcheck-suppress misra-c2012-5.2
    int32_t m_lowInspiratoryMinuteVolumeAlarmThresholdNextCommand;

    /// Threshold for high inspiratory minute volume alarm
    int32_t m_highInspiratoryMinuteVolumeAlarmThresholdCommand;
    /// Threshold for high inspiratory minute volume alarm for next cycle
    // cppcheck-suppress misra-c2012-5.2
    int32_t m_highInspiratoryMinuteVolumeAlarmThresholdNextCommand;

    /// Threshold for low inspiratory minute volume alarm
    int32_t m_lowExpiratoryMinuteVolumeAlarmThresholdCommand;
    /// Threshold for low inspiratory minute volume alarm for next cycle
    // cppcheck-suppress misra-c2012-5.2
    int32_t m_lowExpiratoryMinuteVolumeAlarmThresholdNextCommand;

    /// Threshold for high inspiratory minute volume alarm
    int32_t m_highExpiratoryMinuteVolumeAlarmThresholdCommand;
    /// Threshold for high inspiratory minute volume alarm for next cycle
    // cppcheck-suppress misra-c2012-5.2
    int32_t m_highExpiratoryMinuteVolumeAlarmThresholdNextCommand;

    /// Threshold for low respiratory rate
    int32_t m_lowRespiratoryRateAlarmThresholdCommand;
    /// Threshold for low respiratory rate for next cycle
    // cppcheck-suppress misra-c2012-5.2
    int32_t m_lowRespiratoryRateAlarmThresholdNextCommand;

    /// Threshold for low respiratory rate
    int32_t m_highRespiratoryRateAlarmThresholdCommand;
    /// Threshold for low respiratory rate for next cycle
    // cppcheck-suppress misra-c2012-5.2
    int32_t m_highRespiratoryRateAlarmThresholdNextCommand;

    /// Threshold for low tidal Volume Alarm
    int32_t m_lowTidalVolumeAlarmThresholdCommand;
    /// Threshold for low tidal Volume Alarm next cycle
    int32_t m_lowTidalVolumeAlarmThresholdNextCommand;

    /// Threshold for high tidal Volume Alarm
    int32_t m_highTidalVolumeAlarmThresholdCommand;
    /// Threshold for high tidal Volume Alarm next cycle
    // cppcheck-suppress misra-c2012-5.2
    int32_t m_highTidalVolumeAlarmThresholdNextCommand;

    /// Threshold for leak alarm
    int32_t m_leakAlarmThresholdCommand;
    /// Threshold for leak alarm for next cycle
    int32_t m_leakAlarmThresholdNextCommand;

    /// Threshold for peak pressure alarm
    int16_t m_peakPressureAlarmThresholdCommand;
    /// Threshold for peak pressure alarmfor next cycle
    int16_t m_peakPressureAlarmThresholdNextCommand;

    /// Volume expired by the patient during the exhalation phase
    int32_t m_expiratoryVolume;

    /// Number of hundredth of second per cycle
    uint16_t m_ticksPerCycle;

    /// Number of hundredth of second per inhalation
    uint32_t m_ticksPerInhalation;

    /// Measured pressure
    int16_t m_pressure;

    /// Measured inspiratory flow
    int32_t m_inspiratoryFlow;

    /// Measured max inspiratory flow
    int32_t m_maxInspiratoryFlow;

    /// Measured inspiratory flow
    int32_t m_expiratoryFlow;

    /// Measured max inspiratory flow
    int32_t m_maxExpiratoryFlow;

    /// Measured max inspiratory flow
    int32_t m_lastMaxExpiratoryFlow;

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

    // Height of the patient in cm
    int32_t m_patientHeight;

    // Gender of patient 0 = male, 1 = female
    int32_t m_patientGender;
};

extern MainController mainController;
