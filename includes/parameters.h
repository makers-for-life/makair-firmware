/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file parameters.h
 * @brief Various settings
 *****************************************************************************/
#pragma once

// INCLUDES ===================================================================

// External
#include "Arduino.h"

// Internal
#include "../includes/config.h"

// PARAMETERS =================================================================

/// Current version of the software
#define VERSION "dms"

/**
 * @name Core parameters
 */
///@{

// Main controller computes period in millisecond: minimum 1 ms, maximum 10 ms
#define MAIN_CONTROLLER_COMPUTE_PERIOD_MS 10u
// cppcheck-suppress misra-c2012-5.4
#define MAIN_CONTROLLER_COMPUTE_PERIOD_MICROSECONDS (1000u * MAIN_CONTROLLER_COMPUTE_PERIOD_MS)

// Minimum and maximum bounds of execution parameters
#define CONST_MAX_PEAK_PRESSURE 700               // arbitrary [mmH2O]
#define CONST_MIN_PEAK_PRESSURE 100               // arbitrary [mmH2O]
#define CONST_MAX_PLATEAU_PRESSURE 400            // PP MAX ARDS = 300 [mmH2O]
#define CONST_MIN_PLATEAU_PRESSURE 50             // arbitrary [mmH2O]
#define CONST_MAX_PEEP_PRESSURE 300               // PP MAX = 300, or PEEP < PP [mmH2O]
#define CONST_MIN_PEEP_PRESSURE 0                 // arbitrary but > 0 [mmH2O]
#define CONST_MIN_TRIGGER_OFFSET 0u               // [mmH2O]
#define CONST_MAX_TRIGGER_OFFSET 100u             // [mmH2O]
#define CONST_INITIAL_ZERO_PRESSURE 0u            // [mmH2O]
#define CONST_INITIAL_ZERO_VOLUME 0u              // [mL]
#define CONST_MIN_TIDAL_VOLUME 50u                // [mL]
#define CONST_MAX_TIDAL_VOLUME 2000u              // [mL]
#define CONST_MAX_INSPIRATORY_TRIGGER_FLOW 100u   // [%]
#define CONST_MIN_INSPIRATORY_TRIGGER_FLOW 0u     // [%]
#define CONST_MAX_EXPIRATORY_TRIGGER_FLOW 100u    // [%]
#define CONST_MIN_EXPIRATORY_TRIGGER_FLOW 0u      // [%]
#define CONST_MIN_MIN_INSPIRATION_DURATION 100u   // [in ms]
#define CONST_MAX_MIN_INSPIRATION_DURATION 3000u  // [in ms]
#define CONST_MIN_MAX_INSPIRATION_DURATION 200u   // [in ms]
#define CONST_MAX_MAX_INSPIRATION_DURATION 5000u  // [in ms]
#define CONST_MIN_PLATEAU_DURATION 0u             // [in ms]
#define CONST_MAX_PLATEAU_DURATION 2000u          // [in ms]
#define CONST_MIN_INSPIRATORY_FLOW 5000           // [in ml/min]
#define CONST_MAX_INSPIRATORY_FLOW 80000          // [in ml/min]
#define CONST_MIN_INSPIRATORY_DURATION 200u       // [in ms]
#define CONST_MAX_INSPIRATORY_DURATION 3000u      // [in ms]
#define CONST_MIN_LOW_INSPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 0u    // [in L/min]
#define CONST_MAX_LOW_INSPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 20u   // [in L/min]
#define CONST_MIN_LOW_EXPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 0u     // [in L/min]
#define CONST_MAX_LOW_EXPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 20u    // [in L/min]
#define CONST_MIN_HIGH_INSPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 1u   // [in L/min]
#define CONST_MAX_HIGH_INSPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 40u  // [in L/min]
#define CONST_MIN_HIGH_EXPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 1u    // [in L/min]
#define CONST_MAX_HIGH_EXPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 40u   // [in L/min]
#define CONST_MIN_LOW_RESPIRATORY_RATE_ALARM_THRESHOLD 5u             // [in cpm]
#define CONST_MAX_LOW_RESPIRATORY_RATE_ALARM_THRESHOLD 25u            // [in cpm]
#define CONST_MIN_HIGH_RESPIRATORY_RATE_ALARM_THRESHOLD 15u           // [in cpm]
#define CONST_MAX_HIGH_RESPIRATORY_RATE_ALARM_THRESHOLD 35u           // [in cpm]
#define CONST_MIN_LOW_TIDAL_VOLUME_ALARM_THRESHOLD 0u                 // [in mL]
#define CONST_MAX_LOW_TIDAL_VOLUME_ALARM_THRESHOLD 1000u              // [in mL]
#define CONST_MIN_HIGH_TIDAL_VOLUME_ALARM_THRESHOLD 50u               // [in mL]
#define CONST_MAX_HIGH_TIDAL_VOLUME_ALARM_THRESHOLD 2000u             // [in mL]
#define CONST_MIN_LEAK_ALARM_THRESHOLD 0u                             // [in cL/min]
#define CONST_MIN_LEAK_ALARM_THRESHOLD 10000u                         // [in cL/min]
#define CONST_MIN_PEAK_PRESSURE_ALARM_THRESHOLD 50                    // [in mmH2O]
#define CONST_MAX_PEAK_PRESSURE_ALARM_THRESHOLD 700                   // [in mmH2O]
#define CONST_MIN_PATIENT_HEIGHT 30                                   // [in cm]
#define CONST_MAX_PATIENT_HEIGHT 250                                  // [in cm]
#define CONST_MIN_PATIENT_GENDER 0                                    // [male]
#define CONST_MAX_PATIENT_GENDER 1                                    // [female]

// Expiration term in the "Inspiration/Expiration" ratio given that Inspiration = 10
#define CONST_MIN_EXPIRATORY_TERM 10u
#define CONST_MAX_EXPIRATORY_TERM 60u

#define DEFAULT_PEEP_COMMAND 50u                        // in mmH2O
#define DEFAULT_PLATEAU_COMMAND 200u                    // in mmH2O
#define DEFAULT_PEAK_PRESSURE_COMMAND 200u              // in mmH2O
#define DEFAULT_EXPIRATORY_TERM_COMMAND 20u             // 20 means I:E = 10:20 = 1:2
#define DEFAULT_TIDAL_VOLUME_COMMAND 400u               // in mL
#define DEFAULT_PLATEAU_DURATION_COMMAND 200u           // in ms
#define DEFAULT_TRIGGER_OFFSET 20u                      // in mmH2O
#define DEFAULT_INSPIRATORY_TRIGGER_FLOW_COMMAND 10u    // in percent of current flow
#define DEFAULT_EXPIRATORY_TRIGGER_FLOW_COMMAND 30u     // in percent of max inspirated flow
#define DEFAULT_MIN_INSPIRATION_DURATION_COMMAND 200u   // in ms
#define DEFAULT_MAX_INSPIRATION_DURATION_COMMAND 1000u  // in ms
#define DEFAULT_TARGET_FLOW_COMMAND 40000u              // in mL/min
#define DEFAULT_INSPIRATORY_DURATION 800u               // in ms
#define DEFAULT_LOW_INSPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 3000u    // [in mL/min]
#define DEFAULT_LOW_EXPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 3000u     // [in mL/min]
#define DEFAULT_HIGH_INSPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 20000u  // [in mL/min]
#define DEFAULT_HIGH_EXPIRATORY_MINUTE_VOLUME_ALARM_THRESHOLD 20000u   // [in mL/min]
#define DEFAULT_LOW_RESPIRATORY_RATE_ALARM_THRESHOLD 16u               // [in cpm]
#define DEFAULT_HIGH_RESPIRATORY_RATE_ALARM_THRESHOLD 24u              // [in cpm]
#define DEFAULT_LOW_TIDAL_VOLUME_ALARM_THRESHOLD 200u                  // [in mL]
#define DEFAULT_HIGH_TIDAL_VOLUME_ALARM_THRESHOLD 1000u                // [in mL]
#define DEFAULT_LEAK_ALARM_THRESHOLD 2000u                             // [in mL/min]
#define DEFAULT_PEAK_PRESSURE_ALARM_THRESHOLD 500u                     // [in mmH2O]
#define DEFAULT_PATIENT_HEIGHT 170u                                    // [in cm]
#define DEFAULT_PATIENT_GENDER 0u                                      // [0 = male, 1 = female]

#define PEAK_PRESSURE_ALARM_THRESHOLD_OFFSET_FROM_PLATEAU 150  // [in mmH2O]
#define AIR_EXHAUST_THRESHOLD_FROM_PEAK_PRESSURE_ALARM 100     // [in mmH2O]

#define DEFAULT_CYCLE_PER_MINUTE_COMMAND 20u
#define CONST_MAX_CYCLE 35u
#define CONST_MIN_CYCLE 5u

#define DEFAULT_PEAK_PRESSURE_DELTA 10u

#define MAX_PRESSURE_OFFSET 40

#define MIN_EXPIRATORY_FLOW_OFFSET 2.5

#define TRIGGER_MODE_ENABLED_BY_DEFAULT false

///@}

/**
 * @name PID gains & settings
 */
///@{

static const int32_t PID_BLOWER_KP = 2000;
static const int32_t PID_BLOWER_KI = 50;
static const int32_t PID_BLOWER_KD = 0;
static const int32_t PID_BLOWER_INTEGRAL_MAX = 1000;
static const int32_t PID_BLOWER_INTEGRAL_MIN = -1000;

static const int32_t PID_PATIENT_KP = 15000;
static const int32_t PID_PATIENT_KI = 200;
static const int32_t PID_PATIENT_KD = 110;
static const int32_t PID_PATIENT_INTEGRAL_MAX = 1000;
static const int32_t PID_PATIENT_INTEGRAL_MIN = -1000;

/// Increase target pressure by an offset (in mmH2O) for safety, to avoid going below the target
/// pressure
static const int32_t PID_PATIENT_SAFETY_PEEP_OFFSET = 0;

#define PC_NUMBER_OF_SAMPLE_DERIVATIVE_MOVING_MEAN 10u

#define NUMBER_OF_SAMPLE_LAST_VALUES 20u

/// Number of periods used for calculating the respiratory rate
#define NUMBER_OF_BREATH_PERIOD 3u

///@}

/**
 * @name Valves
 */
///@{

/// Angle when opened
#define VALVE_OPEN_STATE 0

/// Angle when closed
#define VALVE_CLOSED_STATE 125
#define VALVE_PERIOD 1000     // 1 kHz Faulhaber motors are controlled with a 1 kHz PWM
#define FAULHABER_OPENED 660  // PWM duty cycle 64% -> open
#define FAULHABER_CLOSED 900  // PWM duty cycle 90% -> closed

#define VALVE_RESPONSE_TIME_MS 50  // estimated response time for going to open state to close state

#define PIN_INSPIRATORY_VALVE D5  // PB4 / TIM3_CH1
#define PIN_EXPIRATORY_VALVE D4   // PB5 / TIM3_CH2
#define TIM_CHANNEL_INSPIRATORY_VALVE 1
#define TIM_CHANNEL_EXPIRATORY_VALVE 2
#define ESC_PPM_PERIOD                                                                             \
    50  // ESC should be driven in 50 Hz. 100 Hz is a security against ESC or nucleo bugs. Some
           // ESC stops very quickly

///@}

/**
 * @name Blower
 */
///@{

#define MIN_BLOWER_SPEED 300u
#define MAX_BLOWER_SPEED 1800u
#define DEFAULT_BLOWER_SPEED 900u

///@}

/**
 * @name LCD screen
 */
///@{

#define PIN_LCD_RS PA8
#define PIN_LCD_RW PC12
#define PIN_LCD_EN PA5
#define PIN_LCD_D4 PC7   // PC7
#define PIN_LCD_D5 PB10  // PB6
#define PIN_LCD_D6 PA7   // PA7
#define PIN_LCD_D7 PA6   // PA6

/// Number of lines
#define SCREEN_LINE_NUMBER 4

/// Number of characters per line
#define SCREEN_LINE_LENGTH 20

/// Period between screen updates in microsecond. Should be a multiple of
/// MAIN_CONTROLLER_COMPUTE_PERIOD_MICROSECONDS
#define LCD_UPDATE_PERIOD_US 300000u

/// Period between screen resets in minutes
#define LCD_RESET_PERIOD 5

///@}

/**
 * @name Buttons
 */
///@{

#define PIN_BTN_ALARM_OFF PB2
#define PIN_BTN_START PC13
#define PIN_BTN_STOP PB15
// other buttons are in a 3x3 matrix
#define PIN_OUT_COL1 PC2
#define PIN_OUT_COL2 PC3
#define PIN_OUT_COL3 PC6
#define PIN_IN_ROW1 PC9
#define PIN_IN_ROW2 PC10
#define PIN_IN_ROW3 PC11
// expander used to read AC status
#define PIN_IN_MAINS_CONNECTED PB1
#define PIN_IN_CONNECTION_TO_SUPPLY_OK PB6

///@}

/**
 * @name LED
 */
///@{

#define PIN_LED_START PC8    // below start button, close to START label
#define PIN_LED_GREEN PB13   // "alarm off"
#define PIN_LED_YELLOW PB14  // "alarm med"
#define PIN_LED_RED PC4      // "alarm high"

#define LED_START_ACTIVE HIGH
#define LED_START_INACTIVE LOW
#define LED_RED_ACTIVE HIGH
#define LED_RED_INACTIVE LOW
#define LED_YELLOW_ACTIVE HIGH
#define LED_YELLOW_INACTIVE LOW
#define LED_GREEN_ACTIVE HIGH
#define LED_GREEN_INACTIVE LOW

///@}

/**
 * @name Other I/O
 */
///@{

#define PIN_PRESSURE_SENSOR PA1
#define PIN_BUZZER PB7       // TIM4_CH2
#define PIN_ESC_BLOWER PA10  // PA10 / TIM1_CH3
#define TIM_CHANNEL_ESC_BLOWER 3
#define PIN_TEMP_BLOWER PC1
#define PIN_BATTERY PA4
#define PIN_TELEMETRY_SERIAL_RX PB3  // UART1
#define PIN_TELEMETRY_SERIAL_TX PA9  // UART1
#define PIN_ENABLE_PWR_RASP PD2      // Raspberry Power Supply. High is OFF.
#define PWR_RASP_ACTIVE LOW
#define PWR_RASP_INACTIVE HIGH

///@}

/**
 * @name Flow meter
 */
///@{

#ifdef MASS_FLOW_METER_ENABLED
#define MASS_FLOW_TIMER TIM10
#define MASS_FLOW_CHANNEL 1
#define PIN_I2C_SDA PB9
#define PIN_I2C_SCL PB8
#define MFM_ANALOG_INPUT A3
#define MFM_POWER_CONTROL PC0
#define MFM_POWER_OFF LOW
#define MFM_POWER_ON HIGH
#endif
#define MASS_FLOW_ERROR_VALUE 999999

///@}

/**
 * @name Alarm thresholds
 */
///@{

#define ALARM_THRESHOLD_MIN_PRESSURE 20         // RCM-SW-2 + RCM-SW-19
#define ALARM_THRESHOLD_DIFFERENCE_PERCENT 20   // RCM-SW-1 + RCM-SW-14
#define ALARM_THRESHOLD_DIFFERENCE_PRESSURE 20  // RCM-SW-3 + RCM-SW-15

///@}

#define ADC_RESOLUTION_MAKAIR 12
