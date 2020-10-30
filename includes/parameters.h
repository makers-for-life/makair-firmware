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
#define VERSION "dev"

/**
 * @name Core parameters
 */
///@{

// Période de traitement en microseconds
#define PCONTROLLER_COMPUTE_PERIOD_US 10000u

// Minimum & maximum execution parameters
#define CONST_MAX_PEAK_PRESSURE 700u     // arbitrary [mmH2O]
#define CONST_MIN_PEAK_PRESSURE 100u     // arbitrary [mmH2O]
#define CONST_MAX_PLATEAU_PRESSURE 400u  // PP MAX ARDS = 300 [mmH2O]
#define CONST_MIN_PLATEAU_PRESSURE 100u  // arbitrary [mmH2O]
#define CONST_MAX_PEEP_PRESSURE 300u     // PP MAX = 300, or PEEP < PP [mmH2O]
#define CONST_MIN_PEEP_PRESSURE 50u      // arbitrary but > 0 [mmH2O]
#define CONST_MIN_TRIGGER_OFFSET 0u      // [mmH2O]
#define CONST_MAX_TRIGGER_OFFSET 100u    // [mmH2O]
#define CONST_INITIAL_ZERO_PRESSURE 0    // [mmH2O]

// Expiration term in the "Inspiration/Expiration" ratio given that Inspiration = 10
#define CONST_MIN_EXPIRATORY_TERM 10u
#define CONST_MAX_EXPIRATORY_TERM 60u

#define DEFAULT_MIN_PEEP_COMMAND 100
#define DEFAULT_MAX_PLATEAU_COMMAND 200
#define DEFAULT_MAX_PEAK_PRESSURE_COMMAND 220
#define DEFAULT_EXPIRATORY_TERM_COMMAND 20
#define DEFAULT_TRIGGER_OFFSET 20

#define INITIAL_CYCLE_NUMBER 20
#define CONST_MAX_CYCLE 35u
#define CONST_MIN_CYCLE 5u

#define DEFAULT_PEAK_PRESSURE_DELTA 10u

#define MAX_PRESSURE_OFFSET 40

#define TRIGGER_MODE_ENABLED_BY_DEFAULT false

///@}

/**
 * @name PID gains & settings
 */
///@{

#if PNEUMATIC_HARDWARE_VERSION == PHW_PIGGY

static const int32_t PID_BLOWER_KP = 4;
static const int32_t PID_BLOWER_KI = 32;
static const int32_t PID_BLOWER_KD = 4;
static const int32_t PID_BLOWER_INTEGRAL_MAX = 1000;
static const int32_t PID_BLOWER_INTEGRAL_MIN = -1000;

static const int32_t PID_PATIENT_KP = 4;
static const int32_t PID_PATIENT_KI = 32;
static const int32_t PID_PATIENT_KD = 8;
static const int32_t PID_PATIENT_INTEGRAL_MAX = 500;
static const int32_t PID_PATIENT_INTEGRAL_MIN = -500;

/// Increase target pressure by an offset (in mmH2O) for safety, to avoid going below the target
/// pressure
static const int32_t PID_PATIENT_SAFETY_PEEP_OFFSET = 20;

#elif PNEUMATIC_HARDWARE_VERSION == PHW_CHU

static const int32_t PID_BLOWER_KP = 1;
static const int32_t PID_BLOWER_KI = 25;
static const int32_t PID_BLOWER_KD = 0;
static const int32_t PID_BLOWER_INTEGRAL_MAX = 1000;
static const int32_t PID_BLOWER_INTEGRAL_MIN = -1000;

static const int32_t PID_PATIENT_KP = 4;
static const int32_t PID_PATIENT_KI = 32;
static const int32_t PID_PATIENT_KD = 12;
static const int32_t PID_PATIENT_INTEGRAL_MAX = 630;
static const int32_t PID_PATIENT_INTEGRAL_MIN = -630;

/// Increase target pressure by an offset (in mmH2O) for safety, to avoid going below the target
/// pressure
static const int32_t PID_PATIENT_SAFETY_PEEP_OFFSET = 0;

#elif PNEUMATIC_HARDWARE_VERSION == PHW_FAULHABER

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

#endif

#define NUMBER_OF_SAMPLE_BLOWER_DERIVATIVE_MOVING_MEAN 10u

/// Number of periods used for calculating the respiratory rate
#define NUMBER_OF_BREATH_PERIOD 3u

///@}

/**
 * @name Valves
 */
///@{

/// Angle when opened
#define VALVE_OPEN_STATE 0u

/// Angle when closed
#define VALVE_CLOSED_STATE 125u

#if VALVE_TYPE == VT_SERVO_V1
#define SERVO_VALVE_PERIOD                                                                         \
    10000  // 100 Hz : on hardware 1, esc timer is shared between servo and esc. Servo can handle
           // 100 Hz too
#elif VALVE_TYPE == VT_EMERSON_ASCO
#define SERVO_VALVE_PERIOD 3278  // 305 Hz
#define EMERSON_MIN_PWM 600      // 18 % PWM is the minimum to start opening (3278 * 0.18)
#elif VALVE_TYPE == VT_FAULHABER
#define SERVO_VALVE_PERIOD 1000  // 1 kHz Faulhaber motors are controlled with a 1 kHz PWM
#define FAULHABER_OPENED 640     // PWM duty cycle 64% -> open
#define FAULHABER_CLOSED 900     // PWM duty cycle 90% -> closed
#endif

#if HARDWARE_VERSION == 1
#define PIN_SERVO_BLOWER D2   // PA10 / TIM1_CH3
#define PIN_SERVO_PATIENT D4  // PB5 / TIM3_CH2
#define TIM_CHANNEL_SERVO_VALVE_BLOWER 3
#define TIM_CHANNEL_SERVO_VALVE_PATIENT 2
#elif HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
#define PIN_SERVO_BLOWER D5   // PB4 / TIM3_CH1
#define PIN_SERVO_PATIENT D4  // PB5 / TIM3_CH2
#define TIM_CHANNEL_SERVO_VALVE_BLOWER 1
#define TIM_CHANNEL_SERVO_VALVE_PATIENT 2
#define ESC_PPM_PERIOD                                                                             \
    10000  // ESC should be driven in 50 Hz. 100 Hz is a security against ESC or nucleo bugs. Some
           // ESC stops very quickly
#endif

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

#if HARDWARE_VERSION == 1
#define PIN_LCD_RS D7
#define PIN_LCD_RW PA13
#define PIN_LCD_EN D8
#define PIN_LCD_D4 D9
#define PIN_LCD_D5 D10
#define PIN_LCD_D6 D11
#define PIN_LCD_D7 D12
#elif HARDWARE_VERSION == 2
#define PIN_LCD_RS D7  // PA8
#define PIN_LCD_RW PA14
#define PIN_LCD_EN D8   // PA9
#define PIN_LCD_D4 D9   // PC7
#define PIN_LCD_D5 D10  // PB6
#define PIN_LCD_D6 D11  // PA7
#define PIN_LCD_D7 D12  // PA6
#elif HARDWARE_VERSION == 3
#define PIN_LCD_RS PA8
#define PIN_LCD_RW PC12
#define PIN_LCD_EN PA5
#define PIN_LCD_D4 PC7   // PC7
#define PIN_LCD_D5 PB10  // PB6
#define PIN_LCD_D6 PA7   // PA7
#define PIN_LCD_D7 PA6   // PA6
#endif

/// Number of lines
#define SCREEN_LINE_NUMBER 4

/// Number of characters per line
#define SCREEN_LINE_LENGTH 20

/// Period between screen updates in microsecond. Should be a multiple of
/// PCONTROLLER_COMPUTE_PERIOD_US
#define LCD_UPDATE_PERIOD_US 300000u

/// Period between screen resets in minutes
#define LCD_RESET_PERIOD 5

///@}

/**
 * @name Buttons
 */
///@{

#if HARDWARE_VERSION == 1
#define PIN_CONTROL_BUTTONS A0
#define PIN_BTN_ALARM_OFF PB2
#define PIN_BTN_START PB1
#define PIN_BTN_STOP PB15
#define VOLTAGE_BUTTON_PEAK_PRESSURE_INCREASE 913
#define VOLTAGE_BUTTON_PEAK_PRESSURE_DECREASE 819
#define VOLTAGE_BUTTON_PLATEAU_PRESSURE_INCREASE 745
// cppcheck-suppress misra-c2012-5.4 ; symbol name is too long, should be < 31 chars
#define VOLTAGE_BUTTON_PLATEAU_PRESSURE_DECREASE 608
#define VOLTAGE_BUTTON_PEEP_PRESSURE_INCREASE 512
#define VOLTAGE_BUTTON_PEEP_PRESSURE_DECREASE 415
#define VOLTAGE_BUTTON_CYCLE_INCREASE 294
#define VOLTAGE_BUTTON_CYCLE_DECREASE 216
#elif HARDWARE_VERSION == 2
#define PIN_BTN_ALARM_OFF PB2
#define PIN_BTN_START PB1
#define PIN_BTN_STOP PB15
#define PIN_BTN_PEAK_PRESSURE_INCREASE PC3
#define PIN_BTN_PEAK_PRESSURE_DECREASE PC6
#define PIN_BTN_PLATEAU_PRESSURE_INCREASE PC2
#define PIN_BTN_PLATEAU_PRESSURE_DECREASE PC13
#define PIN_BTN_PEEP_PRESSURE_INCREASE PH1
#define PIN_BTN_PEEP_PRESSURE_DECREASE PC0
#define PIN_BTN_CYCLE_INCREASE PA0
#define PIN_BTN_CYCLE_DECREASE PB7
#elif HARDWARE_VERSION == 3
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
#endif

///@}

/**
 * @name LED
 */
///@{

#if HARDWARE_VERSION == 1

#define PIN_LED_START PC8
#define PIN_LED_GREEN PB13
#define PIN_LED_YELLOW PB14
#define PIN_LED_RED PC4

#define LED_START_ACTIVE HIGH
#define LED_START_INACTIVE LOW
#define LED_RED_ACTIVE HIGH
#define LED_RED_INACTIVE LOW
#define LED_YELLOW_ACTIVE HIGH
#define LED_YELLOW_INACTIVE LOW
#define LED_GREEN_ACTIVE HIGH
#define LED_GREEN_INACTIVE LOW

#elif HARDWARE_VERSION == 2

#define PIN_LED_START PC8
#define PIN_LED_GREEN PB13
#define PIN_LED_YELLOW PB14
#define PIN_LED_RED PC4

#define LED_START_ACTIVE LOW
#define LED_START_INACTIVE HIGH
#define LED_RED_ACTIVE LOW
#define LED_RED_INACTIVE HIGH
#define LED_YELLOW_ACTIVE LOW
#define LED_YELLOW_INACTIVE HIGH
#define LED_GREEN_ACTIVE LOW
#define LED_GREEN_INACTIVE HIGH

#elif HARDWARE_VERSION == 3

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

#endif

///@}

/**
 * @name Other I/O
 */
///@{

#if HARDWARE_VERSION == 1
#define PIN_PRESSURE_SENSOR A1
#define PIN_BUZZER D13     // PA5
#define PIN_ESC_BLOWER D5  // PB4 / TIM3_CH1
#define TIM_CHANNEL_ESC_BLOWER 1
#define PIN_BATTERY A2
#ifdef MASS_FLOW_METER
#error "Hardware v1 cannot support any kind of mass flow meter sensor"
#endif
#elif HARDWARE_VERSION == 2
#define PIN_PRESSURE_SENSOR A1
#define PIN_BUZZER D13     // PA5
#define PIN_ESC_BLOWER D2  // PA10 / TIM1_CH3
#define TIM_CHANNEL_ESC_BLOWER 3
#define PIN_TEMP_BLOWER A4  // PC1
#define PIN_BATTERY A2
#define PIN_TELEMETRY_SERIAL_RX PA12
#define PIN_TELEMETRY_SERIAL_TX PA11
/**
 * Define the flow meter parameters for Hardware v2
 */
#ifdef MASS_FLOW_METER
#define MASS_FLOW_TIMER TIM10
#define MASS_FLOW_CHANNEL 1
#define PIN_I2C_SDA PB9
#define PIN_I2C_SCL PB8
#define MFM_ANALOG_INPUT A3
#endif
#elif HARDWARE_VERSION == 3
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
/**
 * Define the flow meter parameters for Hardware v3
 */
#ifdef MASS_FLOW_METER
#define MASS_FLOW_TIMER TIM10
#define MASS_FLOW_CHANNEL 1
#define PIN_I2C_SDA PB9
#define PIN_I2C_SCL PB8
#define MFM_ANALOG_INPUT A3
#define MFM_POWER_CONTROL PC0
#define MFM_POWER_OFF LOW
#define MFM_POWER_ON HIGH
#endif
#endif

///@}

/**
 * @name Alarm thresholds
 */
///@{

#define ALARM_THRESHOLD_MIN_PRESSURE 20u         // RCM-SW-2 + RCM-SW-19
#define ALARM_THRESHOLD_MAX_PRESSURE 800u        // RCM-SW-18
#define ALARM_THRESHOLD_DIFFERENCE_PERCENT 20u   // RCM-SW-1 + RCM-SW-14
#define ALARM_THRESHOLD_DIFFERENCE_PRESSURE 20u  // RCM-SW-3 + RCM-SW-15

///@}

// Preprocessor checks
#if VALVE_TYPE == VT_FAULHABER && HARDWARE_VERSION == 1
#error "Faulhaber can only be driven with hardware v2+"
#endif
