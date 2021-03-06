/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file battery.h
 * @brief Battery related functions
 *****************************************************************************/

#pragma once

#include "Arduino.h"

/**
 * The divider between real battery voltage and STM32 input is 8.2K-1k resistors
 * So, the multiplier is 1/(1+8.2)=0.1087
 * Considering 0.1% precision resistances, multiplier is between 0,108502 and 0,108889
 * The reference is 3.348V (measured on 53 HW V3 boards)
 * RawValue = (Vbat*0.1087)*1024/3.348
 * So, Vbat = RawValue * (3.348/(1024*0.1087))
 *
 * 1 bit = 3.348/(4096*0.1087) = 7.5mV
 */
#define RAW_BATTERY_MULTIPLIER 0.0075196210

/**
 * Expected voltage in volts when power cord is plugged
 * 27,4 V => 27,4 / RAW_BATTERY_MULTIPLIER
 */
#define RAW_VOLTAGE_MAINS 3643u

/**
 * RCM_SW_16
 * Expected voltage in volts when power cord is unplugged
 *  = 27 => 27 / RAW_BATTERY_MULTIPLIER
 */
#define RAW_VOLTAGE_ON_BATTERY_HIGH 3590u

/**
 * Hysteresis is used to prevent fast switching when voltage is at the limit of 2 states
 *  analogRead(PIN) * RAW_BATTERY_MULTIPLIER = 0,1 => 0,1 / RAW_BATTERY_MULTIPLIER = 3
 */
#define RAW_VOLTAGE_HYSTERESIS 12u

/**
 * RCM_SW_11
 *  = 23,2 => 23,2 / RAW_BATTERY_MULTIPLIER
 */
#define RAW_VOLTAGE_ON_BATTERY 3085u

/**
 * RCM_SW_12
 *  = 22,6 => 22,6 / RAW_BATTERY_MULTIPLIER
 */
#define RAW_VOLTAGE_ON_BATTERY_LOW 3005u

/**
 * Below this value, the machine wont start
 *  = 22 => 22 / RAW_BATTERY_MULTIPLIER
 */
#define RAW_VOLTAGE_ON_BATTERY_NOT_STARTING_THRESHOLD 3191u

/**
 * Below this value, the machine will stop immediately
 *  = 20 => 20 / RAW_BATTERY_MULTIPLIER
 */
#define RAW_VOLTAGE_ON_BATTERY_STOP_THRESHOLD 2660u

/// Number of samples of the moving average
#define BATTERY_MAX_SAMPLES 20u

/**
 * Initialize battery abstraction
 *
 * @warning It must be called once to be able to check battery level
 */
void initBattery();

/**
 * Handle battery events
 *
 * @param p_cycleNumber Number of cycles since boot
 * @warning It must be called in the program loop
 */
void batteryLoop(uint32_t p_cycleNumber);

/// Handle battery voltage calculation
void updateBatterySample();

/**
 * Updates battery states
 *
 * @param p_cycleNumber Number of cycle since start
 */
void updateBatteryState(uint32_t p_cycleNumber);

/**
 * Returns battery level
 *
 * @return Battery level in volts
 */
uint32_t getBatteryLevel();

/**
 * Returns battery level x10 for better accuracy
 *
 * @return Battery level in volts x10
 */
uint32_t getBatteryLevelX10();

/**
 * Returns battery level x100 for better accuracy
 *
 * @return Battery level in volts x100
 */
uint32_t getBatteryLevelX100();

/// Check if battery level is very low
bool isBatteryVeryLow();

/// Check if battery is deeply discharged
bool isBatteryDeepDischarged();

/// Check if mains are connected
bool isMainsConnected();

/// Check if the cable between power supply and expander input is connected
bool isMainsAvailable();
