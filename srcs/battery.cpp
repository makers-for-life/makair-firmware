/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file battery.cpp
 * @brief Battery related functions
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/battery.h"

// Externals
#include "Arduino.h"

// Internal
#include "../includes/alarm_controller.h"
#include "../includes/debug.h"
#include "../includes/parameters.h"

// PROGRAM =====================================================================

static uint32_t rawBatterySample[BATTERY_MAX_SAMPLES];  // Array to store battery voltage samples
static uint32_t batteryCurrentSample = 0;               // Current battery sample index
static uint32_t batteryTotalSamples = 0;                // Battery total samples
static uint32_t rawBatteryMeanVoltage = RAW_VOLTAGE_MAINS;  // Mean battery voltage in volts
static bool isRunningOnBattery = false;
static bool mainsConnected = false;

void initBattery() {
    for (uint8_t i = 0; i < BATTERY_MAX_SAMPLES; i++) {
        rawBatterySample[i] = 0;
    }

    // Running this in setup avoids triggering alarms at startup
    for (uint8_t i = 0; i < BATTERY_MAX_SAMPLES; i++) {
        updateBatterySample();
    }
}

void updateBatterySample() {
    uint16_t rawVout = analogRead(PIN_BATTERY);

    // Assign sample from Vout
    rawBatterySample[batteryCurrentSample] = rawVout;

    // Increment sample
    batteryCurrentSample++;

    // If we reached max samples
    if (batteryCurrentSample >= BATTERY_MAX_SAMPLES) {
        batteryCurrentSample = 0;
    }

    batteryTotalSamples = 0;
    for (uint8_t i = 0; i < BATTERY_MAX_SAMPLES; i++) {
        batteryTotalSamples += rawBatterySample[i];
    }
    // Updates mean voltage
    rawBatteryMeanVoltage = (batteryTotalSamples / BATTERY_MAX_SAMPLES);
}

void updateBatteryState(uint32_t p_cycleNumber) {
#if HARDWARE_VERSION == 3
    pinMode(PIN_IN_MAINS_CONNECTED, INPUT_PULLUP);
    mainsConnected = (LOW == digitalRead(PIN_IN_MAINS_CONNECTED));
#else
    mainsConnected = false;
#endif

    if (!mainsConnected
        && (rawBatteryMeanVoltage < (RAW_VOLTAGE_ON_BATTERY_HIGH - RAW_VOLTAGE_HYSTERESIS))) {
        alarmController.detectedAlarm(RCM_SW_16, p_cycleNumber,
                                      (RAW_VOLTAGE_ON_BATTERY_HIGH - RAW_VOLTAGE_HYSTERESIS),
                                      rawBatteryMeanVoltage);
        isRunningOnBattery = true;
    } else if (mainsConnected || (rawBatteryMeanVoltage > RAW_VOLTAGE_ON_BATTERY_HIGH)) {
        alarmController.notDetectedAlarm(RCM_SW_16);
        isRunningOnBattery = false;
    } else {
        // This is an hysteresis, so do nothing here
    }

    if (!mainsConnected
        && (rawBatteryMeanVoltage < (RAW_VOLTAGE_ON_BATTERY - RAW_VOLTAGE_HYSTERESIS))) {
        alarmController.detectedAlarm(RCM_SW_11, p_cycleNumber,
                                      (RAW_VOLTAGE_ON_BATTERY - RAW_VOLTAGE_HYSTERESIS),
                                      rawBatteryMeanVoltage);
    } else if (mainsConnected || (rawBatteryMeanVoltage > RAW_VOLTAGE_ON_BATTERY)) {
        alarmController.notDetectedAlarm(RCM_SW_11);
    } else {
        // This is an hysteresis, so do nothing here
    }

    if (!mainsConnected
        && (rawBatteryMeanVoltage < (RAW_VOLTAGE_ON_BATTERY_LOW - RAW_VOLTAGE_HYSTERESIS))) {
        alarmController.detectedAlarm(RCM_SW_12, p_cycleNumber,
                                      (RAW_VOLTAGE_ON_BATTERY_LOW - RAW_VOLTAGE_HYSTERESIS),
                                      rawBatteryMeanVoltage);
    } else if (mainsConnected || (rawBatteryMeanVoltage > RAW_VOLTAGE_ON_BATTERY_LOW)) {
        alarmController.notDetectedAlarm(RCM_SW_12);
    } else {
        // This is an hysteresis, so do nothing here
    }
}

void batteryLoop(uint32_t p_cycleNumber) {
    updateBatterySample();
    updateBatteryState(p_cycleNumber);
}

// cppcheck-suppress unusedFunction
uint32_t getBatteryLevel() { return rawBatteryMeanVoltage * RAW_BATTERY_MULTIPLIER; }

// cppcheck-suppress unusedFunction
uint32_t getBatteryLevelX10() { return rawBatteryMeanVoltage * (10.0 * RAW_BATTERY_MULTIPLIER); }

// cppcheck-suppress unusedFunction
uint32_t getBatteryLevelX100() { return rawBatteryMeanVoltage * (100.0 * RAW_BATTERY_MULTIPLIER); }

// cppcheck-suppress unusedFunction
bool isBatteryVeryLow() {
    return rawBatteryMeanVoltage < RAW_VOLTAGE_ON_BATTERY_NOT_STARTING_THRESHOLD;
}

bool isBatteryDeepDischarged() {
    return rawBatteryMeanVoltage < RAW_VOLTAGE_ON_BATTERY_STOP_THRESHOLD;
}

// cppcheck-suppress unusedFunction
bool isMainsConnected() { return (!isRunningOnBattery); }
