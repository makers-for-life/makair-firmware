/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file calibration.cpp
 * @brief Calibration of the ventilator
 *****************************************************************************/

#pragma once

#include "../includes/config.h"

// INCLUDES ==================================================================

// Internal
#include "../includes/blower.h"
#include "../includes/buzzer.h"
#include "../includes/calibration.h"
#include "../includes/keyboard.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/pressure.h"
#include "../includes/pressure_valve.h"
#include "../includes/screen.h"
#include "../includes/telemetry.h"

// External
#include "Arduino.h"

// PROGRAM =====================================================================

int32_t pressureOffsetSum;
uint32_t pressureOffsetCount;
int32_t minOffsetValue = 0;
int32_t maxOffsetValue = 0;
bool startButtonPressed = false;
bool calibationStarted = false;
bool calibrationValid = false;

void Calibration_Init() {
    // Restart calibration process when invalid
    while (calibrationValid == false) {
        delay(1000);
        resetScreen();
        calibationStarted = true;
        // RCM-SW-17 (Christmas tree at startup)
        Buzzer_Boot_Start();
        digitalWrite(PIN_LED_START, LED_START_ACTIVE);
        digitalWrite(PIN_LED_GREEN, LED_GREEN_ACTIVE);
        digitalWrite(PIN_LED_RED, LED_RED_ACTIVE);
        digitalWrite(PIN_LED_YELLOW, LED_YELLOW_ACTIVE);
        Calibration_Wait_Measure_Pressure(1000);
        digitalWrite(PIN_LED_START, LED_START_INACTIVE);
        digitalWrite(PIN_LED_GREEN, LED_GREEN_INACTIVE);
        digitalWrite(PIN_LED_RED, LED_RED_INACTIVE);
        digitalWrite(PIN_LED_YELLOW, LED_YELLOW_INACTIVE);
        Calibration_Wait_Measure_Pressure(1000);

        displayPatientMustBeUnplugged();
        Calibration_Wait_Measure_Pressure(2000);

        int32_t inspiratoryPressureSensorOffset = 0;
        resetScreen();
        if (pressureOffsetCount != 0u) {
            inspiratoryPressureSensorOffset =
                pressureOffsetSum / static_cast<int32_t>(pressureOffsetCount);
        } else {
            inspiratoryPressureSensorOffset = 0;
        }

        // Happens when patient is plugged at startup
        if (((maxOffsetValue - minOffsetValue) >= 10)
            || (inspiratoryPressureSensorOffset >= MAX_PRESSURE_OFFSET)) {
            // Invalid calibration
            calibrationValid = false;
            displayPressureOffsetUnstable(minOffsetValue, maxOffsetValue);
            Buzzer_High_Prio_Start();
            Calibration_Read_Keyboard();
        } else {
            calibrationValid = true;
            inspiratoryPressureSensor.setPressureSensorOffset(inspiratoryPressureSensorOffset);
        }

        if (calibrationValid) {
#ifdef MASS_FLOW_METER_ENABLED
            int32_t flowMeterFlowAtStarting = MFM_read_airflow();
            (void)MFM_calibrateZero();
#else
            int32_t flowMeterFlowAtStarting = 0;
#endif
            inspiratoryValve.open();
            inspiratoryValve.execute();
            expiratoryValve.open();
            expiratoryValve.execute();
            delay(500);
            blower.runSpeed(DEFAULT_BLOWER_SPEED);
            delay(1000);
#ifdef MASS_FLOW_METER_ENABLED
            int32_t flowMeterFlowWithBlowerOn = MFM_read_airflow();
#else
            int32_t flowMeterFlowWithBlowerOn = 30000;
#endif

            blower.stop();

            // Happens when flow meter fails
            if (((flowMeterFlowAtStarting < -1000) || (flowMeterFlowAtStarting > 1000)
                 || (flowMeterFlowWithBlowerOn < 20000) || (flowMeterFlowWithBlowerOn > 100000))) {
                // Invalid calibration
                calibrationValid = false;
                displayFlowMeterFail(flowMeterFlowAtStarting, flowMeterFlowWithBlowerOn);
                Buzzer_High_Prio_Start();
                Calibration_Read_Keyboard();
            } else {
                calibrationValid = true;
            }

            displayPressureOffset(inspiratoryPressureSensorOffset);
            delay(1000);
#ifdef MASS_FLOW_METER_ENABLED
            displayFlowMeterOffset(MFM_getOffset());
            delay(1000);
#endif

            // Send calibration fatal error to telemetry?
            if (calibrationValid == false) {
                // MFM reports an out-of-range value, it might not be connected
                if ((flowMeterFlowAtStarting < -1000) || (flowMeterFlowAtStarting > 1000)) {
                    // MFM failure (eg. not connected)
                    sendMassFlowMeterFatalError();
                } else {
                    // Other calibration errors
                    sendCalibrationFatalError(inspiratoryPressureSensorOffset, minOffsetValue, maxOffsetValue, flowMeterFlowAtStarting, flowMeterFlowWithBlowerOn);
                }
            }
        }
        // Reset values to default state
        calibationStarted = false;
        startButtonPressed = false;
    }
}

void Calibration_Wait_Measure_Pressure(uint16_t ms) {
    uint16_t start = millis();
    minOffsetValue = inspiratoryPressureSensor.read();
    maxOffsetValue = inspiratoryPressureSensor.read();
    pressureOffsetSum = 0;
    pressureOffsetCount = 0;

    // Open valves
    inspiratoryValve.close();
    inspiratoryValve.execute();
    expiratoryValve.open();
    expiratoryValve.execute();

    while ((millis() - start) < ms) {
        // Measure 1 pressure per ms we wait
        if ((millis() - start) > pressureOffsetCount) {
            int32_t pressureValue = inspiratoryPressureSensor.read();
            pressureOffsetSum += pressureValue;
            minOffsetValue = min(pressureValue, minOffsetValue);
            maxOffsetValue = max(pressureValue, maxOffsetValue);
            pressureOffsetCount++;
        }
        continue;
    }
}

void Calibration_Read_Keyboard() {
    while (startButtonPressed == false) {
        keyboardLoop();
        delay(100);
    }
}

void Calibration_Restart() { startButtonPressed = true; }

bool Calibration_Started() { return calibationStarted; }
