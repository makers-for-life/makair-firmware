/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file keyboard.cpp
 * @brief Analog buttons related functions
 *
 * This relies on the Analog Buttons library (https://github.com/rlogiacco/AnalogButtons).
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/keyboard.h"

// External
#include "../includes/config.h"
#if HARDWARE_VERSION == 1
#include <AnalogButtons.h>
#endif
#include <OneButton.h>

// Internal
#include "../includes/activation.h"
#include "../includes/buzzer.h"
#include "../includes/debug.h"
#include "../includes/parameters.h"
#include "../includes/pressure_controller.h"

// INITIALISATION =============================================================

#if HARDWARE_VERSION == 1
/// Abstraction to handle buttons connected to one analog pin through a voltage divider
static AnalogButtons analogButtons(PIN_CONTROL_BUTTONS, INPUT, 5, 30);
#elif HARDWARE_VERSION == 2
static OneButton buttonPeakPressureIncrease(PIN_BTN_PEAK_PRESSURE_INCREASE, false, false);
static OneButton buttonPeakPressureDecrease(PIN_BTN_PEAK_PRESSURE_DECREASE, false, false);
static OneButton buttonPlateauPressureIncrease(PIN_BTN_PLATEAU_PRESSURE_INCREASE, false, false);
static OneButton buttonPlateauPressureDecrease(PIN_BTN_PLATEAU_PRESSURE_DECREASE, false, false);
static OneButton buttonPeepPressureIncrease(PIN_BTN_PEEP_PRESSURE_INCREASE, false, false);
static OneButton buttonPeepPressureDecrease(PIN_BTN_PEEP_PRESSURE_DECREASE, false, false);
static OneButton buttonCycleIncrease(PIN_BTN_CYCLE_INCREASE, false, false);
static OneButton buttonCycleDecrease(PIN_BTN_CYCLE_DECREASE, false, false);
#endif

static OneButton buttonAlarmOff(PIN_BTN_ALARM_OFF, false, false);
static OneButton buttonStart(PIN_BTN_START, false, false);
static OneButton buttonStop(PIN_BTN_STOP, false, false);

/*-----------------------------------------------------------------------------
 * Button handlers
 */

/// Handler of the button to increase the crete pressure
void onPeakPressureIncrease() { pController.onPeakPressureIncrease(DEFAULT_PEAK_PRESSURE_DELTA); }

/// Handler of the button to decrease the crete pressure
void onPeakPressureDecrease() { pController.onPeakPressureDecrease(DEFAULT_PEAK_PRESSURE_DELTA); }

/// Handler of the button to increase the plateau pressure
void onPlateauPressureIncrease() { pController.onPlateauPressureIncrease(); }

/// Handler of the button to decrease the plateau pressure
void onPlateauPressureDecrease() { pController.onPlateauPressureDecrease(); }

/// Handler of the button to increase the PEP pressure
void onPeepPressureIncrease() { pController.onPeepPressureIncrease(); }

/// Handler of the button to decrease the PEP pressure
void onPeepPressureDecrease() { pController.onPeepPressureDecrease(); }

/// Handler of the button to increase the number of breathing cycles
void onCycleIncrease() { pController.onCycleIncrease(); }

/// Handler of the button to decrease the number of breathing cycles
void onCycleDecrease() { pController.onCycleDecrease(); }

/// Handler of the button to stop alarm
void onAlarmOff() { alarmController.snooze(); }

/// Handler of the button to start
void onStart() { activationController.onStartButton(); }

/// Handler of the button to stop
void onStop() { activationController.onStopButton(); }

/**
 * @name Bindings between analog levels and handlers
 */
///@{
#if HARDWARE_VERSION == 1
Button buttonPeakPressureIncrease(VOLTAGE_BUTTON_PEAK_PRESSURE_INCREASE, &onPeakPressureIncrease);
Button buttonPeakPressureDecrease(VOLTAGE_BUTTON_PEAK_PRESSURE_DECREASE, &onPeakPressureDecrease);
Button buttonPlateauPressureIncrease(VOLTAGE_BUTTON_PLATEAU_PRESSURE_INCREASE,
                                     &onPlateauPressureIncrease);
Button buttonPlateauPressureDecrease(VOLTAGE_BUTTON_PLATEAU_PRESSURE_DECREASE,
                                     &onPlateauPressureDecrease);
Button buttonPeepPressureIncrease(VOLTAGE_BUTTON_PEEP_PRESSURE_INCREASE, &onPeepPressureIncrease);
Button buttonPeepPressureDecrease(VOLTAGE_BUTTON_PEEP_PRESSURE_DECREASE, &onPeepPressureDecrease);
Button buttonCycleIncrease(VOLTAGE_BUTTON_CYCLE_INCREASE, &onCycleIncrease);
Button buttonCycleDecrease(VOLTAGE_BUTTON_CYCLE_DECREASE, &onCycleDecrease);
#endif
///@}

void initKeyboard() {
#if HARDWARE_VERSION == 1
    analogButtons.add(buttonPeakPressureIncrease);
    analogButtons.add(buttonPeakPressureDecrease);
    analogButtons.add(buttonPlateauPressureIncrease);
    analogButtons.add(buttonPlateauPressureDecrease);
    analogButtons.add(buttonPeepPressureIncrease);
    analogButtons.add(buttonPeepPressureDecrease);
    analogButtons.add(buttonCycleIncrease);
    analogButtons.add(buttonCycleDecrease);
#elif HARDWARE_VERSION == 2
    buttonPeakPressureIncrease.attachClick(onPeakPressureIncrease);
    buttonPeakPressureDecrease.attachClick(onPeakPressureDecrease);
    buttonPlateauPressureIncrease.attachClick(onPlateauPressureIncrease);
    buttonPlateauPressureDecrease.attachClick(onPlateauPressureDecrease);
    buttonPeepPressureIncrease.attachClick(onPeepPressureIncrease);
    buttonPeepPressureDecrease.attachClick(onPeepPressureDecrease);
    buttonCycleIncrease.attachClick(onCycleIncrease);
    buttonCycleDecrease.attachClick(onCycleDecrease);
#elif HARDWARE_VERSION == 3
    // define the 3x3 matrix keyboard input and output
    pinMode(PIN_OUT_COL1, OUTPUT);
    pinMode(PIN_OUT_COL2, OUTPUT);
    pinMode(PIN_OUT_COL3, OUTPUT);
    digitalWrite(PIN_OUT_COL1, LOW);
    digitalWrite(PIN_OUT_COL2, LOW);
    digitalWrite(PIN_OUT_COL3, LOW);
    pinMode(PIN_IN_ROW1, INPUT);
    pinMode(PIN_IN_ROW2, INPUT);
    pinMode(PIN_IN_ROW3, INPUT);
#endif

    buttonAlarmOff.attachClick(onAlarmOff);
    buttonStart.attachClick(onStart);
    buttonStop.attachClick(onStop);
}

#if HARDWARE_VERSION == 3
// current powered column of the matrix keyboard.
int scanMatrixCurrentColumn = 1;
// fast solution : no debouncing, no hardware timer. Close to Arduino philosophy.
void scanMatrixLoop() {
    if (1 == scanMatrixCurrentColumn) {
        if (HIGH == digitalRead(PIN_IN_ROW1)) {
            onPeakPressureIncrease();
        }
        if (HIGH == digitalRead(PIN_IN_ROW2)) {
            onPeakPressureDecrease();
        }
        if (HIGH == digitalRead(PIN_IN_ROW3)) {
            onPlateauPressureIncrease();
        }
    } else if (2 == scanMatrixCurrentColumn) {
        if (HIGH == digitalRead(PIN_IN_ROW1)) {
            onPlateauPressureDecrease();
        }
        if (HIGH == digitalRead(PIN_IN_ROW2)) {
            onPeepPressureIncrease();
        }
        if (HIGH == digitalRead(PIN_IN_ROW3)) {
            onPeepPressureDecrease();
        }
    } else if (3 == scanMatrixCurrentColumn) {
        if (HIGH == digitalRead(PIN_IN_ROW1)) {
            onCycleIncrease();
        }
        if (HIGH == digitalRead(PIN_IN_ROW2)) {
            onCycleDecrease();
        }
        // there is no button on col3 x row3
    }

    // next column
    scanMatrixCurrentColumn++;
    if (4 == scanMatrixCurrentColumn) {
        scanMatrixCurrentColumn = 1;
    }
    digitalWrite(PIN_OUT_COL1, 1 == scanMatrixCurrentColumn ? HIGH : LOW);
    digitalWrite(PIN_OUT_COL2, 2 == scanMatrixCurrentColumn ? HIGH : LOW);
    digitalWrite(PIN_OUT_COL3, 3 == scanMatrixCurrentColumn ? HIGH : LOW);
}
#endif

void keyboardLoop() {
#if HARDWARE_VERSION == 1
    analogButtons.check();
#elif HARDWARE_VERSION == 2
    buttonPeakPressureIncrease.tick();
    buttonPeakPressureDecrease.tick();
    buttonPlateauPressureIncrease.tick();
    buttonPlateauPressureDecrease.tick();
    buttonPeepPressureIncrease.tick();
    buttonPeepPressureDecrease.tick();
    buttonCycleIncrease.tick();
    buttonCycleDecrease.tick();
#elif HARDWARE_VERSION == 3
    scanMatrixLoop();
#endif
    buttonAlarmOff.tick();
    buttonStart.tick();
    buttonStop.tick();
}

// cppcheck-suppress unusedFunction
void calibrateButtons() {
#if HARDWARE_VERSION == 1
    uint16_t value = analogRead(PIN_CONTROL_BUTTONS);
    Serial.println(value);
    delay(250);
#endif
}
