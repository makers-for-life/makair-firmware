/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file keyboard.cpp
 * @brief Buttons related functions
 *
 * This relies on the OneButton library (https://github.com/mathertel/OneButton).
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/keyboard.h"

// External
#include "../includes/config.h"
#include <OneButton.h>

// Internal
#include "../includes/activation.h"
#include "../includes/buzzer.h"
#include "../includes/debug.h"
#include "../includes/main_controller.h"
#include "../includes/parameters.h"

// INITIALISATION =============================================================

static OneButton buttonAlarmOff(PIN_BTN_ALARM_OFF, false, false);
static OneButton buttonStart(PIN_BTN_START, false, false);
static OneButton buttonStop(PIN_BTN_STOP, false, false);

// FUNCTIONS ==================================================================

/// Handler of the button to increase the crete pressure
void onPeakPressureIncrease() { mainController.onPeakPressureIncrease(); }

/// Handler of the button to decrease the crete pressure
void onPeakPressureDecrease() { mainController.onPeakPressureDecrease(); }

/// Handler of the button to increase the plateau pressure
void onPlateauPressureIncrease() { mainController.onPlateauPressureIncrease(); }

/// Handler of the button to decrease the plateau pressure
void onPlateauPressureDecrease() { mainController.onPlateauPressureDecrease(); }

/// Handler of the button to increase the PEP pressure
void onPeepPressureIncrease() { mainController.onPeepPressureIncrease(); }

/// Handler of the button to decrease the PEP pressure
void onPeepPressureDecrease() { mainController.onPeepPressureDecrease(); }

/// Handler of the button to increase the number of breathing cycles
void onCycleIncrease() { mainController.onCycleIncrease(); }

/// Handler of the button to decrease the number of breathing cycles
void onCycleDecrease() { mainController.onCycleDecrease(); }

/// Handler of the button to stop alarm
void onAlarmOff() { alarmController.snooze(); }

/// Handler of the button to start
void onStart() { activationController.onStartButton(); }

/// Handler of the button to stop
void onStop() { activationController.onStopButton(); }

void initKeyboard() {
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

    buttonAlarmOff.attachClick(onAlarmOff);
    buttonStart.attachClick(onStart);
    buttonStop.attachClick(onStop);
}

// current powered column of the matrix keyboard
uint16_t scanMatrixCurrentColumn = 1;
uint16_t scanMatrixCounterC1R1 = 0;
uint16_t scanMatrixCounterC1R2 = 0;
uint16_t scanMatrixCounterC1R3 = 0;
uint16_t scanMatrixCounterC2R1 = 0;
uint16_t scanMatrixCounterC2R2 = 0;
uint16_t scanMatrixCounterC2R3 = 0;
uint16_t scanMatrixCounterC3R1 = 0;
uint16_t scanMatrixCounterC3R2 = 0;
uint16_t scanMatrixCounterC3R3 = 0;
#define SM_DEBOUNCE 2u  // number of debounce ticks before trigger an action
#define SM_REPEAT 20u   // number of ticks before starting continuous press action repeat
#define SM_PERIOD 15u   // period of action repeat in case of a continuous press

// fast solution: no hardware timer, no interrupt priority or atomicity issue. Close to Arduino
// philosophy.
void scanMatrixLoop() {
    if (1u == scanMatrixCurrentColumn) {
        // Increase counter for each column and row
        if (HIGH == digitalRead(PIN_IN_ROW1)) {
            scanMatrixCounterC1R1++;
        } else {
            scanMatrixCounterC1R1 = 0;
        }
        if (HIGH == digitalRead(PIN_IN_ROW2)) {
            scanMatrixCounterC1R2++;
        } else {
            scanMatrixCounterC1R2 = 0;
        }
        if (HIGH == digitalRead(PIN_IN_ROW3)) {
            scanMatrixCounterC1R3++;
        } else {
            scanMatrixCounterC1R3 = 0;
        }
        // first click (after debounce ticks) or
        // later clicks if continuous press trigger action
        if ((SM_DEBOUNCE == scanMatrixCounterC1R1)
            || ((scanMatrixCounterC1R1 >= SM_REPEAT)
                && (0u == (scanMatrixCounterC1R1 - SM_REPEAT) % SM_PERIOD))) {
            onPeakPressureIncrease();
        }
        if ((SM_DEBOUNCE == scanMatrixCounterC1R2)
            || ((scanMatrixCounterC1R2 >= SM_REPEAT)
                && (0u == (scanMatrixCounterC1R2 - SM_REPEAT) % SM_PERIOD))) {
            onPeakPressureDecrease();
        }
        if ((SM_DEBOUNCE == scanMatrixCounterC1R3)
            || ((scanMatrixCounterC1R3 >= SM_REPEAT)
                && (0u == (scanMatrixCounterC1R3 - SM_REPEAT) % SM_PERIOD))) {
            onPlateauPressureIncrease();
        }
    } else if (2u == scanMatrixCurrentColumn) {
        if (HIGH == digitalRead(PIN_IN_ROW1)) {
            scanMatrixCounterC2R1++;
        } else {
            scanMatrixCounterC2R1 = 0;
        }
        if (HIGH == digitalRead(PIN_IN_ROW2)) {
            scanMatrixCounterC2R2++;
        } else {
            scanMatrixCounterC2R2 = 0;
        }
        if (HIGH == digitalRead(PIN_IN_ROW3)) {
            scanMatrixCounterC2R3++;
        } else {
            scanMatrixCounterC2R3 = 0;
        }
        // first click (after debounce ticks) or
        // later clicks if continuous press trigger action
        if ((SM_DEBOUNCE == scanMatrixCounterC2R1)
            || ((scanMatrixCounterC2R1 >= SM_REPEAT)
                && (0u == (scanMatrixCounterC2R1 - SM_REPEAT) % SM_PERIOD))) {
            onPlateauPressureDecrease();
        }
        if ((SM_DEBOUNCE == scanMatrixCounterC2R2)
            || ((scanMatrixCounterC2R2 >= SM_REPEAT)
                && (0u == (scanMatrixCounterC2R2 - SM_REPEAT) % SM_PERIOD))) {
            onPeepPressureIncrease();
        }
        if ((SM_DEBOUNCE == scanMatrixCounterC2R3)
            || ((scanMatrixCounterC2R3 >= SM_REPEAT)
                && (0u == (scanMatrixCounterC2R3 - SM_REPEAT) % SM_PERIOD))) {
            onPeepPressureDecrease();
        }
    } else if (3u == scanMatrixCurrentColumn) {
        if (HIGH == digitalRead(PIN_IN_ROW1)) {
            scanMatrixCounterC3R1++;
        } else {
            scanMatrixCounterC3R1 = 0;
        }
        if (HIGH == digitalRead(PIN_IN_ROW2)) {
            scanMatrixCounterC3R2++;
        } else {
            scanMatrixCounterC3R2 = 0;
        }
        if (HIGH == digitalRead(PIN_IN_ROW3)) {
            scanMatrixCounterC3R3++;
        } else {
            scanMatrixCounterC3R3 = 0;
        }
        // first click (after debounce ticks) or
        // later clicks if continuous press trigger action
        if ((SM_DEBOUNCE == scanMatrixCounterC3R1)
            || ((scanMatrixCounterC3R1 >= SM_REPEAT)
                && (0u == (scanMatrixCounterC3R1 - SM_REPEAT) % SM_PERIOD))) {
            onCycleIncrease();
        }
        if ((SM_DEBOUNCE == scanMatrixCounterC3R2)
            || ((scanMatrixCounterC3R2 >= SM_REPEAT)
                && (0u == (scanMatrixCounterC3R2 - SM_REPEAT) % SM_PERIOD))) {
            onCycleDecrease();
        }
        // there is no button on col3 x row3
    } else {
        // Do nothing
    }

    // next column
    scanMatrixCurrentColumn++;
    if (4u == scanMatrixCurrentColumn) {
        scanMatrixCurrentColumn = 1;
    }
    digitalWrite(PIN_OUT_COL1, (1u == scanMatrixCurrentColumn) ? HIGH : LOW);
    digitalWrite(PIN_OUT_COL2, (2u == scanMatrixCurrentColumn) ? HIGH : LOW);
    digitalWrite(PIN_OUT_COL3, (3u == scanMatrixCurrentColumn) ? HIGH : LOW);
}

void keyboardLoop() {
    scanMatrixLoop();
    buttonAlarmOff.tick();
    buttonStart.tick();
    buttonStop.tick();
}

// cppcheck-suppress unusedFunction
void calibrateButtons() {}
