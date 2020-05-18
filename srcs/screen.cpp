/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file screen.cpp
 * @brief Display and LCD screen related functions
 *
 * This relies on the LiquidCrystal library (https://github.com/arduino-libraries/LiquidCrystal).
 * LCD screen must have 4 lines of 20 characters.
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/screen.h"

// Internal
#include "../includes/parameters.h"

// INITIALISATION =============================================================

/// Number of alarm codes to display on screen at most
static const uint8_t MAX_ALARMS_DISPLAYED = 4;

/// Text to display at the third line of the screen when no alarm is triggered
static const char* NO_ALARM_LINE = "PEAK  PLAT  PEEP    ";

/// Static label to display at the begining of the third line of the screen when at least one alarm
/// is triggered
static const char* ALARM_LINE = "Alarm:              ";

/// Position of the first alarm code in the third line of the screen
static const int ALARMS_CODE_POS = 6;

/// Instance of the screen controller
LiquidCrystal
    screen(PIN_LCD_RS, PIN_LCD_RW, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// FUNCTIONS ==================================================================

void startScreen() {
    screen.begin(SCREEN_LINE_LENGTH, SCREEN_LINE_NUMBER);
    screen.setCursor(0, 0);
    screen.print("Initialization      ");
    screen.setCursor(0, 1);
    screen.print(VERSION);
}

void resetScreen() { screen.clear(); }

// cppcheck-suppress unusedFunction
void displayCurrentVolume(int32_t volumeMassFlow, uint16_t cyclesPerMinute) {
    screen.setCursor(0, 0);

    char message[SCREEN_LINE_LENGTH + 1];

    (void)snprintf(message, SCREEN_LINE_LENGTH + 1, "Cpm:%2u      %4dml", cyclesPerMinute,
                   volumeMassFlow);

    screen.print(message);
}

// cppcheck-suppress unusedFunction
void displayCurrentPressure(uint16_t pressure, uint16_t cyclesPerMinute) {
    screen.setCursor(0, 0);

    char message[SCREEN_LINE_LENGTH + 1];

    (void)snprintf(message, SCREEN_LINE_LENGTH + 1, "Pressure:%2u    %2ucpm",
                   convertAndRound(pressure), cyclesPerMinute);

    screen.print(message);
}

void displayCurrentSettings(uint16_t peakPressureMax,
                            uint16_t plateauPressureMax,
                            uint16_t peepMin) {
    // cppcheck-suppress misra-c2012-12.3
    screen.setCursor(0, 1);

    char message[SCREEN_LINE_LENGTH + 1];

    (void)snprintf(message, SCREEN_LINE_LENGTH + 1, "%2u    %2u    %2u  set ",
                   convertAndRound(peakPressureMax), convertAndRound(plateauPressureMax),
                   convertAndRound(peepMin));

    screen.print(message);
}

void displayCurrentInformation(uint16_t peakPressure, uint16_t plateauPressure, uint16_t peep) {
    // cppcheck-suppress misra-c2012-12.3 ; call to unknown external: screen.setCursor
    screen.setCursor(0, 3);
    char message[SCREEN_LINE_LENGTH + 1];

    // If plateau was not detected
    if (plateauPressure == UINT16_MAX) {
        (void)snprintf(message, SCREEN_LINE_LENGTH + 1, "%2u     ?    %2u  meas",
                       convertAndRound(peakPressure), convertAndRound(peep));
    } else {
        (void)snprintf(message, SCREEN_LINE_LENGTH + 1, "%2u    %2u    %2u  meas",
                       convertAndRound(peakPressure), convertAndRound(plateauPressure),
                       convertAndRound(peep));
    }

    screen.print(message);
}

static uint8_t prevNbAlarmToPrint = 255;
static uint8_t prevAlarmCodes[MAX_ALARMS_DISPLAYED] = {0};
static bool clearCache = false;

/// Check whether triggered alarms are already displayed on screen or not
static bool hasAlarmInformationChanged(uint8_t p_alarmCodes[], uint8_t p_nbTriggeredAlarms) {
    bool hasChanged = false;

    if (clearCache == true) {
        clearCache = false;
        hasChanged = true;
    } else {
        uint8_t nbAlarmToPrint = min(MAX_ALARMS_DISPLAYED, p_nbTriggeredAlarms);

        if (nbAlarmToPrint != prevNbAlarmToPrint) {
            hasChanged = true;
        } else {
            for (uint8_t i = 0; i < nbAlarmToPrint; ++i) {
                if (p_alarmCodes[i] != prevAlarmCodes[i]) {
                    hasChanged = true;
                    break;
                }
            }
        }

        if (hasChanged) {
            prevNbAlarmToPrint = nbAlarmToPrint;
            for (uint8_t i = 0; i < nbAlarmToPrint; ++i) {
                prevAlarmCodes[i] = p_alarmCodes[i];
            }
        }
    }
    return hasChanged;
}

void clearAlarmDisplayCache() { clearCache = true; }

void displayAlarmInformation(uint8_t p_alarmCodes[], uint8_t p_nbTriggeredAlarms) {
    // WARNING There is a risk of data not being displayed as expected
    // if the line is overwritten somewhere else in the code.
    if (hasAlarmInformationChanged(p_alarmCodes, p_nbTriggeredAlarms)) {
        if (p_nbTriggeredAlarms == 0u) {
            screen.setCursor(0, 2);
            screen.print(NO_ALARM_LINE);
        } else {
            uint8_t nbAlarmToPrint = min(MAX_ALARMS_DISPLAYED, p_nbTriggeredAlarms);

            // +1 for trailing NULL char
            char buf[SCREEN_LINE_LENGTH + 1];

            // Write beginning of line
            (void)strncpy(buf, ALARM_LINE, ALARMS_CODE_POS);

            // Write alarm codes
            int pos = ALARMS_CODE_POS;
            for (uint8_t i = 0; i < nbAlarmToPrint; i++) {
                int spaceLeft = SCREEN_LINE_LENGTH - pos;
                // + 1 for the trailing NULL char
                int n = snprintf(&buf[pos], spaceLeft + 1, " %u", p_alarmCodes[i]);
                if ((n < 0) || (n > spaceLeft)) {
                    break;  // Error or no space left in buffer
                }
                pos += n;
            }

            // Fill the end of the line with spaces
            (void)strncpy(&buf[pos], &ALARM_LINE[pos], SCREEN_LINE_LENGTH - pos);

            // Make sure string is NULL terminated
            buf[SCREEN_LINE_LENGTH] = '\0';

            screen.setCursor(0, 2);
            screen.print(buf);
        }
    }
}

void displayMachineStopped(void) {
    screen.setCursor(0, 3);
    screen.print("Press start to begin");
}

uint16_t convertAndRound(uint16_t pressure) {
    uint16_t result;
    uint16_t lastDigit = pressure % 10u;

    if (lastDigit < 5u) {
        result = (pressure / 10u);
    } else {
        result = (pressure / 10u) + 1u;
    }

    return result;
}
