/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file screen.h
 * @brief Display and LCD screen related functions
 *
 * This relies on the LiquidCrystal library (https://github.com/arduino-libraries/LiquidCrystal).
 * LCD screen must have 4 lines of 20 characters.
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// External
#include <LiquidCrystal.h>

// Internal
#include "../includes/parameters.h"

// INITIALISATION =============================================================

/// Instance of the screen controller
extern LiquidCrystal screen;

// FUNCTIONS ==================================================================

/**
 * Start the screen
 *
 * @warning It must be called once to be able to use the screen
 */
void startScreen();

/// Erase everything that is on the screen
void resetScreen();

/**
 * Display the current step of the breathing
 *
 * @param pressure            The current pressure [mmH2O]
 * @param cyclesPerMinute     Next desired number of cycles per minute
 */
void displayCurrentPressure(uint16_t pressure, uint16_t cyclesPerMinute);

/**
 * Display the current step of the breathing
 *
 * @param volumeMassFlow      The number of liter breathed in this cycle
 * @param cyclesPerMinute     Next desired number of cycles per minute
 */
void displayCurrentVolume(int32_t volumeMassFlow, uint16_t cyclesPerMinute);

/**
 * Display the current settings
 *
 * @param peakPressureMax      PeakPressureMax [mmH2O]
 * @param plateauPressureMax   Next maximal plateau pressure [mmH2O]
 * @param peepMin              Next desired Positive End Expiratory Pressure (PEEP) [mmH2O]
 */
void displayCurrentSettings(uint16_t peakPressureMax,
                            uint16_t plateauPressureMax,
                            uint16_t peepMin);

/**
 * Display relevant values from the ongoing cycle
 *
 * @param peakPressure     The peak pressure [mmH2O]
 * @param plateauPressure  The plateau pressure [mmH2O]
 * @param peep             The Positive End Expiratory Pressure (PEEP) [mmH2O]
 */
void displayCurrentInformation(uint16_t peakPressure, uint16_t plateauPressure, uint16_t peep);

/**
 * Display triggered alarm codes
 *
 * @param p_alarmCodes          List of alarm codes to display
 * @param p_nbTriggeredAlarms   Number of triggered alarms
 */
void displayAlarmInformation(uint8_t p_alarmCodes[], uint8_t p_nbTriggeredAlarms);

/**
 * Display a message when the machine is stopped
 */
void displayMachineStopped(void);

/// Force clear the alarm display cache
void clearAlarmDisplayCache();

/**
 * Convert and round a pressure value
 *
 * @param pressure Pressure in mmH2O
 * @return Rounded pressure in cmH2O
 */
uint16_t convertAndRound(uint16_t pressure);
