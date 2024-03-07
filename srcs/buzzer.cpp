/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file buzzer.cpp
 * @brief Buzzer related functions
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Externals
#include "Arduino.h"

/// Internals

#include "../includes/buzzer.h"
#include "../includes/buzzer_control.h"
#include "../includes/parameters.h"

// PROGRAM =====================================================================

/**
 * @name Definition of bips durations
 * 10 ticks = 1 ms
 * @warning it is not possible to have 1 tick = 1 ms because prescaler is 16 bits and input
 * STM32F411: frequency is 100Mhz
 * STM32F401: frequency is 84Mhz
 *
 * As waiting times defined below are greater than 2^16, the timer must be a 32 bits timer.
 */
///@{
#define TIMER_TICK_PER_MS 10
#define BIP (100 * TIMER_TICK_PER_MS)
#define BIP_PAUSE BIP
#define BEEEEP (250 * TIMER_TICK_PER_MS)
#define BEEEEP_PAUSE BEEEEP
#define PAUSE_120S (120 * 1000 * TIMER_TICK_PER_MS)
#define PAUSE_20S (20 * 1000 * TIMER_TICK_PER_MS)
#define PAUSE_10S (10 * 1000 * TIMER_TICK_PER_MS)
#define PAUSE_1S (1 * 1000 * TIMER_TICK_PER_MS)
///@}

#define BZ_OFF (0u)
#define BZ_ON (1u)

/// High priority alarm buzzer pattern size
#define BUZZER_HIGH_PRIO_SIZE 40

/// High priority alarm buzzer pattern definition, composed of
/// multiple couple of states (Actif/Inactif) and duration (miliseconds)
const uint32_t Buzzer_High_Prio[BUZZER_HIGH_PRIO_SIZE] = {
    BZ_ON,  BIP,       BZ_OFF, BIP_PAUSE, BZ_ON,  BIP,       BZ_OFF, BIP_PAUSE, BZ_ON,  BIP,
    BZ_OFF, BIP_PAUSE, BZ_ON,  BIP,       BZ_OFF, BIP_PAUSE, BZ_ON,  BEEEEP,    BZ_OFF, PAUSE_1S,
    BZ_ON,  BIP,       BZ_OFF, BIP_PAUSE, BZ_ON,  BIP,       BZ_OFF, BIP_PAUSE, BZ_ON,  BIP,
    BZ_OFF, BIP_PAUSE, BZ_ON,  BIP,       BZ_OFF, BIP_PAUSE, BZ_ON,  BEEEEP,    BZ_OFF, PAUSE_10S};

/// Medium priority alarm buzzer pattern size
#define BUZZER_MEDIUM_PRIO_SIZE 8

/// Medium priority alarm buzzer pattern definition, composed of
/// multiple couple of states (Actif/Inactif) and duration (miliseconds)
const uint32_t Buzzer_Medium_Prio[BUZZER_MEDIUM_PRIO_SIZE] = {BZ_ON, BEEEEP, BZ_OFF, BEEEEP_PAUSE,
                                                              BZ_ON, BEEEEP, BZ_OFF, PAUSE_20S};

/// Low priority alarm buzzer pattern size
#define BUZZER_LOW_PRIO_SIZE 8

/// Low priority alarm buzzer pattern definition, composed of
/// multiple couple of states (Actif/Inactif) and duration (miliseconds)
const uint32_t Buzzer_Low_Prio[BUZZER_LOW_PRIO_SIZE] = {BZ_ON, BIP, BZ_OFF, BIP_PAUSE,
                                                        BZ_ON, BIP, BZ_OFF, BIP_PAUSE};

/// Boot buzzer pattern size
#define BUZZER_BOOT_SIZE 8

/// Boot buzzer pattern definition, composed of multiple couple of states (Actif/Inactif) and
/// duration (miliseconds)
const uint32_t Buzzer_Boot[BUZZER_BOOT_SIZE] = {BZ_ON, BEEEEP, BZ_OFF, BEEEEP_PAUSE,
                                                BZ_ON, BEEEEP, BZ_OFF, BEEEEP_PAUSE};

// INITIALISATION =============================================================

const uint32_t* Active_Buzzer = nullptr;
uint32_t Active_Buzzer_Index = 0;
uint32_t Active_Buzzer_Size = 2;
bool Active_Buzzer_Repeat = false;
bool Active_Buzzer_Has_Begun = false;
bool Buzzer_Muted = false;

HardwareTimer* BuzzerTim;
uint32_t BuzzerTimerChannel;

// FUNCTIONS ==================================================================

/**
 * When timer period expires, switch to next state in the pattern of the buzzer
 * @note API update since version 1.9.0 of Arduino_Core_STM32
 */
#if (STM32_CORE_VERSION < 0x01090000)
// cppcheck-suppress misra-c2012-2.7 ; valid unused parameter
void Update_IT_callback(HardwareTimer*)  // NOLINT(readability/casting)
#else
void Update_IT_callback(void)
#endif
{
    if (Buzzer_Muted == true) {
        // If the buzzer was muted, then we must resume the previous alarm
        Buzzer_Resume();
    } else if ((Active_Buzzer_Index == 0u) && (Active_Buzzer_Repeat == false)
               && (Active_Buzzer_Has_Begun == true)) {
        // If we are at start of pattern, check for repeating mode
        BuzzerTim->pause();
        BuzzerControl_Off();
    } else {
        // Previous state is finished, switch to next one
        if (Active_Buzzer[Active_Buzzer_Index] == BZ_ON) {
            BuzzerControl_On();
        } else {
            BuzzerControl_Off();
        }
        BuzzerTim->setOverflow(Active_Buzzer[Active_Buzzer_Index + 1u], TICK_FORMAT);
        Active_Buzzer_Index = (Active_Buzzer_Index + 2u) % Active_Buzzer_Size;
        Active_Buzzer_Has_Begun = true;
    }
}

void Buzzer_Init() {
    // Buzzer HardwareTimer object creation
    BuzzerTim = new HardwareTimer(BUZZER_TIMER);

    BuzzerControl_Off();
    // CPU Clock down to 10 kHz
    BuzzerTim->setPrescaleFactor((BuzzerTim->getTimerClkFreq() / (TIMER_TICK_PER_MS * 1000)));
    BuzzerTim->setOverflow(1);  // don't care right now, timer is not started in init
    BuzzerTim->setMode(BUZZER_TIM_CHANNEL, TIMER_OUTPUT_COMPARE, NC);
    BuzzerTim->attachInterrupt(Update_IT_callback);
}

void Buzzer_Start(const uint32_t* Buzzer, uint32_t Size, bool RepeatBuzzer) {
    BuzzerTim->setCount(0);
    Active_Buzzer = Buzzer;
    Active_Buzzer_Index = 0;
    Active_Buzzer_Size = Size;
    Active_Buzzer_Repeat = RepeatBuzzer;
    Active_Buzzer_Has_Begun = false;
    Buzzer_Muted = false;

    BuzzerTim->setPreloadEnable(false);
    BuzzerTim->setOverflow(100, TICK_FORMAT);

    // Timer starts
    BuzzerTim->resume();
}

void Buzzer_Mute() {
    // If we are in Repeat and not muted, then an alarm is ringing
    if ((Active_Buzzer_Repeat == true) && (Buzzer_Muted == false)) {
        // Set the buzzer as muted
        Buzzer_Muted = true;
        // Reset the timer
        BuzzerTim->setCount(0);
        // Reset the index, so that we will restart after the mute period
        Active_Buzzer_Index = 0;

        // Configuration of mute pattern
        BuzzerControl_Off();
        BuzzerTim->setOverflow(PAUSE_120S, TICK_FORMAT);

        // Timer starts. Required to configure output on GPIO
        BuzzerTim->resume();
    }
}

void Buzzer_Resume() {
    BuzzerTim->setCount(0);
    Buzzer_Muted = false;
    Active_Buzzer_Index = 0;

    BuzzerTim->setOverflow(Active_Buzzer[Active_Buzzer_Index + 1u], TICK_FORMAT);

    // Timer starts. Required to configure output on GPIO
    BuzzerTim->resume();
}

void Buzzer_High_Prio_Start(void) {
     //Buzzer_Start(Buzzer_High_Prio, BUZZER_HIGH_PRIO_SIZE, true);
      }

void Buzzer_Medium_Prio_Start(void) {
    //Buzzer_Start(Buzzer_Medium_Prio, BUZZER_MEDIUM_PRIO_SIZE, true);
}

void Buzzer_Low_Prio_Start(void) {
    // Buzzer_Start(Buzzer_Low_Prio, BUZZER_LOW_PRIO_SIZE, false);
     }

void Buzzer_Boot_Start(void) {
    //Buzzer_Start(Buzzer_Boot, BUZZER_BOOT_SIZE, false);
    }

void Buzzer_Stop(void) {
    Active_Buzzer_Repeat = false;

    // Avoid unexpected infinite buzzing
    BuzzerControl_Off();
    BuzzerTim->pause();
}
