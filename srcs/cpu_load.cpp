/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file cpu_load.cpp
 * @brief A CPU load estimation
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Associated header
#include "../includes/cpu_load.h"

#include "Arduino.h"

// INITIALISATION =============================================================

// idleCyclesCount will be incremented in the loop, and read in the systick IRQ
// If the systick IRQ set idleCyclesCount to 0, there is no guarantee it returns to 0,
// because chances are high to be in the middle of the increment operation, which is not atomic.
volatile uint32_t idleCyclesCount = 0;

// Computed CPU load in percent
volatile uint8_t cpuLoadPercent = 0;

// As there is no Free RTOS here, we can use is for CPU load estimator
uint16_t cpuLoadTimeCount = TIME_CPU_CYCLE_RESET;

uint32_t cpuLoadLatestCycleCount = 0;

// When nothing is enabled, the increment in the loop is done xx /second
// To measure it, remove all the code in setup except Serial initialization,
// and uncomment Serial output below
#define CPU_MAX_LOOP_PER_SECOND 8327007u

// FUNCTIONS ==================================================================

void countIdleCycle(void) { idleCyclesCount++; }

uint8_t readCpuLoadPercent(void) { return cpuLoadPercent; }

void cpuLoadCallback(void) {
    cpuLoadTimeCount--;
    if (cpuLoadTimeCount == 0u) {
        cpuLoadTimeCount = TIME_CPU_CYCLE_RESET;
        // Overflow won't be a problem, same type
        uint32_t delta = idleCyclesCount - cpuLoadLatestCycleCount;
        cpuLoadLatestCycleCount = idleCyclesCount;
        if (delta > CPU_MAX_LOOP_PER_SECOND) {
            delta = CPU_MAX_LOOP_PER_SECOND;
        }
        cpuLoadPercent = static_cast<uint8_t>(100u - (100u * delta) / CPU_MAX_LOOP_PER_SECOND);

        // Uncomment to tune CPU_MAX_LOOP_PER_SECOND value
        // Serial.println(delta);
    }
}

extern "C" {
// This is the highest priority 1 ms callback
// cppcheck-suppress unusedFunction
void osSystickHandler() { cpuLoadCallback(); }
}
