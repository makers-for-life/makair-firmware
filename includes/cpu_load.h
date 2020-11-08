/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file cpu_load.h
 * @brief A CPU load estimation
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

#include <stdint.h>

// INITIALISATION =============================================================

/// Duration in ms after which the CPU cycle count is reset and the result is stored
#define TIME_CPU_CYCLE_RESET 1000u

// FUNCTIONS ==================================================================

/**
 * Increment the idle cycles counter
 *
 * @note This must be called in the main loop() with nothing else in the loop
 */
#define COUNT_IDLE_CYCLE idleCyclesCount++

/**
 * Get the value of the CPU load
 *
 * @return CPU load in percent
 * @note If this raises to 100%, there is some blocking code somewhere
 */
uint8_t readCpuLoadPercent(void);

/**
 * Access to idleCyclesCount needed by COUNT_IDLE_CYCLE macro
 */
extern volatile uint32_t idleCyclesCount;
