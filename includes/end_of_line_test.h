/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file end_of_line_test.h
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Internal

#include "../includes/blower.h"
#include "../includes/pressure_valve.h"

/**
 * This an arbitrary value allowing to check if test mode is active. A variable is initialized at
 * beginning of testMode with this value. Due to the fact this value is arbitrary and big, it is
 * highly impossible that a memory corruption randomly gives this value
 */
#define EOL_TEST_ACTIVE 0xa240183a

// ENUMS =================================================================

enum TestStep {
    START,
    SUPPLY_TO_EXPANDER_NOT_CONNECTED,
    CHECK_FAN,
    TEST_BAT_DEAD,
    BATTERY_DEEP_DISCHARGE,
    DISCONNECT_MAINS,
    CONNECT_MAINS,
    CHECK_BUZZER,
    // cppcheck-suppress misra-c2012-12.3
    CHECK_ALL_BUTTONS,
    CHECK_UI_SCREEN,
    // cppcheck-suppress misra-c2012-12.3
    PLUG_AIR_TEST_SYTEM,
    // cppcheck-suppress misra-c2012-12.3
    REACH_MAX_PRESSURE,
    // cppcheck-suppress misra-c2012-12.3
    MAX_PRESSURE_REACHED_OK,
    // cppcheck-suppress misra-c2012-12.3
    MAX_PRESSURE_NOT_REACHED,
    // cppcheck-suppress misra-c2012-12.3
    START_LEAK_MESURE,
    // cppcheck-suppress misra-c2012-12.3
    LEAK_IS_TOO_HIGH,
    // cppcheck-suppress misra-c2012-12.3
    REACH_NULL_PRESSURE,
    // cppcheck-suppress misra-c2012-12.3
    MIN_PRESSURE_NOT_REACHED,
    USER_CONFIRMATION_BEFORE_O2_TEST,
    // cppcheck-suppress misra-c2012-12.3
    START_O2_TEST,
    // cppcheck-suppress misra-c2012-12.3
    O2_PRESSURE_NOT_REACH,
    WAIT_USER_BEFORE_LONG_RUN,
    // cppcheck-suppress misra-c2012-12.3
    START_LONG_RUN_BLOWER,
    // cppcheck-suppress misra-c2012-12.3
    PRESSURE_NOT_STABLE,
    FLOW_NOT_STABLE,
    END_SUCCESS,
    DISPLAY_PRESSURE,
    DISPLAY_FLOW
};

enum TestState {
    STATE_IN_PROGRESS,
    STATE_ERROR,
    STATE_SUCCESS
};

// CLASS ======================================================================

/// Controls the running of the embedded auto tests
// cppcheck-suppress misra-c2012-5.2 ; false positive
class EolTest {
 public:
    /// Default constructor
    EolTest();

    /// Enable test mode
    void activate();

    /**
     * Check if test mode is enabled
     *
     * @return True if test mode is enabled
     */
    bool isRunning();

    /// Run test mode
    void setupAndStart();

 private:
    /// Test mode activation state
    uint32_t testActive;
};

extern EolTest eolTest;
