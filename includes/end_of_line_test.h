/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file end_of_line_test.h
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

#include "../includes/blower.h"
#include "../includes/pressure_valve.h"

/**
 * This an arbitrary value allowing to check if test mode is active. A variable is initialized at
 * beginning of testMode with this value. Due to the fact this value is arbitrary and big, it is
 * highly impossible that a memory corruption randomly gives this value
 */
#define EOL_TEST_ACTIVE 0xa240183a

/// Controls the running of the embedded auto tests
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
