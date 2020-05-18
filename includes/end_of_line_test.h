/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file end_of_line_test.h
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

#include "../includes/blower.h"
#include "../includes/pressure_valve.h"

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

extern HardwareTimer* eolTimer;
extern EolTest eolTest;

// These are defined and initialized in the main program
extern PressureValve servoBlower;
extern PressureValve servoPatient;
extern Blower blower;
extern int16_t pressureOffset;
