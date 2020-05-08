/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file end_of_line_test.h
 * @brief End of line tests
 *
 *****************************************************************************/

#pragma once

#define EOL_TEST_ACTIVE 0xa240183a

/// Controls an Pressure Valve's servomotor
class EolTest {
 public:
    /// Default constructor
    EolTest();

    void setupAndStart();

    void mainLoop();
    bool isRunning();
    void activate();

 private:
    uint32_t testActive;
};

void millisecondTimerEOL(HardwareTimer*);

extern HardwareTimer* eolTimer;
extern EolTest eolTest;

#include "../includes/blower.h"
#include "../includes/pressure_valve.h"

// these are defined and initialized in the main program
extern PressureValve servoBlower;
extern PressureValve servoPatient;
extern Blower blower;
extern int16_t pressureOffset;
