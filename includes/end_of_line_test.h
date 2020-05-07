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
    HardwareTimer* eolTimer;

};

void millisecondTimerEOL(HardwareTimer*);

extern EolTest eolTest;
