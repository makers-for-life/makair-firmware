/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file end_of_line_test.h
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

#include "../includes/alarm_controller.h"
#include "../includes/blower.h"
#include "../includes/main_controller.h"
#include "../includes/pressure_valve.h"

/// Controls the running of the embedded auto tests
class MainStateMachine {
 public:
    /// Default constructor
    MainStateMachine();

    /// Enable test mode
    void activate();

    /**
     * Check if test mode is enabled
     *
     * @return True if test mode is enabled
     */
    bool isRunning();

    void ScreenUpdate();

    //void millisecondTimerMSM();

    /// Run test mode
    void setupAndStart();

 private:
    /// Test mode activation state
    bool isMsmActive;
    uint32_t testActive;
};

extern MainStateMachine mainStateMachine;
