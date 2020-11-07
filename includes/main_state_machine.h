/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file main_state_machine.h
 * @brief Main state machine
 *****************************************************************************/

#pragma once

#include "../includes/alarm_controller.h"
#include "../includes/blower.h"
#include "../includes/main_controller.h"
#include "../includes/pressure_valve.h"

/// Main state machine
class MainStateMachine {
 public:
    /// Default constructor
    MainStateMachine();

    /// Starts the state machine and its hardware timer
    void activate();

    /**
     * Check if the state machine is enabled
     *
     * @return True if the state machine is enabled
     */
    bool isRunning();

    /// Display information on screen
    void ScreenUpdate();

    // void millisecondTimerMSM();

    /// Run the state machine
    void setupAndStart();

 private:
    /// Main state machine activation state
    bool isMsmActive;
    uint32_t testActive;
};

extern MainStateMachine mainStateMachine;
