/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file activation.h
 * @brief Breathing activation related functions
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// External libraries
#include "Arduino.h"

// CLASS ======================================================================

/// Controls breathing activation ON/OFF state
class ActivationController {
 public:
    ActivationController();

    /**
     * Refresh the current state
     *
     * @warning It must be called regularly to protect against time counter overflow
     */
    void refreshState();

    /**
     * Return if breathing is activated or not
     */
    bool isRunning() const { return m_state != STOPPED; }

    /**
     * Change the current state
     *
     * @param state New state: 0 = STOPPED, 1 = RUNNING
     */
    void changeState(uint16_t state);

    /**
     * Callback to call each time the start button is pushed
     */
    void onStartButton();

    /**
     * Callback to call each time the stop button is pushed
     */
    void onStopButton();

 private:
    enum State {
        /// Breathing is OFF
        STOPPED = 0,

        /// Breathing is ON
        RUNNING,

        /// Breathing is ON, waiting for a second push to stop
        RUNNING_READY_TO_STOP,
    };

    /// Activation status
    State m_state;

    /// Last time stop button was pushed
    uint32_t m_timeOfLastStopPushed;
};

// INITIALISATION =============================================================

/// Instance of the activation controller
extern ActivationController activationController;
