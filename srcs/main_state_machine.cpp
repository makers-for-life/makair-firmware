/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file main_state_machine.cpp
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

#include "../includes/parameters.h"
#include "Arduino.h"
#include <IWatchdog.h>

#include "../includes/activation.h"
#include "../includes/battery.h"
#include "../includes/buzzer_control.h"
#include "../includes/debug.h"
#include "../includes/keyboard.h"
#include "../includes/main_state_machine.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/pressure.h"
#include "../includes/pressure_controller.h"
#include "../includes/screen.h"
#include "../includes/serial_control.h"
#include "../includes/telemetry.h"

uint32_t clockMsmTimer = 0;
uint32_t lastMillis = 0;
uint32_t MainStateMachineNumber = 1;
HardwareTimer* msmTimer;
uint32_t lastMicro = 0;
uint32_t tick= 0;


enum TestStep {
    START,
    WAIT_FOR_START,
    START_BREATHING,
    INIT_CYCLE,
    INHALE_RISE,
    INHALE_HOLD,
    EXHALE_FALL,
    EXHALE_HOLD,
    TRIGGER_RAISED,
    END_CYCLE
};

TestStep msmstep = START;
TestStep previousmsmstep = START;

MainStateMachine::MainStateMachine() {
    isMsmActive = false;
    ::msmTimer = new HardwareTimer(TIM9);
}

// cppcheck-suppress unusedFunction
void MainStateMachine::activate() {
    isMsmActive = true;
    ::clockMsmTimer = 0;
}

bool MainStateMachine::isRunning() { return isMsmActive; }

// Display informations on screen.
void MainStateMachine::ScreenUpdate() {
    displayCurrentVolume(pController.tidalVolume(), pController.cyclesPerMinuteCommand());
    displayCurrentSettings(pController.PeakPressureCommand(), pController.PlateauPressureCommand(),
                           pController.PeepCommand());
    if (msmstep == WAIT_FOR_START) {
        displayMachineStopped();
    }
}



void millisecondTimerMSM(HardwareTimer*) {

    clockMsmTimer++;
    uint32_t pressure = inspiratoryPressureSensor.read();
    pController.updatePressure(pressure);
    uint32_t inspiratoryflow = 0;
    pController.updateInspiratoryFlow(inspiratoryflow);

    if (clockMsmTimer % 10 == 0) {
        // Check if some buttons have been pushed
        keyboardLoop();
        // Check if battery state has changed
        batteryLoop(pController.cycleNumber());
        // Check serial input
        serialControlLoop();

        if (isBatteryDeepDischarged()) {
            // Delay will trigger the watchdog and the machine will restart with a message
            // on screen
            delay(10000);
        }
        alarmController.runAlarmEffects(tick);
    }
    // Because this kind of LCD screen is not reliable, we need to reset it every 5 min or
    // so
    if (clockMsmTimer % 300000 == 0) {
        DBG_DO(Serial.println("resetting LCD screen");)
        resetScreen();
        clearAlarmDisplayCache();
    }

    // Refresh screen every 300 ms, no more
    if ((clockMsmTimer % 300u) == 0u) {
        mainStateMachine.ScreenUpdate();
    }

    if (msmstep == START) {
        pController.setup();
        mainStateMachine.ScreenUpdate();
        displayMachineStopped();
        msmstep = WAIT_FOR_START;

    }
    // Executed juste after booting, until the first start.
    else if (msmstep == WAIT_FOR_START) {
        if ((clockMsmTimer % 1000u) == 0u) {
            pController.sendSnapshot(false);
        }

        if ((clockMsmTimer % 100u) == 0u) {
            pController.stop();
            displayMachineStopped();
        }

        activationController.refreshState();
        if (activationController.isRunning()) {
            msmstep = START_BREATHING;
        }

    }

    else if (msmstep == START_BREATHING) {
        msmstep = INIT_CYCLE;

    }

    else if (msmstep == INIT_CYCLE) {
        pController.initRespiratoryCycle();
        lastMillis = millis();
        tick = 0;
        msmstep = INHALE_RISE;

    }

    // If breathing
    else if (msmstep == INHALE_RISE || msmstep == INHALE_HOLD || msmstep == EXHALE_FALL
             || msmstep == EXHALE_HOLD) {

        if (clockMsmTimer % 10 == 0) {
            uint32_t currentMicro = micros();
            pController.updateDt(currentMicro - lastMicro);
            lastMicro = currentMicro;

            uint32_t currentMillis = millis();
            tick = (currentMillis - lastMillis) / 10u;
            pController.compute(tick);

            if (tick > pController.tickPerCycle()) {
                msmstep = END_CYCLE;
            }
        }

        /*if (msmstep == INHALE_RISE) {

        } else if (msmstep == INHALE_HOLD) {

        } else if (msmstep == EXHALE_FALL) {

        } else if (msmstep == EXHALE_HOLD) {
        }*/

        if (pController.triggered()) {
            msmstep = TRIGGER_RAISED;
        }
        // Check if machine has been paused
        activationController.refreshState();
        if (!activationController.isRunning()) {
            msmstep = END_CYCLE;
        }

    } else if (msmstep == TRIGGER_RAISED) {
        msmstep = END_CYCLE;
    }

    else if (msmstep == END_CYCLE) {
        pController.endRespiratoryCycle();
        displayCurrentInformation(pController.peakPressure(), pController.plateauPressure(),
                                  pController.peep());
        if (activationController.isRunning()){
            msmstep = START_BREATHING;
        } else {
            msmstep = WAIT_FOR_START;
        }
        
    }

    previousmsmstep = msmstep;
}

void MainStateMachine::setupAndStart() {

    // Set a 1 ms timer for the event loop
    // Prescaler at 10 kHz; stm32f411 clock is 100 mHz
    ::msmTimer->setPrescaleFactor((::msmTimer->getTimerClkFreq() / 10000) - 1);
    // Set the period at 1 ms
    ::msmTimer->setOverflow(10);
    ::msmTimer->setMode(1, TIMER_OUTPUT_COMPARE, NC);
    ::msmTimer->attachInterrupt(millisecondTimerMSM);
    ::msmTimer->resume();
}

MainStateMachine mainStateMachine = MainStateMachine();
