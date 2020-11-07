/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file main_state_machine.cpp
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

#include "../includes/parameters.h"
#include "Arduino.h"
#include <IWatchdog.h>

#include "../includes/activation.h"
#include "../includes/battery.h"
#include "../includes/buzzer_control.h"
#include "../includes/debug.h"
#include "../includes/keyboard.h"
#include "../includes/main_controller.h"
#include "../includes/main_state_machine.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/pressure.h"
#include "../includes/screen.h"
#include "../includes/serial_control.h"
#include "../includes/telemetry.h"

// INITIALISATION =============================================================

MainStateMachine mainStateMachine = MainStateMachine();

uint32_t clockMsmTimer = 0;
uint32_t lastMillis = 0;
uint32_t lastMillisInTheLoop = 0;  // TODO better naming for this variable
uint32_t MainStateMachineNumber = 1;
HardwareTimer* msmTimer;
uint32_t lastMicro = 0;
uint32_t tick = 0;

enum TestStep { SETUP, STOPPED, INIT_CYCLE, BREATH, TRIGGER_RAISED, END_CYCLE };

TestStep msmstep = SETUP;
TestStep previousmsmstep = SETUP;

// FUNCTIONS ==================================================================

MainStateMachine::MainStateMachine() { isMsmActive = false; }

// cppcheck-suppress unusedFunction
void MainStateMachine::activate() {
    isMsmActive = true;
    ::clockMsmTimer = 0;
    ::msmTimer = new HardwareTimer(TIM9);
}

bool MainStateMachine::isRunning() { return isMsmActive; }

void MainStateMachine::ScreenUpdate() {
    displayCurrentVolume(mainController.tidalVolumeMeasure(),
                         mainController.cyclesPerMinuteNextCommand());
    displayCurrentSettings(mainController.peakPressureNextCommand(),
                           mainController.plateauPressureNextCommand(),
                           mainController.peepNextCommand());
    if (msmstep == STOPPED) {
        displayMachineStopped();
    }
}

void millisecondTimerMSM(HardwareTimer*) {
    IWatchdog.reload();
    clockMsmTimer++;
    uint32_t pressure = inspiratoryPressureSensor.read();
    mainController.updatePressure(pressure);
    int32_t inspiratoryflow = MFM_read_airflow();
    mainController.updateInspiratoryFlow(inspiratoryflow);
    // TODO: reactivate because higher priority timerIWatchdog.reload();

    if (clockMsmTimer % 10 == 0) {
        // Check if some buttons have been pushed
        keyboardLoop();
        // Check if battery state has changed
        batteryLoop(mainController.cycleNumber());
        // Check serial input
        serialControlLoop();

        if (isBatteryDeepDischarged()) {
            // Delay will trigger the watchdog and the machine will restart with a message
            // on screen
            delay(10000);
        }
        alarmController.runAlarmEffects(tick);
        tick++;  // TODO this is not very beautiful
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

    if (msmstep == SETUP) {
        mainController.setup();
        mainStateMachine.ScreenUpdate();
        displayMachineStopped();
        msmstep = STOPPED;
    }

    // Executed just after booting, until the first start
    else if (msmstep == STOPPED) {
        if ((clockMsmTimer % 100u) == 0u) {
            mainController.stop();
            displayMachineStopped();
        }

        activationController.refreshState();
        if (activationController.isRunning()) {
            msmstep = INIT_CYCLE;
        }
    }

    else if (msmstep == INIT_CYCLE) {
        mainController.initRespiratoryCycle();
        lastMillis = millis();
        lastMillisInTheLoop = millis();
        tick = 0;
        msmstep = BREATH;
        MFM_read_milliliters(true);  // Reset volume integral
    }

    // If breathing
    else if (msmstep == BREATH) {
        uint32_t currentMillis = millis();
        tick = (currentMillis - lastMillis) / 10u;

        if (currentMillis - lastMillisInTheLoop > 10u) {
            if (tick >= mainController.tickPerCycle()) {
                msmstep = END_CYCLE;
            } else {
                uint32_t currentMicro = micros();
                mainController.updateCurrentDeliveredVolume(MFM_read_milliliters(false));
                mainController.updateDt(currentMicro - lastMicro);
                lastMicro = currentMicro;
                mainController.updateTick(tick);
                mainController.compute();
            }
            lastMillisInTheLoop = currentMillis;
        }

        if (mainController.triggered()) {
            msmstep = TRIGGER_RAISED;
        }

        // Check if machine has been paused
        activationController.refreshState();
        if (!activationController.isRunning()) {
            msmstep = STOPPED;
        }
    } else if (msmstep == TRIGGER_RAISED) {
        if (activationController.isRunning()) {
            msmstep = END_CYCLE;
        } else {
            msmstep = STOPPED;
        }
    }

    else if (msmstep == END_CYCLE) {
        mainController.endRespiratoryCycle();
        displayCurrentInformation(mainController.peakPressureMeasure(),
                                  mainController.plateauPressureMeasure(),
                                  mainController.peepMeasure());
        if (activationController.isRunning()) {
            msmstep = INIT_CYCLE;
        } else {
            msmstep = STOPPED;
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
    // priority level :
    // https://stm32f4-discovery.net/2014/05/stm32f4-stm32f429-nvic-or-nested-vector-interrupt-controller/
    ::msmTimer->setInterruptPriority(0, 0);
    ::msmTimer->setMode(1, TIMER_OUTPUT_COMPARE, NC);
    ::msmTimer->attachInterrupt(millisecondTimerMSM);
    ::msmTimer->resume();
}
