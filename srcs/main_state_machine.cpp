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
#include "../includes/rpi_watchdog.h"
#include "../includes/screen.h"
#include "../includes/serial_control.h"
#include "../includes/telemetry.h"

// INITIALISATION =============================================================

MainStateMachine mainStateMachine = MainStateMachine();

uint32_t clockMsmTimer = 0;
uint32_t lastMillis = 0;
uint32_t lastMainControllerCall = 0;
HardwareTimer* msmTimer;
uint32_t lastMicro = 0;
uint32_t tick = 0;

// cppcheck-suppress misra-c2012-12.3 ; cppcheck error
enum Step { SETUP, STOPPED, INIT_CYCLE, BREATH, TRIGGER_RAISED, END_CYCLE };

Step msmstep = SETUP;
Step previousmsmstep = SETUP;

// FUNCTIONS ==================================================================

// cppcheck-suppress misra-c2012-5.2 ; false positive
MainStateMachine::MainStateMachine() { isMsmActive = false; }

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

// API update since version 1.9.0 of Arduino_Core_STM32
#if (STM32_CORE_VERSION < 0x01090000)
// cppcheck-suppress misra-c2012-2.7 ; valid unused parameter
void millisecondTimerMSM(HardwareTimer*)  // NOLINT(readability/casting)
#else
void millisecondTimerMSM(void)
#endif
{
    IWatchdog.reload();
    clockMsmTimer++;
    int32_t pressure = inspiratoryPressureSensor.read();
    mainController.updatePressure(pressure);
    int32_t inspiratoryflow = 0;
#ifdef MASS_FLOW_METER_ENABLED
    inspiratoryflow = MFM_read_airflow();
#endif

    mainController.updateInspiratoryFlow(inspiratoryflow);

    if ((clockMsmTimer % 10u) == 0u) {
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
    }

    // Because this kind of LCD screen is not reliable, we need to reset it every 5 min or
    // so
    if ((clockMsmTimer % 300000u) == 0u) {
        DBG_DO(Serial.println("resetting LCD screen");)
        resetScreen();
        clearAlarmDisplayCache();
    }

    // Refresh screen every 300 ms, no more
    if ((clockMsmTimer % 300u) == 0u) {
        mainStateMachine.ScreenUpdate();
    }

    // Check that the UI software on the Raspberry PI has sent a heartbeat in the last 60s
    // Otherwise restart the power
    if ((clockMsmTimer % 1000u) == 0u) {
        rpiWatchdog.update();
    }

    if (msmstep == SETUP) {
        mainController.setup();
        mainStateMachine.ScreenUpdate();
        displayMachineStopped();
        msmstep = STOPPED;
    } else if (msmstep == STOPPED) {
        // Executed just after booting, until the first start
        if ((clockMsmTimer % 100u) == 0u) {
            mainController.stop(millis());
            displayMachineStopped();
        }

        if ((clockMsmTimer % 10u) == 0u) {
            tick++;  // Also increase ticks during stop, for alarm controller
        }

        activationController.refreshState();
        if (activationController.isRunning()) {
            msmstep = INIT_CYCLE;
        }

    } else if (msmstep == INIT_CYCLE) {
        mainController.initRespiratoryCycle();
        lastMillis = millis();
        lastMainControllerCall = millis();
        tick = 0;
        msmstep = BREATH;
#ifdef MASS_FLOW_METER_ENABLED
        (void)MFM_read_milliliters(true);  // Reset volume integral
#endif

    } else if (msmstep == BREATH) {
        // If breathing
        uint32_t currentMillis = millis();
        tick = (currentMillis - lastMillis) / MAIN_CONTROLLER_COMPUTE_PERIOD_MS;

        if ((currentMillis - lastMainControllerCall) > MAIN_CONTROLLER_COMPUTE_PERIOD_MS) {
            if (tick >= mainController.ticksPerCycle()) {
                msmstep = END_CYCLE;
            } else {
                mainController.updateFakeExpiratoryFlow();
                uint32_t currentMicro = micros();
#ifdef MASS_FLOW_METER_ENABLED
                mainController.updateCurrentDeliveredVolume(MFM_read_milliliters(false));
#endif
                mainController.updateDt(currentMicro - lastMicro);
                lastMicro = currentMicro;
                mainController.updateTick(tick);
                mainController.compute();
                lastMainControllerCall = currentMillis;
                tick++;
            }
        }

        if (mainController.triggered()) {
            msmstep = TRIGGER_RAISED;
        }

        // Check if machine has been paused
        activationController.refreshState();
        if (!activationController.isRunning()) {
            msmstep = SETUP;
        }
    } else if (msmstep == TRIGGER_RAISED) {
        if (activationController.isRunning()) {
            msmstep = END_CYCLE;
        } else {
            msmstep = SETUP;
        }
    } else if (msmstep == END_CYCLE) {
        mainController.endRespiratoryCycle(millis());
        displayCurrentInformation(mainController.peakPressureMeasure(),
                                  mainController.plateauPressureMeasure(),
                                  mainController.peepMeasure());
        if (activationController.isRunning()) {
            msmstep = INIT_CYCLE;
        } else {
            msmstep = SETUP;
        }
    } else {
        // Do nothing
    }

    previousmsmstep = msmstep;
}

void MainStateMachine::setupAndStart() {
    isMsmActive = true;
    ::clockMsmTimer = 0;
    ::msmTimer = new HardwareTimer(TIM9);
    // Set a 1 ms timer for the event loop
    // Prescaler at 10 kHz; stm32f411 clock is 100 mHz
    ::msmTimer->setPrescaleFactor((::msmTimer->getTimerClkFreq() / 10000) - 1);
    // Set the period at 1 ms
    ::msmTimer->setOverflow(10);
    // priority level :
    // https://stm32f4-discovery.net/2014/05/stm32f4-stm32f429-nvic-or-nested-vector-interrupt-controller/
    ::msmTimer->setInterruptPriority(6, 0);
    ::msmTimer->setMode(1, TIMER_OUTPUT_COMPARE, NC);
    ::msmTimer->attachInterrupt(millisecondTimerMSM);
    ::msmTimer->resume();
}
