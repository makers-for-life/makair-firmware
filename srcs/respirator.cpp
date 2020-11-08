/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file respirator.cpp
 * @brief Entry point of ventilator program
 *****************************************************************************/

#pragma once

#include "../includes/config.h"
#if MODE == MODE_PROD

// INCLUDES ==================================================================

// External
#include "Arduino.h"
#include <HardwareSerial.h>
#include <IWatchdog.h>
#include <LiquidCrystal.h>

// Internal
#include "../includes/battery.h"
#include "../includes/blower.h"
#include "../includes/buzzer.h"
#include "../includes/buzzer_control.h"
#include "../includes/cpuLoad.h"
#include "../includes/debug.h"
#include "../includes/end_of_line_test.h"
#include "../includes/keyboard.h"
#include "../includes/main_controller.h"
#include "../includes/main_state_machine.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/parameters.h"
#include "../includes/pressure.h"
#include "../includes/pressure_valve.h"
#include "../includes/screen.h"
#include "../includes/serial_control.h"
#include "../includes/telemetry.h"

// PROGRAM =====================================================================

HardwareTimer* hardwareTimer1;  // ESC command
HardwareTimer* hardwareTimer3;  // valves command

int32_t pressureOffsetSum;
uint32_t pressureOffsetCount;
int32_t minOffsetValue = 0;
int32_t maxOffsetValue = 0;

HardwareSerial Serial6(PIN_TELEMETRY_SERIAL_RX, PIN_TELEMETRY_SERIAL_TX);

/**
 * Block execution for a given duration
 *
 * @param ms  Duration of the blocking in millisecond
 */
void waitAndMeasurePressure(uint16_t ms) {
    uint16_t start = millis();
    minOffsetValue = inspiratoryPressureSensor.read();
    maxOffsetValue = inspiratoryPressureSensor.read();
    pressureOffsetSum = 0;
    pressureOffsetCount = 0;

    // Open valves
    inspiratoryValve.close();
    inspiratoryValve.execute();
    expiratoryValve.open();
    expiratoryValve.execute();

    while ((millis() - start) < ms) {
        // Measure 1 pressure per ms we wait
        if ((millis() - start) > pressureOffsetCount) {
            int32_t pressureValue = inspiratoryPressureSensor.read();
            pressureOffsetSum += pressureValue;
            minOffsetValue = min(pressureValue, minOffsetValue);
            maxOffsetValue = max(pressureValue, maxOffsetValue);
            pressureOffsetCount++;
        }
        continue;
    }
}

void setup(void) {
    Serial.begin(115200);
    DBG_DO(Serial.println("Booting the system...");)

    startScreen();

    initBattery();
    if (isBatteryDeepDischarged()) {
        displayBatteryDeepDischarge();
        while (true) {
        }
    }

    initTelemetry();
    sendBootMessage();

    // Timer for valves
    hardwareTimer3 = new HardwareTimer(TIM3);
    hardwareTimer3->setOverflow(VALVE_PERIOD, MICROSEC_FORMAT);

    // Valves setup
    inspiratoryValve = PressureValve(hardwareTimer3, TIM_CHANNEL_INSPIRATORY_VALVE,
                                     PIN_INSPIRATORY_VALVE, VALVE_OPEN_STATE, VALVE_CLOSED_STATE);
    inspiratoryValve.setup();
    hardwareTimer3->resume();
    expiratoryValve = PressureValve(hardwareTimer3, TIM_CHANNEL_EXPIRATORY_VALVE,
                                    PIN_EXPIRATORY_VALVE, VALVE_OPEN_STATE, VALVE_CLOSED_STATE);
    expiratoryValve.setup();
    hardwareTimer3->resume();

    // Blower setup
    hardwareTimer1 = new HardwareTimer(TIM1);
    hardwareTimer1->setOverflow(ESC_PPM_PERIOD, MICROSEC_FORMAT);
    blower = Blower(hardwareTimer1, TIM_CHANNEL_ESC_BLOWER, PIN_ESC_BLOWER);
    blower.setup();

    // Init controllers
    mainController = MainController();
    alarmController = AlarmController();

    // Init sensors
    inspiratoryPressureSensor = PressureSensor();
#ifdef MASS_FLOW_METER_ENABLED
    MFM_init();
    MFM_calibrateZero();  // Patient unplugged, also set the zero of mass flow meter. It has no
                          // effect with the actual flowmeter
#endif

    // Setup pins of the microcontroller
    pinMode(PIN_PRESSURE_SENSOR, INPUT);
    pinMode(PIN_BATTERY, INPUT);
    pinMode(PIN_ENABLE_PWR_RASP, OUTPUT);
    pinMode(PIN_LED_START, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PB12, INPUT);

    // Turn on the Raspberry Pi power
    digitalWrite(PIN_ENABLE_PWR_RASP, PWR_RASP_ACTIVE);

    // Activate test mode if a service button is pressed
    // The end of line test mode cannot be activated later on.
    // Autotest inputs: the service button on PB12, top right of the board's rear side
    if (HIGH == digitalRead(PB12)) {
        eolTest.activate();
        displayEndOfLineTestMode();
        while (HIGH == digitalRead(PB12)) {
            continue;
        }
    }

    // Catch potential Watchdog reset
    if (IWatchdog.isReset(true)) {
        // Run a high priority alarm
        BuzzerControl_Init();
        Buzzer_Init();
        Buzzer_High_Prio_Start();
        displayWatchdogError();
        while (1) {
        }
    }

    initKeyboard();
    BuzzerControl_Init();
    Buzzer_Init();

    // RCM-SW-17 (Christmas tree at startup)
    Buzzer_Boot_Start();
    digitalWrite(PIN_LED_START, LED_START_ACTIVE);
    digitalWrite(PIN_LED_GREEN, LED_GREEN_ACTIVE);
    digitalWrite(PIN_LED_RED, LED_RED_ACTIVE);
    digitalWrite(PIN_LED_YELLOW, LED_YELLOW_ACTIVE);
    waitAndMeasurePressure(1000);
    digitalWrite(PIN_LED_START, LED_START_INACTIVE);
    digitalWrite(PIN_LED_GREEN, LED_GREEN_INACTIVE);
    digitalWrite(PIN_LED_RED, LED_RED_INACTIVE);
    digitalWrite(PIN_LED_YELLOW, LED_YELLOW_INACTIVE);
    waitAndMeasurePressure(1000);

    displayPatientMustBeUnplugged();
    waitAndMeasurePressure(2000);

    int32_t inspiratoryPressureSensorOffset = 0;
    resetScreen();
    if (pressureOffsetCount != 0u) {
        inspiratoryPressureSensorOffset =
            pressureOffsetSum / static_cast<int32_t>(pressureOffsetCount);
    } else {
        inspiratoryPressureSensorOffset = 0;
    }

    inspiratoryPressureSensor.setPressureSensorOffset(inspiratoryPressureSensorOffset);

    // Happens when patient is plugged at starting
    if ((maxOffsetValue - minOffsetValue) >= 10
        || inspiratoryPressureSensorOffset >= MAX_PRESSURE_OFFSET) {
        displayPressureOffsetUnstable(minOffsetValue, maxOffsetValue);
        Buzzer_High_Prio_Start();
        while (true) {
        }
    }

#ifdef MASS_FLOW_METER_ENABLED
    int32_t flowMeterFlowAtStarting = MFM_read_airflow();
#else
    int32_t flowMeterFlowAtStarting = 0;
#endif
    inspiratoryValve.open();
    inspiratoryValve.execute();
    expiratoryValve.open();
    expiratoryValve.execute();
    delay(500);
    blower.runSpeed(DEFAULT_BLOWER_SPEED);
    delay(1000);
#ifdef MASS_FLOW_METER_ENABLED
    int32_t flowMeterFlowWithBlowerOn = MFM_read_airflow();
#else
    int32_t flowMeterFlowWithBlowerOn = 0;
#endif

    blower.stop();

    // Happens when flow meter fails
    if (flowMeterFlowAtStarting < -1000 || flowMeterFlowAtStarting > 1000
        || flowMeterFlowWithBlowerOn < 20000 || flowMeterFlowWithBlowerOn > 100000) {
        displayFlowMeterFail(flowMeterFlowAtStarting, flowMeterFlowWithBlowerOn);
        Buzzer_High_Prio_Start();
        while (true) {
        }
    }

    displayPressureOffset(inspiratoryPressureSensorOffset);
    delay(1000);

    if (!eolTest.isRunning()) {
        mainStateMachine.setupAndStart();

        // Init the watchdog timer. It must be reloaded frequently otherwise MCU resests
        IWatchdog.begin(WATCHDOG_TIMEOUT);
        IWatchdog.reload();
    } else {
        eolTest.setupAndStart();
    }
}

// Implements a CPU load estimation

// idleCyclesCount will be incremented in the loop, and read in the systick IRQ.
// if the systick IRQ set idleCyclesCount to 0, there is no guarantee it returns to 0,
// because chances are high to be in the middle of the increment operation, which is not atomic.
volatile uint32_t idleCyclesCount = 0;

// every 1000ms, reset the cpu cycle count and store the result in totalCycleCount.
#define TIME_CPU_CYCLE_RESET 1000u

// the result is in percent
volatile int32_t cpuLoadPercent = 0;

// do not add anything in the loop
void loop(void) { idleCyclesCount++; }

// As there is no Free RTOS here, we can use is for CPU load estimator
uint16_t cpuLoadTimeCount = TIME_CPU_CYCLE_RESET;

// when nothing is enabled, the increment in the loop is done xx /second
// to measure it, remove all the code in setup except Serial initialization,
// and uncomment Serial output below.
#define CPU_MAX_LOOP_PER_SECOND 8327007

uint32_t cpuLoadLatestCycleCount = 0;

extern "C" {
// this is the highest priority 1 ms callback.
void osSystickHandler() {
    cpuLoadTimeCount--;
    if (cpuLoadTimeCount == 0) {
        cpuLoadTimeCount = TIME_CPU_CYCLE_RESET;
        // overflow won't be a problem, same type.
        uint32_t delta = idleCyclesCount - cpuLoadLatestCycleCount;
        cpuLoadLatestCycleCount = idleCyclesCount;
        if (delta > CPU_MAX_LOOP_PER_SECOND) {
            delta = CPU_MAX_LOOP_PER_SECOND;
        }
        cpuLoadPercent = 100 - (100 * delta) / CPU_MAX_LOOP_PER_SECOND;
        // Uncomment to tune CPU_MAX_LOOP_PER_SECOND value.
        // Serial.println(delta);
    }
}
}

int32_t readCpuLoadPercent(void) { return cpuLoadPercent; }

#endif
