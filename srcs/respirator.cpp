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
#include "../includes/activation.h"
#include "../includes/battery.h"
#include "../includes/blower.h"
#include "../includes/buzzer.h"
#include "../includes/buzzer_control.h"
#include "../includes/debug.h"
#include "../includes/end_of_line_test.h"
#include "../includes/keyboard.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/parameters.h"
#include "../includes/pressure.h"
#include "../includes/pressure_controller.h"
#include "../includes/pressure_valve.h"
#include "../includes/screen.h"
#include "../includes/serial_control.h"
#include "../includes/telemetry.h"

// PROGRAM =====================================================================

PressureValve inspiratoryValve;
PressureValve expiratoryValve;
HardwareTimer* hardwareTimer1;
HardwareTimer* hardwareTimer3;
Blower* blower_pointer;
Blower blower;

int16_t pressureOffset;
int32_t pressureOffsetSum;
uint32_t pressureOffsetCount;
int16_t minOffsetValue = 0;
int16_t maxOffsetValue = 0;

HardwareSerial Serial6(PIN_TELEMETRY_SERIAL_RX, PIN_TELEMETRY_SERIAL_TX);

/**
 * Block execution for a given duration
 *
 * @param ms  Duration of the blocking in millisecond
 */
void waitForInMs(uint16_t ms) {
    uint16_t start = millis();
    minOffsetValue = readPressureSensor(0, 0);
    maxOffsetValue = readPressureSensor(0, 0);
    pressureOffsetSum = 0;
    pressureOffsetCount = 0;

    while ((millis() - start) < ms) {
        // Measure 1 pressure per ms we wait
        if ((millis() - start) > pressureOffsetCount) {
            int16_t pressureValue = readPressureSensor(0, 0);
            pressureOffsetSum += pressureValue;
            minOffsetValue = min(pressureValue, minOffsetValue);
            maxOffsetValue = max(pressureValue, maxOffsetValue);
            pressureOffsetCount++;
        }
        continue;
    }
}

uint32_t lastpControllerComputeDate;

void setup(void) {
    DBG_DO(Serial.begin(115200);)
    DBG_DO(Serial.println("Booting the system...");)

    startScreen();

    initBattery();
    if (isBatteryDeepDischarged()) {
        screen.clear();
        screen.setCursor(0, 0);
        screen.print("Battery very low");
        screen.setCursor(0, 2);
        screen.print("Please charge");
        screen.setCursor(0, 3);
        screen.print("before running.");
        while (true) {
        }
    }

    initTelemetry();
    sendBootMessage();

    pinMode(PIN_PRESSURE_SENSOR, INPUT);
    pinMode(PIN_BATTERY, INPUT);

    // Timer for servos
    hardwareTimer3 = new HardwareTimer(TIM3);
    hardwareTimer3->setOverflow(SERVO_VALVE_PERIOD, MICROSEC_FORMAT);

    // Servo blower setup
    inspiratoryValve = PressureValve(hardwareTimer3, TIM_CHANNEL_SERVO_VALVE_BLOWER, PIN_SERVO_BLOWER,
                                VALVE_OPEN_STATE, VALVE_CLOSED_STATE);
    inspiratoryValve.setup();
    hardwareTimer3->resume();

    // Servo patient setup
    expiratoryValve = PressureValve(hardwareTimer3, TIM_CHANNEL_SERVO_VALVE_PATIENT, PIN_SERVO_PATIENT,
                                 VALVE_OPEN_STATE, VALVE_CLOSED_STATE);
    expiratoryValve.setup();
    hardwareTimer3->resume();

    hardwareTimer1 = new HardwareTimer(TIM1);
    hardwareTimer1->setOverflow(ESC_PPM_PERIOD, MICROSEC_FORMAT);
    blower = Blower(hardwareTimer1, TIM_CHANNEL_ESC_BLOWER, PIN_ESC_BLOWER);
    blower.setup();
    blower_pointer = &blower;

    // Turn on the raspberry power
    pinMode(PIN_ENABLE_PWR_RASP, OUTPUT);
    digitalWrite(PIN_ENABLE_PWR_RASP, PWR_RASP_ACTIVE);

    // Activate test mode if a service button is pressed. The end of line test mode cannot be
    // activated later on.
    // Autotest inputs: the service button on PB12, top right of the board's rear side
    pinMode(PB12, INPUT);
    if (HIGH == digitalRead(PB12)) {
        eolTest.activate();
        screen.clear();
        screen.print("EOL Test Mode");
        while (HIGH == digitalRead(PB12)) {
            continue;
        }
    }

    // Open both valves at startup
    inspiratoryValve.open();
    inspiratoryValve.execute();
    expiratoryValve.open();
    expiratoryValve.execute();

    // Catch potential Watchdog reset
    // cppcheck-suppress misra-c2012-14.4 ; unknown external signature
    if (IWatchdog.isReset(true)) {
        // Run a high priority alarm
        BuzzerControl_Init();
        Buzzer_Init();
        Buzzer_High_Prio_Start();

        // Print message on the screen
        screen.clear();
        screen.setCursor(0, 0);
        screen.print("An error has occured");
        screen.setCursor(0, 2);
        screen.print("Check the machine");
        screen.setCursor(0, 3);
        screen.print("before re-using");

        // Wait infinitely
        while (1) {
        }
    }

    // Do not initialize pressure controller and keyboard in test mode
    if (!eolTest.isRunning()) {
        alarmController = AlarmController();

        pController =
            PressureController(INITIAL_CYCLE_NUMBER, DEFAULT_MIN_PEEP_COMMAND,
                               DEFAULT_MAX_PLATEAU_COMMAND, DEFAULT_MAX_PEAK_PRESSURE_COMMAND,
                               inspiratoryValve, expiratoryValve, &alarmController, blower_pointer);
        pController.setup();
        pController.reachSafetyPosition();
        initKeyboard();
    }

    // Prepare LEDs
    pinMode(PIN_LED_START, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);

    BuzzerControl_Init();
    Buzzer_Init();

    // escBlower needs 5s at speed 0 to be properly initalized

    // RCM-SW-17 (Christmas tree at startup)
    Buzzer_Boot_Start();
    digitalWrite(PIN_LED_START, LED_START_ACTIVE);
    digitalWrite(PIN_LED_GREEN, LED_GREEN_ACTIVE);
    digitalWrite(PIN_LED_RED, LED_RED_ACTIVE);
    digitalWrite(PIN_LED_YELLOW, LED_YELLOW_ACTIVE);
    waitForInMs(1000);
    digitalWrite(PIN_LED_START, LED_START_INACTIVE);
    digitalWrite(PIN_LED_GREEN, LED_GREEN_INACTIVE);
    digitalWrite(PIN_LED_RED, LED_RED_INACTIVE);
    digitalWrite(PIN_LED_YELLOW, LED_YELLOW_INACTIVE);
    waitForInMs(3000);

    screen.setCursor(0, 0);
    screen.print("Calibrating P offset");
    screen.setCursor(0, 2);
    screen.print("Patient must be");
    screen.setCursor(0, 3);
    screen.print("unplugged");
    waitForInMs(3000);

// Mass Flow Meter, if any
#ifdef MASS_FLOW_METER
    (void)MFM_init();
    MFM_calibrateZero();  // Patient unplugged, also set the zero of mass flow meter
#endif

    resetScreen();
    if (pressureOffsetCount != 0u) {
        pressureOffset = pressureOffsetSum / static_cast<int32_t>(pressureOffsetCount);
    } else {
        pressureOffset = 0;
    }
    DBG_DO({
        Serial.print("pressure offset = ");
        Serial.print(pressureOffsetSum);
        Serial.print(" / ");
        Serial.print(pressureOffsetCount);
        Serial.print(" = ");
        Serial.print(pressureOffset);
        Serial.println();
    })

    // Happens when patient is plugged at starting
    if ((maxOffsetValue - minOffsetValue) >= 10) {
        resetScreen();
        screen.setCursor(0, 0);
        char line1[SCREEN_LINE_LENGTH + 1];
        (void)snprintf(line1, SCREEN_LINE_LENGTH + 1, "P offset is unstable");
        screen.print(line1);
        screen.setCursor(0, 1);
        char line2[SCREEN_LINE_LENGTH + 1];
        (void)snprintf(line2, SCREEN_LINE_LENGTH + 1, "Max-Min: %3d mmH2O",
                       maxOffsetValue - minOffsetValue);
        screen.print(line2);
        screen.setCursor(0, 2);
        screen.print("Unplug patient and");
        screen.setCursor(0, 3);
        screen.print("reboot");
        Buzzer_High_Prio_Start();
        while (true) {
        }
    }

    if (pressureOffset >= MAX_PRESSURE_OFFSET) {
        resetScreen();
        screen.setCursor(0, 0);
        char line1[SCREEN_LINE_LENGTH + 1];
        (void)snprintf(line1, SCREEN_LINE_LENGTH + 1, "P offset: %3d mmH2O", pressureOffset);
        screen.print(line1);
        screen.setCursor(0, 1);
        char line2[SCREEN_LINE_LENGTH + 1];
        (void)snprintf(line2, SCREEN_LINE_LENGTH + 1, "P offset is > %-3d", MAX_PRESSURE_OFFSET);
        screen.print(line2);
        screen.setCursor(0, 2);
        screen.print("Unplug patient and");
        screen.setCursor(0, 3);
        screen.print("reboot");
        Buzzer_High_Prio_Start();
        while (true) {
        }
    }

    screen.setCursor(0, 3);
    char message[SCREEN_LINE_LENGTH + 1];
    (void)snprintf(message, SCREEN_LINE_LENGTH + 1, "P offset: %3d mmH2O", pressureOffset);
    screen.print(message);
    waitForInMs(1000);

    lastpControllerComputeDate = micros();

    // No watchdog in end of line test mode
    if (!eolTest.isRunning()) {
        // Init the watchdog timer. It must be reloaded frequently otherwise MCU resests
        IWatchdog.begin(WATCHDOG_TIMEOUT);
        IWatchdog.reload();
    } else {
        eolTest.setupAndStart();
    }
}

// Time of the previous loop iteration
int32_t lastMicro = 0;

// Number of cycles before LCD screen reset
// (because this kind of screen is not reliable, we need to reset it every 5 min or so)
int8_t cyclesBeforeScreenReset = LCD_RESET_PERIOD * (int8_t)CONST_MIN_CYCLE;

// cppcheck-suppress unusedFunction
void loop(void) {
    if (!eolTest.isRunning()) {
        /********************************************/
        // INITIALIZE THE RESPIRATORY CYCLE
        /********************************************/
        activationController.refreshState();
        bool shouldRun = activationController.isRunning();

        if (shouldRun) {
            pController.initRespiratoryCycle();
        }

        /********************************************/
        // START THE RESPIRATORY CYCLE
        /********************************************/
        uint32_t tick = 0;

        while (tick < pController.tickPerCycle() && !pController.triggered()) {
            uint32_t pressure = readPressureSensor(tick, pressureOffset);

            uint32_t currentDate = micros();

            uint32_t diff = (currentDate - lastpControllerComputeDate);

            if (diff >= PCONTROLLER_COMPUTE_PERIOD_US) {
                lastpControllerComputeDate = currentDate;

                if (shouldRun) {
                    digitalWrite(PIN_LED_START, LED_START_ACTIVE);
                    pController.updatePressure(pressure);
                    int32_t currentMicro = micros();

                    pController.updateDt(currentMicro - lastMicro);
                    lastMicro = currentMicro;

                    // Perform the pressure control
                    pController.compute(tick);
                } else {
                    digitalWrite(PIN_LED_START, LED_START_INACTIVE);
                    blower.stop();
                    // When stopped, open the valves
                    inspiratoryValve.open();
                    inspiratoryValve.execute();
                    expiratoryValve.open();
                    expiratoryValve.execute();
                    // Stop alarms related to breathing cycle
                    alarmController.notDetectedAlarm(RCM_SW_1);
                    alarmController.notDetectedAlarm(RCM_SW_2);
                    alarmController.notDetectedAlarm(RCM_SW_3);
                    alarmController.notDetectedAlarm(RCM_SW_14);
                    alarmController.notDetectedAlarm(RCM_SW_15);
                    alarmController.notDetectedAlarm(RCM_SW_18);
                    alarmController.notDetectedAlarm(RCM_SW_19);

                    if ((tick % 10u) == 0u) {
                        sendStoppedMessage();
                    }
                }

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

                // Display relevant information during the cycle
                if ((tick % (LCD_UPDATE_PERIOD_US / PCONTROLLER_COMPUTE_PERIOD_US)) == 0u) {
#ifdef MASS_FLOW_METER
                    displayCurrentVolume(MFM_read_milliliters(false),
                                         pController.cyclesPerMinuteCommand());
#else
                    displayCurrentPressure(pController.pressure(),
                                           pController.cyclesPerMinuteCommand());
#endif

                    displayCurrentSettings(pController.maxPeakPressureCommand(),
                                           pController.maxPlateauPressureCommand(),
                                           pController.minPeepCommand());
                }

                alarmController.runAlarmEffects(tick);

                // next tick
                tick++;
                IWatchdog.reload();
            }
        }

        if (shouldRun) {
            pController.endRespiratoryCycle();
        }

        /********************************************/
        // END OF THE RESPIRATORY CYCLE
        /********************************************/

        // Because this kind of LCD screen is not reliable, we need to reset it every 5 min or
        // so
        cyclesBeforeScreenReset--;
        DBG_DO(Serial.println(cyclesBeforeScreenReset);)
        if (cyclesBeforeScreenReset <= 0) {
            DBG_DO(Serial.println("resetting LCD screen");)
            resetScreen();
            clearAlarmDisplayCache();
            cyclesBeforeScreenReset = LCD_RESET_PERIOD * (int8_t)CONST_MIN_CYCLE;
        }

        if (shouldRun) {
            displayCurrentInformation(pController.peakPressure(), pController.plateauPressure(),
                                      pController.peep());
        } else {
            displayMachineStopped();
        }
    }
}

#endif
