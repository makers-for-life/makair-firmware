/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file qualification.cpp
 * @brief Program to launch the blower and simulate a 1:2 open/close ratio with
 * a faulhaber valve connected on either patient or blower output.
 * The blower starts at full speed. speed can be adjusted with start and stop buttons.
 *****************************************************************************/

#pragma once

#include "../includes/config.h"
#if MODE == MODE_BLOWER_TEST

// External
#include "../includes/config.h"
#include <Arduino.h>
#include <OneButton.h>
#include <Wire.h>

// Internal
#include "../includes/parameters.h"
#include "../includes/blower.h"
#include "../includes/pressure_valve.h"
#include "../includes/screen.h"

HardwareTimer* testTimer;
HardwareTimer* hardwareTimer1;
HardwareTimer* hardwareTimer3;
PressureValve servoBlower;
PressureValve servoPatient;
Blower blower;

static OneButton buttonAlarmOff(PIN_BTN_ALARM_OFF, false, false);
static OneButton buttonStart(PIN_BTN_START, false, false);
static OneButton buttonStop(PIN_BTN_STOP, false, false);

int periodValve = 500;

// 10 ms task
int count = 0;
boolean ValveOpened = true;

uint32_t cycleCount = 0;
int currentPwm = 0;

uint32_t clock10ms = 0;
volatile uint32_t clock1s = 0;

// define a normal 20 cycle per minute breathing rate with a 1:2 ratio, for tests.
#define CYCLEPERMINUTE 20
#define CYCLEPERIOD (6000 / CYCLEPERMINUTE)
#define INHALATION (CYCLEPERIOD / 3)
uint32_t cycleClock = 0;
volatile uint32_t closedTime = 0;
volatile uint32_t openedTime = 0;

uint32_t blowerPowerPercent = 90;

void Test_Timer_Callback(HardwareTimer*) {
    // 10 ms period
    clock10ms++;
    if (100 == clock10ms) {
        clock10ms = 0;
        clock1s++;
    }

    cycleClock++;
    if (cycleClock == CYCLEPERIOD) {
        cycleClock = 0;
    }
    if (cycleClock < INHALATION) {
        // inhalation phase, valves opened
        servoBlower.open();
        servoPatient.open();
        digitalWrite(PIN_LED_START, LED_START_ACTIVE);
    } else {
        servoBlower.close();
        servoPatient.close();
        digitalWrite(PIN_LED_START, LED_START_INACTIVE);
    }
    servoBlower.execute();
    servoPatient.execute();
    

    buttonAlarmOff.tick();
    buttonStart.tick();
    buttonStop.tick();
}

void updateBlowerSpeed(void) {
    blower.runSpeed(map(blowerPowerPercent, 1, 100, MIN_BLOWER_SPEED + 1, MAX_BLOWER_SPEED - 1));
}
void onStopClick() {
    // increase blower speed
    if (blowerPowerPercent < 100) {
        blowerPowerPercent++;
    }
    updateBlowerSpeed();
}
void onStartClick() {
    // decrease
    if (blowerPowerPercent > 0) {
        blowerPowerPercent--;
    }
    updateBlowerSpeed();
}
void onAlarmOffClick() {}

void setup(void) {
    Serial.begin(115200);

    pinMode(PIN_SERIAL_TX, OUTPUT);
    pinMode(PIN_LED_START, OUTPUT);

    startScreen();
    resetScreen();
    screen.setCursor(0, 0);
    screen.print("Test FaulHaber");

    buttonAlarmOff.attachClick(onAlarmOffClick);
    buttonStart.attachClick(onStartClick);
    buttonStop.attachClick(onStopClick);
    buttonStart.attachDuringLongPress(onStartClick);
    buttonStop.attachDuringLongPress(onStopClick);

    // set the timer
    testTimer = new HardwareTimer(TIM10);

    // prescaler. stm32f411 clock is 100mhz
    testTimer->setPrescaleFactor((testTimer->getTimerClkFreq() / 10000) - 1);

    // set the period
    testTimer->setOverflow(100);  // 10 ms
    testTimer->setMode(1, TIMER_OUTPUT_COMPARE, NC);
    testTimer->attachInterrupt(Test_Timer_Callback);

    // interrupt priority is documented here:
    // https://stm32f4-discovery.net/2014/05/stm32f4-stm32f429-nvic-or-nested-vector-interrupt-controller/
    testTimer->setInterruptPriority(2, 0);

    

    pinMode(PIN_ESC_BLOWER, OUTPUT);
    pinMode(PIN_SERVO_PATIENT, OUTPUT);
    pinMode(PIN_SERVO_BLOWER, OUTPUT);

#if HARDWARE_VERSION == 2
    // Timer for servos
    hardwareTimer3 = new HardwareTimer(TIM3);
    hardwareTimer3->setOverflow(SERVO_VALVE_PERIOD, MICROSEC_FORMAT);

    // Servo blower setup
    servoBlower = PressureValve(hardwareTimer3, TIM_CHANNEL_SERVO_VALVE_BLOWER, PIN_SERVO_BLOWER,
                                VALVE_OPEN_STATE, VALVE_CLOSED_STATE);
    servoBlower.setup();

    // Servo patient setup
    servoPatient = PressureValve(hardwareTimer3, TIM_CHANNEL_SERVO_VALVE_PATIENT, PIN_SERVO_PATIENT,
                                 VALVE_OPEN_STATE, VALVE_CLOSED_STATE);
    servoPatient.setup();
    hardwareTimer3->resume();

    hardwareTimer1 = new HardwareTimer(TIM1);
    hardwareTimer1->setOverflow(ESC_PPM_PERIOD, MICROSEC_FORMAT);
    blower = Blower(hardwareTimer1, TIM_CHANNEL_ESC_BLOWER, PIN_ESC_BLOWER);
    blower.setup();
#endif

    servoPatient.close();
    servoBlower.close();
    servoBlower.execute();
    servoPatient.execute();

    blower.stop();
    updateBlowerSpeed();

    testTimer->resume(); // start the test timer

}

void loop(void) {
    delay(300);

    char buffer[30];

    int days = clock1s / 86400;
    int hours = (clock1s - 86400 * days) / 3600;
    int minute = (clock1s - 86400 * days - 3600 * hours) / 60;
    int second = (clock1s - 86400 * days - 3600 * hours) % 60;

    resetScreen();
    screen.setCursor(0, 0);
    screen.print("Test Blower");
    screen.setCursor(0, 1);
    snprintf(buffer, sizeof(buffer), "%d days %02d:%02d:%02d", days, hours, minute, second);
    screen.print(buffer);
    screen.setCursor(0, 2);
    screen.print("cycle per minute=");
    screen.print(CYCLEPERMINUTE);
    screen.setCursor(0, 3);
    screen.print("blower power=");
    screen.print(blowerPowerPercent);
    screen.print("%");
}

#endif
