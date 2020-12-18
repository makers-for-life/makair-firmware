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
#include "../includes/calibration.h"
#include "../includes/cpu_load.h"
#include "../includes/debug.h"
#include "../includes/end_of_line_test.h"
#include "../includes/keyboard.h"
#include "../includes/main_controller.h"
#include "../includes/main_state_machine.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/parameters.h"
#include "../includes/pressure.h"
#include "../includes/pressure_valve.h"
#include "../includes/rpi_watchdog.h"
#include "../includes/screen.h"
#include "../includes/serial_control.h"
#include "../includes/telemetry.h"

// PROGRAM =====================================================================

HardwareTimer* hardwareTimer1;  // ESC command
HardwareTimer* hardwareTimer3;  // valves command

HardwareSerial Serial6(PIN_TELEMETRY_SERIAL_RX, PIN_TELEMETRY_SERIAL_TX);

void setup(void) {
    // Nothing should be sent to Serial in production, but this will avoid crashing the program if
    // some Serial.print() was forgotten
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
    (void)MFM_init();
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
#if DEBUG != 0
    rpiWatchdog.disable();
#endif

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
    // cppcheck-suppress misra-c2012-14.4 ; IWatchdog.isReset() returns a boolean
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
    Calibration_Init();

    if (!eolTest.isRunning()) {
        // mainStateMachine.setupAndStart();

        // Init the watchdog timer. It must be reloaded frequently otherwise MCU resests
        // IWatchdog.begin(WATCHDOG_TIMEOUT);
        // IWatchdog.reload();
    } else {
        eolTest.setupAndStart();
    }
}

void loop(void) {

    inspiratoryValve.open();
    expiratoryValve.close();

    int32_t blower_values[2] = {1080, 1800};
    int32_t expiratory_angle_values[2] = {0, 0};
    int32_t inspiratory_angle_values[6] = {0, 0, 0, 0, 0, 0};

    Serial.println("index\tblower speed "
                   "(0-1800)\tinspiratoryValveOpenning(0-125)\texpiratoryValveOpenning(0-125)"
                   "\tinspiratoryFlow(mL/"
                   "min)\texpiratoryFlow(mL/min)\tpressure(mmH2O)");
    for (int32_t i = 0; i < 2; i++) {
        blower.runSpeed(blower_values[i]);

        for (int32_t k = 0; k < 2; k++) {
            expiratoryValve.open(expiratory_angle_values[k]);
            for (int32_t l = 0; l < 6; l++) {    
                inspiratoryValve.open(inspiratory_angle_values[l]);
                inspiratoryValve.execute();
                expiratoryValve.execute();
                delay(10000);
                int32_t sumFlow = 0;
                int32_t sumFlowexpi = 0;
                int32_t sumPressure = 0;
                for (int32_t j = 0; j < 100; j++) {
                    expiratoryValve.execute();
                    inspiratoryValve.execute();
                    sumFlow += MFM_read_airflow();
                    sumFlowexpi += MFM_expi_read_airflow();
                    sumPressure += inspiratoryPressureSensor.read();
                    delay(10);
                }
                Serial.print(i);
                Serial.print("\t");
                Serial.print(blower_values[i]);
                Serial.print("\t");
                Serial.print(inspiratory_angle_values[l]);
                Serial.print("\t");
                Serial.print(expiratory_angle_values[k]);
                Serial.print("\t");
                Serial.print(sumFlow / 100);
                Serial.print("\t");
                Serial.print(sumFlowexpi / 100);
                Serial.print("\t");
                Serial.print(sumPressure / 100);
                Serial.println();
            }
        }
    }

    delay(10000000);

    // COUNT_IDLE_CYCLE;
}
#endif
