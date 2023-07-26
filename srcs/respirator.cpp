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
#include <Wire.h>

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
    Serial.begin(1000000);
    DBG_DO(Serial.println("Booting the system...");)

    /*startScreen();

    initBattery();
    if (isBatteryDeepDischarged()) {
        displayBatteryDeepDischarge();

        // Heartbeat fatal error periodically
        while (true) {
            sendBatteryDeeplyDischargedFatalError(getBatteryLevelX100());
            delay(1000);
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

        // Heartbeat fatal error periodically
        while (true) {
            sendWatchdogRestartFatalError();
            delay(1000);
        }
    }*/

    /*initKeyboard();
    BuzzerControl_Init();
    Buzzer_Init();
    Calibration_Init();

    if (!eolTest.isRunning()) {
        mainStateMachine.setupAndStart();

        // Init the watchdog timer. It must be reloaded frequently otherwise MCU resests
        IWatchdog.begin(WATCHDOG_TIMEOUT);
        IWatchdog.reload();
    } else {
        eolTest.setupAndStart();
    }*/

    // Init the watchdog timer. It must be reloaded frequently otherwise MCU resests
    if (IWatchdog.isReset(true)) {

        while (true) {
            Serial.println("Watchdog");
            delay(1000);
        }
    }
    IWatchdog.begin(WATCHDOG_TIMEOUT);
    IWatchdog.reload();

    Wire.begin();
    Wire.setClock(400000);

    // Initialisation des capteurs
    int errorCount = 0;
    for (int l = 0; l < 2; l++) {
        while (1) {
            IWatchdog.reload();
            // Wire.begin();
            Wire.beginTransmission(0x70);
            Wire.write(1 << l);
            Wire.endTransmission();
            delay(10);
            Wire.beginTransmission(MFM_SFM_3300D_I2C_ADDRESS);
            Wire.write(0x20);  // 0x2000 soft reset
            Wire.write(0x00);
            errorCount = Wire.endTransmission();
            delay(5);  // end of reset
            // Serial.print("debut error count");
            // Serial.println(errorCount);

            Wire.beginTransmission(MFM_SFM_3300D_I2C_ADDRESS);
            Wire.write(0x31);  // 0x31AE read serial
            Wire.write(0xAE);
            errorCount += Wire.endTransmission();
            // Serial.print("millieu error count");
            // Serial.println(errorCount);
            delay(10);

            errorCount += ((6u == Wire.requestFrom(MFM_SFM_3300D_I2C_ADDRESS, 6)) ? 0u : 1u);
            // Serial.print("fin error count");
            // Serial.println(errorCount);
            if (errorCount == 0u) {
                u_int32_t sn_expi = 0;
                sn_expi = Wire.read();
                sn_expi <<= 8;
                sn_expi |= Wire.read();
                sn_expi <<= 8;
                Wire.read();  // ignore inlined crc
                sn_expi |= Wire.read();
                sn_expi <<= 8;
                sn_expi |= Wire.read();
                Wire.read();  // ignore inlined crc
                Serial.println(sn_expi);
                delay(10);
                Wire.beginTransmission(MFM_SFM_3300D_I2C_ADDRESS);
                Wire.write(0x10);  // 0x1000 start measurement
                Wire.write(0x00);
                errorCount += Wire.endTransmission();
                // Wire.end();
                delay(1000);  // wait 100ms before having available data.
                Serial.print("Ok init flowmeter: ");
                Serial.println(l);
                break;
            } else {
                Serial.println("Error reading serial number");
                delay(1000);
            }
        }
    }
}

// cppcheck-suppress unusedFunction
void loop(void) {

    // Interrogation des capteurs
    
    for (int l = 0; l < 2; l++) {  // on itère sur les deux voies I2C
        IWatchdog.reload();
        // Serial.print('-->');
        Wire.begin();
        Wire.beginTransmission(0x70);
        Wire.write(1 << l);
        Wire.endTransmission();
        delay(10);
        uint8_t id = 0x28;                    // i2c address
        uint8_t data[7];                      // holds output data
        uint8_t cmd[3] = {0xAA, 0x00, 0x00};  // command to be sent
        double press_counts = 0;              // digital pressure reading [counts]
        double temp_counts = 0;               // digital temperature reading [counts]
        double pressure = 0;                  // pressure reading [bar, psi, kPa, etc.]
        double temperature = 0;               // temperature reading in deg C
        double outputmax = 15099494;          // output at maximum pressure [counts]
        double outputmin = 1677722;           // output at minimum pressure [counts]
        double pmax = 1;        // maximum value of pressure range [bar, psi, kPa, etc.]
        double pmin = 0;        // minimum value of pressure range [bar, psi, kPa, etc.]
        double percentage = 0;  // holds percentage of full scale data
        char printBuffer[200], cBuff[20], percBuff[20], pBuff[20], tBuff[20];
        Wire.beginTransmission(0x28);
        Wire.write(0xaa);  // write command to the sensor
        Wire.write(0x00);  // write command to the sensor
        Wire.write(0x00);  // write command to the senso
        Wire.endTransmission();
         delay(1);
        int count = Wire.requestFrom(0x28, 7);  // read back Sensor data 7 bytes
        // Serial.print(count);
        // Serial.print(" ");
        int i = 0;
        for (i = 0; i < 7; i++) {
            if (Wire.available()) {
                data[i] = Wire.read();
            }

            // Serial.print(data[i]);
            // Serial.print("  ");
        }
        press_counts =
            data[3] + data[2] * 256 + data[1] * 65536;  // calculate digital pressure counts
        temp_counts =
            data[6] + data[5] * 256 + data[4] * 65536;      // calculate digital temperature counts
        temperature = (temp_counts * 200 / 16777215) - 50;  // calculate temperature in deg c
        // calculation of pressure value according to equation 2 of datasheet
        pressure = ((press_counts - outputmin) * (pmax - pmin)) / (outputmax - outputmin) + pmin;
        Serial.print(pressure, 5);
        Serial.print(",");
        // Wire.end();
        // delay(10);

        // Wire.begin();
        //  do not request crc, only two bytes
        uint8_t readCountExpi = Wire.requestFrom(MFM_SFM_3300D_I2C_ADDRESS, 2);
        unsigned char a, b;
        if (Wire.available()) {
            a = Wire.read();
        }
        if (Wire.available()) {
            b = Wire.read();
        }
        // unsigned char b = Wire.read();
          

        uint16_t rawFlow = a << 8 | b;
        // Serial.print(rawFlow);
        //  Serial.print("  ");
        Serial.print(((int32_t)(rawFlow)-32768) * 1000 / 120);
        Serial.print(",");
        // delay(10);
        // conversion in milliter per minute flow: ((int32_t)(mfmLastData.i) - 32768) * 1000 /
        // 120 but 1000/120 = 8.333. So  *8 and *1/3*/
    }
    Serial.println("");
    //Wire.end();
}

#endif
