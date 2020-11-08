/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file end_of_line_test.cpp
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

#include "../includes/parameters.h"
#include "Arduino.h"

#include "../includes/battery.h"
#include "../includes/buzzer_control.h"
#include "../includes/end_of_line_test.h"
#include "../includes/main_controller.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/pressure.h"
#include "../includes/screen.h"
#include "../includes/serial_control.h"
#include "../includes/telemetry.h"

// INITIALISATION =============================================================

uint32_t clockEOLTimer = 0;
uint32_t eolMSCount = 0;
uint32_t eolTestNumber = 1;
int32_t pressureValue = 0;
int32_t flowValue = 0;
int32_t minPressureValue = INT32_MAX;
int32_t maxPressureValue = 0;
int32_t minFlowValue = INT32_MAX;
int32_t maxFlowValue = 0;

EolTest eolTest = EolTest();
HardwareTimer* eolTimer;

// FUNCTIONS ==================================================================

EolTest::EolTest() { testActive = 0; }

// cppcheck-suppress unusedFunction
void EolTest::activate() {
    testActive = EOL_TEST_ACTIVE;
    ::clockEOLTimer = 0;
    ::eolTimer = new HardwareTimer(TIM9);
}

bool EolTest::isRunning() { return (EOL_TEST_ACTIVE == testActive); }

// Message display helper function
// cppcheck-suppress unusedFunction
void eolScreenMessage(char* message, bool isFailed) {
    screen.clear();
    // cppcheck-suppress misra-c2012-12.3
    screen.setCursor(0, 0);
    screen.print("EOL TEST  #");
    screen.print(eolTestNumber);
    if (isFailed) {
        screen.setCursor(15, 0);
        screen.print("ECHEC");
    } else {
        screen.setCursor(18, 0);
        screen.print("OK");
    }

    // Print line by line, respect newlines
    int line = 1;
    screen.setCursor(0, line);
    int i = 0;
    while (i < 62) {
        if (message[i] == '\n') {
            line++;
            screen.setCursor(0, line);
            i++;
        }
        if ((message[i] == 0) || (line > 3)) {
            break;
        }
        screen.print(message[i]);
        i++;
    }
}

enum TestStep {
    START,
    CHECK_FAN,
    TEST_BAT_DEAD,
    BATTERY_DEEP_DISCHARGE,
    DISCONNECT_MAINS,
    CONNECT_MAINS,
    CHECK_BUZZER,
    // cppcheck-suppress misra-c2012-12.3
    CHECK_ALL_BUTTONS,
    CHECK_UI_SCREEN,
    // cppcheck-suppress misra-c2012-12.3
    PLUG_AIR_TEST_SYTEM,
    // cppcheck-suppress misra-c2012-12.3
    REACH_MAX_PRESSURE,
    // cppcheck-suppress misra-c2012-12.3
    MAX_PRESSURE_REACHED_OK,
    // cppcheck-suppress misra-c2012-12.3
    MAX_PRESSURE_NOT_REACHED,
    // cppcheck-suppress misra-c2012-12.3
    START_LEAK_MESURE,
    // cppcheck-suppress misra-c2012-12.3
    LEAK_IS_TOO_HIGH,
    // cppcheck-suppress misra-c2012-12.3
    REACH_NULL_PRESSURE,
    // cppcheck-suppress misra-c2012-12.3
    MIN_PRESSURE_NOT_REACHED,
    USER_CONFIRMATION_BEFORE_O2_TEST,
    // cppcheck-suppress misra-c2012-12.3
    START_O2_TEST,
    // cppcheck-suppress misra-c2012-12.3
    O2_PRESSURE_NOT_REACH,
    // cppcheck-suppress misra-c2012-12.3
    START_LONG_RUN_BLOWER,
    // cppcheck-suppress misra-c2012-12.3
    PRESSURE_NOT_STABLE,
    FLOW_NOT_STABLE,
    END_SUCCESS
};

TestStep eolstep = START;
TestStep previousEolStep = START;
boolean eolFail = false;
#define EOLSCREENSIZE 100
char eolScreenBuffer[EOLSCREENSIZE + 1];
#define EOL_TOTALBUTTONS 11
int16_t eolMatrixCurrentColumn = 1;

// cppcheck-suppress misra-c2012-2.7 ; valid unused parameter
void millisecondTimerEOL(HardwareTimer*) {
    clockEOLTimer++;
    eolMSCount++;
    static int batlevel = 0;
    static int minbatlevel = 5000;
    static int maxbatlevel = 0;
    static int buttonsPushed[EOL_TOTALBUTTONS];
    if ((clockEOLTimer % 100u) == 0u) {
        // Refresh screen every 100 ms, no more
        eolScreenMessage(eolScreenBuffer, eolFail);
    }

    batteryLoop(0);

    // First step: reset the step count
    if (eolstep == START) {
        eolstep = CHECK_FAN;
        eolMSCount = 0;
        mainController.setup();
    } else if (eolstep == CHECK_FAN) {
        // The operator should check that both fans are running, and hit "start" to confirm
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Verifier Ventilateurs\npuis appuyer sur\nle bouton START");
        if (digitalRead(PIN_BTN_START) == HIGH) {
            while (digitalRead(PIN_BTN_START) == HIGH) {
                continue;
            }
            eolTestNumber++;
            eolstep = TEST_BAT_DEAD;
        }
    } else if (eolstep == TEST_BAT_DEAD) {
        // Check if the voltage is acceptable
        blower.runSpeed(1799);  // Run blower to drain more current
        batlevel = getBatteryLevelX100();
        if (eolMSCount < 2000u) {
            (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Vbat\n  V=%02d.%02d",
                           batlevel / 100, batlevel % 100);
        } else {
            if (isMainsConnected()) {
                eolstep = DISCONNECT_MAINS;
            } else if (batlevel < 2200) {  // Test if battery is under 22V
                eolstep = BATTERY_DEEP_DISCHARGE;
            } else {
                eolTestNumber++;
                blower.stop();
                eolstep = CONNECT_MAINS;
            }
        }
    } else if (eolstep == BATTERY_DEEP_DISCHARGE) {
        // FAIL: Battery voltage is too low
        eolFail = true;
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Test Vbat ECHEC\nBATTERIE TROP FAIBLE\n  V=%02d.%d", batlevel / 100,
                       batlevel % 100);
    } else if (eolstep == DISCONNECT_MAINS) {
        // Ask the operator to unplug the machine
        batlevel = getBatteryLevelX100();

        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Test Vbat\nDebrancher 220V...\n  V=%02d.%02d", batlevel / 100,
                       batlevel % 100);
        if (!isMainsConnected()) {
            eolstep = TEST_BAT_DEAD;
        }
    } else if (eolstep == CONNECT_MAINS) {
        // Ask the operator to reconnect the machine and wait for a voltage raise of 0.4 V
        batlevel = getBatteryLevelX100();

        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Vbat\nConnecter 220V...\nV=%02d.%02d",
                       batlevel / 100, batlevel % 100);
        minbatlevel = min(minbatlevel, batlevel);
        maxbatlevel = max(maxbatlevel, batlevel);
        // Wait for 400 mV raise, or mains connected signal
        if ((maxbatlevel - minbatlevel) > 40 || isMainsConnected()) {
            BuzzerControl_On();
            eolTestNumber++;
            blower.stop();
            eolstep = CHECK_BUZZER;
        }
    } else if (eolstep == CHECK_BUZZER) {
        // Run the buzzer (previous step) and ask the operator to hit the STOP button
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Verifier Buzzer\npuis appuyer sur\nle bouton STOP");
        if (digitalRead(PIN_BTN_STOP) == HIGH) {
            BuzzerControl_Off();
            eolTestNumber++;
            eolstep = CHECK_ALL_BUTTONS;
        }
    } else if (eolstep == CHECK_ALL_BUTTONS) {
        // Ask the operator to hit each button
        if (digitalRead(PIN_BTN_ALARM_OFF) == HIGH) {
            buttonsPushed[0] = 1;
        }
        // this buttons are in a matrix
        if (1 == eolMatrixCurrentColumn) {
            // Increase counter for each column and row
            if (HIGH == digitalRead(PIN_IN_ROW1)) {
                buttonsPushed[1] = 1;
            }
            if (HIGH == digitalRead(PIN_IN_ROW2)) {
                buttonsPushed[2] = 1;
            }
            if (HIGH == digitalRead(PIN_IN_ROW3)) {
                buttonsPushed[3] = 1;
            }
        } else if (2 == eolMatrixCurrentColumn) {
            if (HIGH == digitalRead(PIN_IN_ROW1)) {
                buttonsPushed[4] = 1;
            }
            if (HIGH == digitalRead(PIN_IN_ROW2)) {
                buttonsPushed[5] = 1;
            }
            if (HIGH == digitalRead(PIN_IN_ROW3)) {
                buttonsPushed[6] = 1;
            }
        } else if (3 == eolMatrixCurrentColumn) {
            if (HIGH == digitalRead(PIN_IN_ROW1)) {
                buttonsPushed[7] = 1;
            }
            if (HIGH == digitalRead(PIN_IN_ROW2)) {
                buttonsPushed[8] = 1;
            }
            // there is no button on col3 x row3
        }
        // next column
        eolMatrixCurrentColumn++;
        if (4 == eolMatrixCurrentColumn) {
            eolMatrixCurrentColumn = 1;
        }
        digitalWrite(PIN_OUT_COL1, 1 == eolMatrixCurrentColumn ? HIGH : LOW);
        digitalWrite(PIN_OUT_COL2, 2 == eolMatrixCurrentColumn ? HIGH : LOW);
        digitalWrite(PIN_OUT_COL3, 3 == eolMatrixCurrentColumn ? HIGH : LOW);

        if (digitalRead(PIN_BTN_START) == HIGH) {
            buttonsPushed[9] = 1;
        }
        if (digitalRead(PIN_BTN_STOP) == HIGH) {
            buttonsPushed[10] = 1;
        }
        int totalPushed = 0;
        for (int i = 0; i < EOL_TOTALBUTTONS; i++) {
            totalPushed += buttonsPushed[i];
        }
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Appuyer sur chaque\nbouton... \n %d sur 11 OK", totalPushed);
        if (totalPushed == EOL_TOTALBUTTONS) {
            eolTestNumber++;
            while (digitalRead(PIN_BTN_START) == HIGH) {
                continue;  // Wait release if still pressed in previous test
            }
            eolstep = CHECK_UI_SCREEN;
        }
    } else if (eolstep == CHECK_UI_SCREEN) {
        // Ask the operator to activate the trigger on the UI screen. It allows to test
        // communication with the UI, and tactile function of the UI.
        mainController.sendStopMessageToUi();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Activer le trigger\nSur ecran");

        // Check serial from the UI
        serialControlLoop();

        if (mainController.triggerModeEnabledNextCommand()) {
            eolstep = PLUG_AIR_TEST_SYTEM;
            eolMSCount = 0;
        }
    } else if (eolstep == PLUG_AIR_TEST_SYTEM) {
        // Ask the operator to plug the lung system on the machine, and wait the operator to press
        // start
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Brancher les tuyaux\nde test et appuyer\nsur START");
        if (digitalRead(PIN_BTN_START) == HIGH) {
            eolMSCount = 0;
            eolstep = REACH_MAX_PRESSURE;
        }
    } else if (eolstep == REACH_MAX_PRESSURE) {
        // Turn on the blower and check if able to reach the max pressure 650 mmH2O
        expiratoryValve.close();
        expiratoryValve.execute();
        inspiratoryValve.open();
        inspiratoryValve.execute();
        pressureValue = inspiratoryPressureSensor.read();
        blower.runSpeed(1790);
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Mise sous pression\n  \nP = %d mmH2O",
                       pressureValue);
        if (pressureValue > 650) {
            eolMSCount = 0;
            eolTestNumber++;
            eolstep = MAX_PRESSURE_REACHED_OK;
        }
        if (eolMSCount > 20000u) {
            eolFail = true;
            eolstep = MAX_PRESSURE_NOT_REACHED;
        }
    } else if (eolstep == MAX_PRESSURE_NOT_REACHED) {
        // FAIL: Case max pressure was not reached
        blower.stop();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Mise sous pression\nimpossible ! ");
    } else if (eolstep == MAX_PRESSURE_REACHED_OK) {
        // Close the valves and wait 1000 ms
        inspiratoryValve.close();
        inspiratoryValve.execute();
        expiratoryValve.close();
        expiratoryValve.execute();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Fermeture valves...");
        if (eolMSCount > 1000u) {
            eolMSCount = 0;
            eolstep = START_LEAK_MESURE;
        }
    } else if (eolstep == START_LEAK_MESURE) {
        // Stop the blower and measure leak with the pressure sensor
        blower.stop();
        inspiratoryValve.close();
        inspiratoryValve.execute();
        expiratoryValve.close();
        expiratoryValve.execute();
        pressureValue = inspiratoryPressureSensor.read();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Fuite...\n  \nP = %d mmH2O",
                       pressureValue);
        if (eolMSCount > 10000u) {
            eolMSCount = 0;
            if (pressureValue > 400) {
                eolstep = REACH_NULL_PRESSURE;
                eolTestNumber++;
            } else {
                eolFail = true;
                eolstep = LEAK_IS_TOO_HIGH;
            }
        }
    } else if (eolstep == LEAK_IS_TOO_HIGH) {
        // FAIL: Case Leak is too high
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Fuite importante\nPfinale = %d mmH2O",
                       pressureValue);
    } else if (eolstep == REACH_NULL_PRESSURE) {
        // Open the valves to empty the lung system
        expiratoryValve.open();
        expiratoryValve.execute();
        inspiratoryValve.close();
        inspiratoryValve.execute();
        pressureValue = inspiratoryPressureSensor.read();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Ouverture valves...\n  \nP = %d mmH2O",
                       pressureValue);
        if (pressureValue < 20) {
            eolMSCount = 0;
            eolTestNumber++;
            eolstep = USER_CONFIRMATION_BEFORE_O2_TEST;
        }
        if (eolMSCount > 10000u) {
            eolMSCount = 0;
            eolFail = true;
            eolstep = MIN_PRESSURE_NOT_REACHED;
        }
    } else if (eolstep == MIN_PRESSURE_NOT_REACHED) {
        // FAIL: Case emptying the system did not work
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Vidage valves\nimpossible ! ");
    } else if (eolstep == USER_CONFIRMATION_BEFORE_O2_TEST) {
        // Ask the operator to open the oxygen entrance, and wait for confirmation
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Ouvrir oxygene\npuis appuyer sur\nle bouton START");
        if (digitalRead(PIN_BTN_START) == HIGH) {
            while (digitalRead(PIN_BTN_START) == HIGH) {
                continue;
            }
            eolMSCount = 0;
            eolTestNumber++;
            eolstep = START_O2_TEST;
        }
    } else if (eolstep == START_O2_TEST) {
        // Close the valves, run the blower, and wait for pressure to go above 100 mmH2O
        blower.runSpeed(1790);
        inspiratoryValve.close();
        inspiratoryValve.execute();
        expiratoryValve.close();
        expiratoryValve.execute();
        pressureValue = inspiratoryPressureSensor.read();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test O2...\n  \nP = %d mmH2O",
                       pressureValue);
        if (pressureValue > 100) {
            eolstep = START_LONG_RUN_BLOWER;
            eolTestNumber++;
            eolMSCount = 0;
        } else if (eolMSCount > 20000u) {
            eolMSCount = 0;
            eolFail = true;
            eolstep = O2_PRESSURE_NOT_REACH;
        } else {
            // Do nothing
        }
    } else if (eolstep == O2_PRESSURE_NOT_REACH) {
        // FAIL: the pressure did not bo above 100 mmh2O during O2 test
        blower.stop();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Tuyau O2\nBouche ! ");
    } else if (eolstep == START_LONG_RUN_BLOWER) {
        // Run the blower during 5 minutes and check stability
        blower.runSpeed(1790);
        inspiratoryValve.open();
        inspiratoryValve.execute();
        expiratoryValve.open((expiratoryValve.minAperture() + expiratoryValve.maxAperture()) / 2u);
        expiratoryValve.execute();
        pressureValue = inspiratoryPressureSensor.read();
        flowValue = MFM_read_airflow();

        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Stabilite\nblower \n\n P= %d mmH2O",
                       pressureValue);

        if (eolMSCount > 10000u) {
            maxPressureValue = max(maxPressureValue, pressureValue);
            minPressureValue = min(minPressureValue, pressureValue);
            maxFlowValue = max(maxFlowValue, flowValue);
            minFlowValue = min(minFlowValue, flowValue);
            if ((maxPressureValue - minPressureValue) > 40) {  // 40 mmH2O
                eolstep = PRESSURE_NOT_STABLE;
                eolMSCount = 0;
            }
            if ((maxFlowValue - minFlowValue) > 5000) {  // 5000 mL/min
                eolstep = FLOW_NOT_STABLE;
                eolMSCount = 0;
            }
        }

        if (eolMSCount > 60000u) {
            if ((maxPressureValue - minPressureValue) < 40) {
                eolstep = END_SUCCESS;
                eolMSCount = 0;
                eolTestNumber++;
            } else {
                eolstep = PRESSURE_NOT_STABLE;
                eolMSCount = 0;
            }
        }
    } else if (eolstep == PRESSURE_NOT_STABLE) {
        // FAIL: pressure was not stable during long run test
        blower.stop();
        inspiratoryValve.open();
        inspiratoryValve.execute();
        expiratoryValve.open();
        expiratoryValve.execute();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Pression non stable\nMax= %d mmH2O \nMin= %d mmH2O", maxPressureValue,
                       minPressureValue);
    } else if (eolstep == FLOW_NOT_STABLE) {
        // FAIL: flow was not stable during long run test
        blower.stop();
        inspiratoryValve.open();
        inspiratoryValve.execute();
        expiratoryValve.open();
        expiratoryValve.execute();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Flow non stable\nMax= %d SLM \nMin= %d SLM",
                       maxFlowValue, minFlowValue);
    } else if (eolstep == END_SUCCESS) {
        // SUCESS: end of the procedure
        blower.stop();
        inspiratoryValve.open();
        inspiratoryValve.execute();
        expiratoryValve.open();
        expiratoryValve.execute();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "********************\n**** SUCCESS !! ****\n********************");
    } else {
        // Do nothing
    }

    previousEolStep = eolstep;
}

void EolTest::setupAndStart() {
    // Set a 1 ms timer for the event loop
    // Prescaler at 10 kHz; stm32f411 clock is 100 mHz
    ::eolTimer->setPrescaleFactor((::eolTimer->getTimerClkFreq() / 10000) - 1);
    // Set the period at 1 ms
    ::eolTimer->setOverflow(10);
    ::eolTimer->setMode(1, TIMER_OUTPUT_COMPARE, NC);
    ::eolTimer->attachInterrupt(millisecondTimerEOL);
    ::eolTimer->resume();

    expiratoryValve.close();
    expiratoryValve.execute();
    inspiratoryValve.close();
    inspiratoryValve.execute();

    // define the 3x3 matrix keyboard input and output
    pinMode(PIN_OUT_COL1, OUTPUT);
    pinMode(PIN_OUT_COL2, OUTPUT);
    pinMode(PIN_OUT_COL3, OUTPUT);
    digitalWrite(PIN_OUT_COL1, LOW);
    digitalWrite(PIN_OUT_COL2, LOW);
    digitalWrite(PIN_OUT_COL3, LOW);
    pinMode(PIN_IN_ROW1, INPUT);
    pinMode(PIN_IN_ROW2, INPUT);
    pinMode(PIN_IN_ROW3, INPUT);
}
