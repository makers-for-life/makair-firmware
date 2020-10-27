/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file end_of_line_test.cpp
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

#include "../includes/parameters.h"
#include "Arduino.h"

#include "../includes/battery.h"
#include "../includes/buzzer_control.h"
#include "../includes/end_of_line_test.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/pressure.h"
#include "../includes/screen.h"

uint32_t clockEOLTimer = 0;
uint32_t eolMSCount = 0;
uint32_t eolTestNumber = 1;
int32_t pressureValue = 0;
int32_t flowValue = 0;
int32_t lastVolumeValue = 0;
int32_t MinPressureValue = INT32_MAX;
int32_t MaxPressureValue = 0;
HardwareTimer* eolTimer;

EolTest::EolTest() {
    testActive = 0;
    ::eolTimer = new HardwareTimer(TIM9);
}

// cppcheck-suppress unusedFunction
void EolTest::activate() {
    testActive = EOL_TEST_ACTIVE;
    ::clockEOLTimer = 0;
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
    TEST_BAT_DEAD,
    BATTERY_DEEP_DISCHARGE,
    DISCONNECT_MAINS,
    CONNECT_MAINS,
    CHECK_FAN,
    CHECK_BUZZER,
    // cppcheck-suppress misra-c2012-12.3
    CHECK_ALL_BUTTONS,
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
    // cppcheck-suppress misra-c2012-12.3
    START_O2_TEST,
    // cppcheck-suppress misra-c2012-12.3
    O2_PRESSURE_NOT_REACH,
    // cppcheck-suppress misra-c2012-12.3
    CHECK_MASS_FLOW_METER,
    // cppcheck-suppress misra-c2012-12.3
    MASS_FLOW_METER_FAIL,
    // cppcheck-suppress misra-c2012-12.3
    START_LONG_RUN_BLOWER,
    // cppcheck-suppress misra-c2012-12.3
    PRESSURE_NOT_STABLE,
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
#if HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
    clockEOLTimer++;
    eolMSCount++;
    static int batlevel = 0;
    static int minbatlevel = 500;
    static int maxbatlevel = 0;
    static int buttonsPushed[EOL_TOTALBUTTONS];
    if ((clockEOLTimer % 100u) == 0u) {
        // Refresh screen every 100 ms, no more
        eolScreenMessage(eolScreenBuffer, eolFail);
    }

    if (eolstep == START) {
        blower.runSpeed(1799);  // More current
        eolstep = TEST_BAT_DEAD;
        eolMSCount = 0;
    } else if (eolstep == TEST_BAT_DEAD) {
        updateBatterySample();
        batlevel = getBatteryLevelX10();
        if (eolMSCount < 2000u) {
            (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Vbat\n  V=%02d.%d", batlevel / 10,
                           batlevel % 10);
        } else {
            if (batlevel > 255) {
                eolstep = DISCONNECT_MAINS;
            } else if (batlevel < 220) {
                eolstep = BATTERY_DEEP_DISCHARGE;
            } else {
                eolTestNumber++;
                blower.stop();
                eolstep = CONNECT_MAINS;
            }
        }
    } else if (eolstep == BATTERY_DEEP_DISCHARGE) {
        eolFail = true;
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Test Vbat ECHEC\nBATTERIE TROP FAIBLE\n  V=%02d.%d", batlevel / 10,
                       batlevel % 10);
    } else if (eolstep == DISCONNECT_MAINS) {
        updateBatterySample();
        batlevel = getBatteryLevelX10();

        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Vbat\nDebrancher 220V...\n  V=%02d.%d",
                       batlevel / 10, batlevel % 10);
        if (batlevel < 255) {
            eolstep = START;
        }
    } else if (eolstep == CONNECT_MAINS) {
        updateBatterySample();
        batlevel = getBatteryLevelX10();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Vbat\nConnecter 220V...\n  V=%02d.%d",
                       batlevel / 10, batlevel % 10);
        minbatlevel = min(minbatlevel, batlevel);
        maxbatlevel = max(maxbatlevel, batlevel);
        // Wait for 400 mV raise
        if ((maxbatlevel - minbatlevel) > 3) {
            BuzzerControl_On();
            eolTestNumber++;
            blower.stop();
            eolstep = CHECK_BUZZER;
        }
    } else if (eolstep == CHECK_BUZZER) {
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Verifier Buzzer\npuis appuyer sur\nle bouton STOP");
        if (digitalRead(PIN_BTN_STOP) == HIGH) {
            BuzzerControl_Off();
            eolTestNumber++;
            eolstep = CHECK_FAN;
        }
    } else if (eolstep == CHECK_FAN) {
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Verifier Ventilateur\npuis appuyer sur\nle bouton START");
        if (digitalRead(PIN_BTN_START) == HIGH) {
            while (digitalRead(PIN_BTN_START) == HIGH) {
                continue;
            }
            for (int i = 0; i < EOL_TOTALBUTTONS; i++) {
                buttonsPushed[i] = 0;
            }
            eolTestNumber++;
            eolstep = CHECK_ALL_BUTTONS;
        }
    } else if (eolstep == CHECK_ALL_BUTTONS) {
        if (digitalRead(PIN_BTN_ALARM_OFF) == HIGH) {
            buttonsPushed[0] = 1;
        }
#if HARDWARE_VERSION == 2
        // discrete buttons
        if (digitalRead(PIN_BTN_CYCLE_DECREASE) == HIGH) {
            buttonsPushed[1] = 1;
        }
        if (digitalRead(PIN_BTN_CYCLE_INCREASE) == HIGH) {
            buttonsPushed[2] = 1;
        }
        if (digitalRead(PIN_BTN_PEAK_PRESSURE_DECREASE) == HIGH) {
            buttonsPushed[3] = 1;
        }
        if (digitalRead(PIN_BTN_PEAK_PRESSURE_INCREASE) == HIGH) {
            buttonsPushed[4] = 1;
        }
        if (digitalRead(PIN_BTN_PEEP_PRESSURE_DECREASE) == HIGH) {
            buttonsPushed[5] = 1;
        }
        if (digitalRead(PIN_BTN_PEEP_PRESSURE_INCREASE) == HIGH) {
            buttonsPushed[6] = 1;
        }
        if (digitalRead(PIN_BTN_PLATEAU_PRESSURE_DECREASE) == HIGH) {
            buttonsPushed[7] = 1;
        }
        if (digitalRead(PIN_BTN_PLATEAU_PRESSURE_INCREASE) == HIGH) {
            buttonsPushed[8] = 1;
        }
#endif

#if HARDWARE_VERSION == 3
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
#endif

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
            eolstep = PLUG_AIR_TEST_SYTEM;
        }
    } else if (eolstep == PLUG_AIR_TEST_SYTEM) {
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Brancher les tuyaux\nde test et appuyer\nsur START");
        if (digitalRead(PIN_BTN_START) == HIGH) {
            eolMSCount = 0;
            eolstep = REACH_MAX_PRESSURE;
        }
    } else if (eolstep == REACH_MAX_PRESSURE) {
        servoPatient.close();
        servoPatient.execute();
        servoBlower.open();
        servoBlower.execute();
        pressureValue = readPressureSensor(0, pressureOffset);
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
        blower.stop();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Mise sous pression\nimpossible ! ");
    } else if (eolstep == MAX_PRESSURE_REACHED_OK) {
        servoBlower.close();
        servoBlower.execute();
        servoPatient.close();
        servoPatient.execute();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Fermeture valves...");
        if (eolMSCount > 1000u) {
            eolMSCount = 0;
            eolstep = START_LEAK_MESURE;
        }
    } else if (eolstep == START_LEAK_MESURE) {
        blower.stop();
        servoBlower.close();
        servoBlower.execute();
        servoPatient.close();
        servoPatient.execute();
        pressureValue = readPressureSensor(0, pressureOffset);
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
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Fuite importante\nPfinale = %d mmH2O",
                       pressureValue);
    } else if (eolstep == REACH_NULL_PRESSURE) {
        servoPatient.open();
        servoPatient.execute();
        servoBlower.close();
        servoBlower.execute();
        pressureValue = readPressureSensor(0, pressureOffset);
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Ouverture valves...\n  \nP = %d mmH2O",
                       pressureValue);
        if (pressureValue < 20) {
            eolMSCount = 0;
            eolTestNumber++;
            eolstep = START_O2_TEST;
        }
        if (eolMSCount > 10000u) {
            eolMSCount = 0;
            eolFail = true;
            eolstep = MIN_PRESSURE_NOT_REACHED;
        }
    } else if (eolstep == MIN_PRESSURE_NOT_REACHED) {
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Vidage valves\nimpossible ! ");
    } else if (eolstep == START_O2_TEST) {
        blower.runSpeed(1790);
        servoBlower.close();
        servoBlower.execute();
        servoPatient.close();
        servoPatient.execute();
        pressureValue = readPressureSensor(0, pressureOffset);
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test O2...\n  \nP = %d mmH2O",
                       pressureValue);
        if (pressureValue > 100) {
            eolstep = CHECK_MASS_FLOW_METER;
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
        blower.stop();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Tuyau O2\nBouche ! ");

    } else if (eolstep == CHECK_MASS_FLOW_METER) {
        blower.runSpeed(1790);
        servoBlower.open();
        servoBlower.execute();
        servoPatient.open();
        servoPatient.execute();

        // Read the flow-meter every 100ms, using the same function as the main program.
        if (eolMSCount % READ_MASS_FLOW_METER_PERIOD_MS == 0) {
            int32_t volumeValue = MFM_read_milliliters(false);
            flowValue = (1000 * (volumeValue - lastVolumeValue)) / READ_MASS_FLOW_METER_PERIOD_MS;
            lastVolumeValue = volumeValue;
        }

        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test debitmetre\n  \nD = %d mL/s",
                       flowValue);
        // After 2s of stabilisation, check if flow rate is in range [800-1500]mL/s
        if (eolMSCount > 2000 && flowValue > 800 && flowValue < 1500) {
            eolstep = START_LONG_RUN_BLOWER;
            eolTestNumber++;
            eolMSCount = 0;
        }
        // Fail if value is not reached in 20s
        else if (eolMSCount > 20000) {
            eolMSCount = 0;
            eolFail = true;
            eolstep = MASS_FLOW_METER_FAIL;
        } else {
            // Do nothing
        }
    } else if (eolstep == MASS_FLOW_METER_FAIL) {
        blower.stop();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Erreur\nDebitmetre !\n \nD = %d mL/s",
                       flowValue);

    } else if (eolstep == START_LONG_RUN_BLOWER) {
        blower.runSpeed(1790);
        servoBlower.open();
        servoBlower.execute();
        servoPatient.open((servoPatient.minAperture() + servoPatient.maxAperture()) / 2u);
        servoPatient.execute();
        pressureValue = readPressureSensor(0, pressureOffset);
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Stabilite\nblower \n\n P= %d mmH2O",
                       pressureValue);

        if (eolMSCount > 10000u) {
            MaxPressureValue = max(MaxPressureValue, pressureValue);
            MinPressureValue = min(MinPressureValue, pressureValue);
            if ((MaxPressureValue - MinPressureValue) > 25) {
                eolstep = PRESSURE_NOT_STABLE;
                eolMSCount = 0;
            }
        }

        if (eolMSCount > 300000u) {
            if ((MaxPressureValue - MinPressureValue) < 25) {
                eolstep = END_SUCCESS;
                eolMSCount = 0;
                eolTestNumber++;
            } else {
                eolstep = PRESSURE_NOT_STABLE;
                eolMSCount = 0;
            }
        }
    } else if (eolstep == PRESSURE_NOT_STABLE) {
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "Pression non stable\nMax= %d mmH2O \nMin= %d mmH2O", MaxPressureValue,
                       MinPressureValue);
    } else if (eolstep == END_SUCCESS) {
        blower.stop();
        (void)snprintf(eolScreenBuffer, EOLSCREENSIZE,
                       "********************\n**** SUCCESS !! ****\n********************");
    } else {
        // Do nothing
    }

    previousEolStep = eolstep;
#endif
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

    servoPatient.close();
    servoPatient.execute();
    servoBlower.close();
    servoBlower.execute();

#if HARDWARE_VERSION == 3
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
#endif
}

EolTest eolTest = EolTest();
