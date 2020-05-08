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
#include "../includes/pressure.h"
#include "../includes/screen.h"

uint32_t clockEOLTimer = 0;
uint32_t eolMSCount = 0;
uint32_t eolTestNumber = 1;
HardwareTimer* eolTimer;

EolTest::EolTest() {
    testActive = 0;
    ::eolTimer = new HardwareTimer(TIM10);
}

void EolTest::activate() {
    testActive = EOL_TEST_ACTIVE;
    ::clockEOLTimer = 0;
}

bool EolTest::isRunning() { return (EOL_TEST_ACTIVE == testActive); }

// Message display helper function
void eolScreenMessage(char* message, boolean isFailed) {
    screen.clear();
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
    // print line by line, respect newlines.
    int line = 1;
    screen.setCursor(0, line);
    int i = 0;
    while (i < 62) {
        if (message[i] == '\n') {
            line++;
            screen.setCursor(0, line);
            i++;
        }
        if (message[i] == 0 || line > 3) {
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
    CHECK_ALL_BUTTONS,
    PLUG_AIR_TEST_SYTEM,
    REACH_MAX_PRESSURE,
    MAX_PRESSURE_REACHED_OK,
    MAX_PRESSURE_NOT_REACHED,
    START_LEAK_MESURE,
    END_SUCCESS
};
TestStep eolstep = PLUG_AIR_TEST_SYTEM;
TestStep previousEolStep = START;
boolean eolFail = false;
#define EOLSCREENSIZE 100
char eolScreenBuffer[EOLSCREENSIZE + 1] = "";
#define EOL_TOTALBUTTONS 11

void millisecondTimerEOL(HardwareTimer*) {
    clockEOLTimer++;
    eolMSCount++;
    static int batlevel = 0;
    static int minbatlevel = 500;
    static int maxbatlevel = 0;
    static int buttonsPushed[EOL_TOTALBUTTONS];
    if (clockEOLTimer % 100 == 0) {
        // refresh screen every 100ms, no more.
        eolScreenMessage(eolScreenBuffer, eolFail);
    }

    if (eolstep == START) {
        blower.runSpeed(1799);  // more current
        eolstep = TEST_BAT_DEAD;
        eolMSCount = 0;
    } else if (eolstep == TEST_BAT_DEAD) {
        updateBatterySample();
        batlevel = getBatteryLevelX10();
        if (eolMSCount < 2000) {
            snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Vbat\n  V=%02d.%d", batlevel / 10,
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
        snprintf(eolScreenBuffer, EOLSCREENSIZE,
                 "Test Vbat ECHEC\nBATTERIE TROP FAIBLE\n  V=%02d.%d", batlevel / 10,
                 batlevel % 10);
    } else if (eolstep == DISCONNECT_MAINS) {
        updateBatterySample();
        batlevel = getBatteryLevelX10();

        snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Vbat\nDebrancher 220V...\n  V=%02d.%d",
                 batlevel / 10, batlevel % 10);
        if (batlevel < 255) {
            eolstep = START;
        }
    } else if (eolstep == CONNECT_MAINS) {
        updateBatterySample();
        batlevel = getBatteryLevelX10();
        snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Vbat\nConnecter 220V...\n  V=%02d.%d",
                 batlevel / 10, batlevel % 10);
        minbatlevel = min(minbatlevel, batlevel);
        maxbatlevel = max(maxbatlevel, batlevel);
        // wait for 400mV raise
        if ((maxbatlevel - minbatlevel) > 3) {
            BuzzerControl_On();
            eolTestNumber++;
            blower.stop();
            eolstep = CHECK_BUZZER;
        }
    } else if (eolstep == CHECK_BUZZER) {
        snprintf(eolScreenBuffer, EOLSCREENSIZE,
                 "Verifier Buzzer\npuis appuyer sur\nle bouton STOP");
        if (digitalRead(PIN_BTN_STOP) == HIGH) {
            BuzzerControl_Off();
            eolTestNumber++;
            eolstep = CHECK_FAN;
        }
    } else if (eolstep == CHECK_FAN) {
        snprintf(eolScreenBuffer, EOLSCREENSIZE,
                 "Verifier Ventilateur\npuis appuyer sur\nle bouton START");
        if (digitalRead(PIN_BTN_START) == HIGH) {
            while (digitalRead(PIN_BTN_START) == HIGH)
                continue;
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
        snprintf(eolScreenBuffer, EOLSCREENSIZE, "Appuyer sur chaque\nbouton... \n %d sur 11 OK",
                 totalPushed);
        if (totalPushed == EOL_TOTALBUTTONS) {
            eolTestNumber++;
            while (digitalRead(PIN_BTN_START) == HIGH)
                continue;  // wait release if still pressed in previous test
            eolstep = PLUG_AIR_TEST_SYTEM;
        }
    } else if (eolstep == PLUG_AIR_TEST_SYTEM) {
        snprintf(eolScreenBuffer, EOLSCREENSIZE,
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
        int16_t pressureValue = readPressureSensor(0, pressureOffset);
        blower.runSpeed(1790);
        snprintf(eolScreenBuffer, EOLSCREENSIZE, "Mise sous pression\n  \nP = %d mmH2O",
                 pressureValue);
        if (pressureValue > 650) {
            eolMSCount = 0;
            eolstep = MAX_PRESSURE_REACHED_OK;
        }
        if (eolMSCount > 20000) {
            eolFail = true;
            eolstep = MAX_PRESSURE_NOT_REACHED;
        }
    } else if (eolstep == MAX_PRESSURE_NOT_REACHED) {
        blower.stop();
        snprintf(eolScreenBuffer, EOLSCREENSIZE, "Mise sous pression\nimpossible ! ");
    } else if (eolstep == MAX_PRESSURE_REACHED_OK) {
        servoBlower.close();
        servoBlower.execute();
        snprintf(eolScreenBuffer, EOLSCREENSIZE, "Fermeture valves...");
        if (eolMSCount > 500) {
            eolMSCount = 0;
            eolstep = START_LEAK_MESURE;
        }
    } else if (eolstep == START_LEAK_MESURE) {
        blower.stop();
        int16_t pressureValue = readPressureSensor(0, pressureOffset);

        snprintf(eolScreenBuffer, EOLSCREENSIZE, "Test Fuite...\n  \nP = %d mmH2O", pressureValue);

    }

    else if (eolstep == END_SUCCESS) {
        snprintf(eolScreenBuffer, EOLSCREENSIZE,
                 "********************\n**** SUCCESS !! ****\n********************");
    }

    previousEolStep = eolstep;
}

void EolTest::setupAndStart() {
    // set a 1 ms timer for the event loop
    // prescaler at 10khz. stm32f411 clock is 100mhz.
    ::eolTimer->setPrescaleFactor((::eolTimer->getTimerClkFreq() / 10000) - 1);
    // set the period at 1ms
    ::eolTimer->setOverflow(10);
    ::eolTimer->setMode(1, TIMER_OUTPUT_COMPARE, NC);
    ::eolTimer->attachInterrupt(millisecondTimerEOL);
    ::eolTimer->resume();

    servoPatient.close();
    servoPatient.execute();
    servoBlower.close();
    servoBlower.execute();
    // define all input and output
}

EolTest eolTest = EolTest();
