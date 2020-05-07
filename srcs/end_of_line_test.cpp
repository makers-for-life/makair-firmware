/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file end_of_line_test.cpp
 * @brief Auto test for end of line unit test
 *****************************************************************************/

#pragma once

#include "../includes/parameters.h"
#include "Arduino.h"

#include "../includes/end_of_line_test.h"
#include "../includes/screen.h"

uint32_t clockEOLTimer = 0;
uint32_t EolTestNumber = 1;

EolTest::EolTest() {
    testActive = 0;
    eolTimer = new HardwareTimer(TIM10);
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
    screen.print(EolTestNumber);
    if (isFailed) {
        screen.setCursor(16, 0);
        screen.print("FAIL");
    } else {
        screen.setCursor(18, 0);
        screen.print("OK");
    }
    // print line by line, respect newlines.
    int line = 1;
    screen.setCursor(0, line);
    int i = 0;
    while (i < 60) {
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

void millisecondTimerEOL(HardwareTimer*) {
    clockEOLTimer++;
    if (clockEOLTimer % 1000 == 0) {
        eolScreenMessage("test\nmessage", true);
        EolTestNumber++;
    }
}

void EolTest::setupAndStart() {
    // set a 1 ms timer for the event loop
    // prescaler at 10khz. stm32f411 clock is 100mhz.
    eolTimer->setPrescaleFactor((eolTimer->getTimerClkFreq() / 10000) - 1);
    // set the period at 1ms
    eolTimer->setOverflow(10);
    eolTimer->setMode(1, TIMER_OUTPUT_COMPARE, NC);
    eolTimer->attachInterrupt(millisecondTimerEOL);
    eolTimer->resume();
}

EolTest eolTest = EolTest();