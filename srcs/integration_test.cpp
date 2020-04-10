/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file qualification.cpp
 * @brief Entry point of electrical wiring qualification program
 *****************************************************************************/

#pragma once
 
#include "../includes/config.h"
#if MODE == MODE_INTEGRATION_TEST

// INCLUDES ===================================================================

// External
#include <AnalogButtons.h>
#include <Arduino.h>
#include <IWatchdog.h>
#include <OneButton.h>

// Internal
#include "../includes/blower.h"
#include "../includes/debug.h"
#include "../includes/parameters.h"
#include "../includes/pression.h"
#include "../includes/pressure_valve.h"
#include "../includes/screen.h"
#include "../includes/battery.h"
#include "../includes/buzzer.h"

/**
 * Liste de toutes les étapes de test du montage.
 */
#define STEP_WELCOME 0
#define STEP_BLOWER_TEST 1
#define STEP_VALVE_BLOWER_TEST 2
#define STEP_VALVE_PATIENT_TEST 3
#define STEP_VALVE_BLOWER_LEAK_TEST 4
#define STEP_VALVE_PATIENT_LEAK_TEST 5


#define UNGREEDY(is_drawn, statement)                                                              \
    if (is_drawn == 0) {                                                                           \
        statement;                                                                                 \
        is_drawn = 1;                                                                              \
    }

static uint8_t step = STEP_WELCOME;
static uint8_t is_drawn = false;

PressureValve servoBlower;
PressureValve servoPatient;
HardwareTimer* hardwareTimer1;
HardwareTimer* hardwareTimer3;
Blower blower;

int32_t last_time = millis();

void changeStep(uint8_t new_step) {
    step = new_step;
    is_drawn = 0;
}


//! This function displays 2 lines of 20 characters (or less)
void display(char line1[], char line2[]) {
    resetScreen();
    screen.setCursor(0, 0);
    screen.print(line1);
    screen.setCursor(0, 1);
    screen.print(line2);
}

//! This function displays only a message on line
void displayLine(char msg[], uint8_t line) {
    screen.setCursor(0, line);
    screen.print("                    ");
    screen.setCursor(0, line);
    screen.print(msg);
}

void onPressionCretePlusClick() {
    DBG_DO(Serial.println("pression crete ++"));
   
}

void onPressionCreteMinusClick() {
    DBG_DO(Serial.println("pression crete --"));
    
}

void onStartClick() {
    DBG_DO(Serial.print("Go to step: "));
    DBG_DO(Serial.println((step+1)%6));
    changeStep((step+1)%6);
    last_time = millis();
    blower.stop();

}



static AnalogButtons analogButtons(PIN_CONTROL_BUTTONS, INPUT);

Button btn_pression_crete_plus =
    Button(VOLTAGE_BUTTON_PEAK_PRESSURE_INCREASE, &onPressionCretePlusClick);
Button btn_pression_crete_minus =
    Button(VOLTAGE_BUTTON_PEAK_PRESSURE_DECREASE, &onPressionCreteMinusClick);
OneButton btn_start(PIN_BTN_START, false, false);


void setup() {
    DBG_DO(Serial.begin(115200);)
    DBG_DO(Serial.println("Booting the system in integration mode...");)

    analogButtons.add(btn_pression_crete_plus);
    analogButtons.add(btn_pression_crete_minus);
    btn_start.attachClick(onStartClick);

    startScreen();

    pinMode(PIN_PRESSURE_SENSOR, INPUT);
    pinMode(PIN_BATTERY, INPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    // Timer for servoBlower
    hardwareTimer1 = new HardwareTimer(TIM1);
    hardwareTimer1->setOverflow(SERVO_VALVE_PERIOD, MICROSEC_FORMAT);

    // Timer for servoPatient and escBlower
    hardwareTimer3 = new HardwareTimer(TIM3);
    hardwareTimer3->setOverflow(SERVO_VALVE_PERIOD, MICROSEC_FORMAT);

    // Servo blower setup
    servoBlower = PressureValve(hardwareTimer1, TIM_CHANNEL_SERVO_VALVE_BLOWER, PIN_SERVO_BLOWER,
                                VALVE_OPEN_STATE, VALVE_CLOSED_STATE);

    servoBlower.setup();
    hardwareTimer1->resume();

    // Servo patient setup
    servoPatient = PressureValve(hardwareTimer3, TIM_CHANNEL_SERVO_VALVE_PATIENT, PIN_SERVO_PATIENT,
                                 VALVE_OPEN_STATE, VALVE_CLOSED_STATE);
    servoPatient.setup();

    blower = Blower(hardwareTimer3, TIM_CHANNEL_ESC_BLOWER, PIN_ESC_BLOWER);
    blower.setup();
    blower.stop();

    // Prepare LEDs
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);

    initBattery();

    Buzzer_Init();

}



void loop() {

    analogButtons.check();
    btn_start.tick();

    switch (step) {

        case STEP_WELCOME: {
            UNGREEDY(is_drawn, {
                display("MakAir test", "Press start button");
                displayLine(VERSION, 3);
            });
            break;

        }
        case STEP_BLOWER_TEST: {
            UNGREEDY(is_drawn, display("Test blower", "Continuer : Start"));
            blower.runSpeed(150);
            break;
        }
        case STEP_VALVE_BLOWER_TEST: {
            UNGREEDY(is_drawn, display("Test Valve expi", "Continuer : Start"));
            blower.stop();

            if (millis()-last_time<5000){
                servoBlower.open(0);
                servoBlower.execute();
                displayLine("Etat : Ouvert", 3);
                
            }else if (millis()-last_time<10000){
                servoBlower.open(125);
                servoBlower.execute();
                displayLine("Etat : Ferme", 3);  
                
            }else{
                last_time = millis();
            }
            break;
        }
        case STEP_VALVE_PATIENT_TEST: {
            UNGREEDY(is_drawn, display("Test Valve inspi", "Continuer : Start"));
            if (millis()-last_time<5000){
                servoPatient.open(0);
                servoPatient.execute();
                displayLine("Etat : Ouvert", 3);
                
            }else if (millis()-last_time<10000){
                servoPatient.open(125);
                servoPatient.execute();
                displayLine("Etat : Ferme", 3);  
            }else{
                last_time = millis();
            }
            break;
        }
        case STEP_VALVE_BLOWER_LEAK_TEST: {
            UNGREEDY(is_drawn, display("Test fuite expi", "Continuer : Start"));
            servoPatient.open(0);
            servoPatient.execute();
            servoBlower.open(125);
            servoBlower.execute();
            blower.runSpeed(150);
            break;
        }
        case STEP_VALVE_PATIENT_LEAK_TEST: {
            UNGREEDY(is_drawn, display("Test fuite inspi", "Continuer : Start"));
            servoPatient.open(125);
            servoPatient.execute();
            servoBlower.open(0);
            servoBlower.execute();
            blower.runSpeed(150);
            break;
        }
    

    }

    IWatchdog.reload();

    delay(10);
}

#endif