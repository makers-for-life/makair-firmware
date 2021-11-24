/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure_valve.cpp
 * @brief Tools to control Pressure Valves
 *****************************************************************************/

// INCLUDES ===================================================================

// Associated header
#include "../includes/pressure_valve.h"

// External libraries
#include <algorithm>

// Internal libraries
#include "../includes/parameters.h"

// INITIALISATION =============================================================

PressureValve expiratoryValve;
PressureValve inspiratoryValve;

// FUNCTIONS ==================================================================

PressureValve::PressureValve() {}

PressureValve::PressureValve(HardwareTimer* p_hardwareTimer,
                             uint16_t p_timerChannel,
                             uint16_t p_valvePin,
                             uint16_t p_openApertureAngle,
                             uint16_t p_closeApertureAngle) {
    actuator = p_hardwareTimer;
    timerChannel = p_timerChannel;
    valvePin = p_valvePin;
    openApertureAngle = p_openApertureAngle;
    closeApertureAngle = p_closeApertureAngle;
    minApertureAngle = min(p_closeApertureAngle, p_openApertureAngle);
    maxApertureAngle = max(p_closeApertureAngle, p_openApertureAngle);
    command = p_closeApertureAngle;
    position = -1;
    positionLinear = 0;
}

void PressureValve::setup() {
    actuator->setMode(timerChannel, TIMER_OUTPUT_COMPARE_PWM1, valvePin);
    actuator->setCaptureCompare(timerChannel, 0, MICROSEC_COMPARE_FORMAT);
}

void PressureValve::open() { command = openApertureAngle; }

void PressureValve::close() { command = closeApertureAngle; }

void PressureValve::open(uint16_t p_command) { command = p_command; }

void PressureValve::openSection(int32_t p_sectionMultiplyBy100) {
    // Min-max to prevent overflow
    int32_t cappedSectionMultiplyBy100 =
        // cppcheck-suppress misra-c2012-12.3 ; cppcheck error
        min(max(int32_t(0), p_sectionMultiplyBy100), int32_t(3318));

    int32_t tempCommand;
    if (cappedSectionMultiplyBy100 < 1960) {
        tempCommand = 98 - (318 * cappedSectionMultiplyBy100 / 10000);
    } else {
        tempCommand =
            2626 - (36 * cappedSectionMultiplyBy100) / 10
            + 168 * ((cappedSectionMultiplyBy100 * cappedSectionMultiplyBy100) / 100) / 1000
            - 264
                  * (((cappedSectionMultiplyBy100 * cappedSectionMultiplyBy100) / 100)
                     * (cappedSectionMultiplyBy100) / 100)
                  / 100000;
    }
    // cppcheck-suppress misra-c2012-12.3
    command = min(max(int32_t(minApertureAngle), tempCommand), int32_t(maxApertureAngle));
}

// Linearization has been made experimentaly
uint16_t PressureValve::openLinear(uint16_t p_command) {
    uint16_t cappedCommand =
        // cppcheck-suppress misra-c2012-12.3 ; cppcheck error
        min(max(uint16_t(minApertureAngle), p_command), uint16_t(maxApertureAngle));

    positionLinear = cappedCommand;
    // The cappedCommand is in [ 0 ; 125 ], but the valve is only effective in [30; 100]
    // So lets make it between 30 and 100
    // A x10 multiplier is used here for better precision with integer calculations
    // The value of intermediateValue will consequently be in [300 ; 1000]
    uint32_t intermediateValue =
        (((static_cast<uint32_t>(cappedCommand) * 75u) / 125u) + 30u) * 10u;

    /* An order 3 polynom is used to correct the non linearity of the valve.
    To find this polynom, the following experimental protocol was used:
        - Run the blower max speed during the full experience
        - Plug inspiratory output on expiratory input
        - For each openning value of the valve, wait 5s for stabilisation and measure Flow

    This gives a csv with:
        openningValue[0;125], flow (mL/min)
        0, 48137
        10, 47780
        ...
        125, 127

    Then we calculated for each openning value the "linear target flow" with a basic linear
    equation:
        openningValue[0;125], flow (mL/min), targetFlow (mL/min)
        0, 48137,48137
        10, 47780, 44296
        ...
        125, 127, 0

    Then we manually found for each openning value, the openning value that should have been
    used to reach the targetFlow: openningValue[0;125], flow (mL/min), targetFlow (mL/min),
        correctedOpenningValue[0;125] 0, 48137,48137, 30 10, 47780, 44296, 55.2
        ...
        125, 127, 0, 100

    Then when made an order 3 regression between correctedOpenningValue and openningValue. This
    is the relation below:
    */
    command = (uint16_t)((((76u * intermediateValue) / 10u)
                          - (((985u * intermediateValue) * intermediateValue) / 100000u)
                          + (((((44u * intermediateValue) * intermediateValue) / 1000u)
                              * intermediateValue)
                             / 10000u)
                          - 1140u)
                         / 10u);

    // cppcheck-suppress misra-c2012-12.3 ; cppcheck error
    command = min(max(uint16_t(minApertureAngle), command), uint16_t(maxApertureAngle));

    return command;
}

// This is used to calculate a theorical expiratory flow
int32_t PressureValve::getSectionBigHoseX100() {
    int32_t section;
    if (command > 105u) {
        section = 0;
    } else if (command >= 50u) {
        section = 5760u - (558u * command / 10u);
    } else {
        section = 4390u - (5u * command) - (47u * command * command / 100u);
    }

    return max(int32_t(0), section);
}

uint16_t valveAngle2MicroSeconds(uint16_t value) {
    // Faulhaber motors works with PWM
    return map(value, 0, 125, FAULHABER_OPENED, FAULHABER_CLOSED);
}
