/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file telemetry.cpp
 * @brief Implementation of the telemetry protocol
 *****************************************************************************/

#pragma once

#include "../includes/config.h"

// INCLUDES ===================================================================

// Associated header
#include "../includes/telemetry.h"

// Externals
#if HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
#include "Arduino.h"
#include "CRC32.h"
#include "LL/stm32yyxx_ll_utils.h"
#endif

/// Internals
#include "../includes/pressure_controller.h"

// GLOBAL ITEMS ==============================================================

#if HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
/// The device ID to be joined with telemetry messages
static byte deviceId[12];  // 3 * 32 bits = 96 bits
#endif

#define FIRST_BYTE (uint8_t)0xFF

#define HEADER_SIZE 2
static const uint8_t header[HEADER_SIZE] = {0x03, 0x0C};
#define FOOTER_SIZE 2
static const uint8_t footer[FOOTER_SIZE] = {0x30, 0xC0};

// FUNCTIONS ==================================================================

/**
 * Convert a u16 so that it can be sent through serial
 *
 * @param bytes Empty array of 2 elements
 * @param data Input number
 * @return Array of 2 bytes
 */
// cppcheck-suppress unusedFunction
void toBytes16(byte bytes[], uint16_t data) {
    bytes[0] = (data >> 8) & FIRST_BYTE;
    bytes[1] = data & FIRST_BYTE;
}

/**
 * Convert a u32 so that it can be sent through serial
 *
 * @param bytes Empty array of 4 elements
 * @param data Input number
 */
// cppcheck-suppress unusedFunction
void toBytes32(byte bytes[], uint32_t data) {
    bytes[0] = (data >> 24) & FIRST_BYTE;
    bytes[1] = (data >> 16) & FIRST_BYTE;
    bytes[2] = (data >> 8) & FIRST_BYTE;
    bytes[3] = data & FIRST_BYTE;
}

/**
 * Convert a u64 so that it can be sent through serial
 *
 * @param bytes Empty array of 8 elements
 * @param data Input number
 */
// cppcheck-suppress unusedFunction
void toBytes64(byte bytes[], uint64_t data) {
    bytes[0] = (data >> 56) & FIRST_BYTE;
    bytes[1] = (data >> 48) & FIRST_BYTE;
    bytes[2] = (data >> 40) & FIRST_BYTE;
    bytes[3] = (data >> 32) & FIRST_BYTE;
    bytes[4] = (data >> 24) & FIRST_BYTE;
    bytes[5] = (data >> 16) & FIRST_BYTE;
    bytes[6] = (data >> 8) & FIRST_BYTE;
    bytes[7] = data & FIRST_BYTE;
}

#if HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
/**
 * Compute device ID
 *
 * @warning This requires (and mutates) a static deviceId variable (which must be an array of 12
 * elements)
 */
// cppcheck-suppress unusedFunction
void computeDeviceId(void) {
    deviceId[0] = (LL_GetUID_Word0() >> 24) & FIRST_BYTE;
    deviceId[1] = (LL_GetUID_Word0() >> 16) & FIRST_BYTE;
    deviceId[2] = (LL_GetUID_Word0() >> 8) & FIRST_BYTE;
    deviceId[3] = LL_GetUID_Word0() & FIRST_BYTE;
    deviceId[4] = (LL_GetUID_Word1() >> 24) & FIRST_BYTE;
    deviceId[5] = (LL_GetUID_Word1() >> 16) & FIRST_BYTE;
    deviceId[6] = (LL_GetUID_Word1() >> 8) & FIRST_BYTE;
    deviceId[7] = LL_GetUID_Word1() & FIRST_BYTE;
    deviceId[8] = (LL_GetUID_Word2() >> 24) & FIRST_BYTE;
    deviceId[9] = (LL_GetUID_Word2() >> 16) & FIRST_BYTE;
    deviceId[10] = (LL_GetUID_Word2() >> 8) & FIRST_BYTE;
    deviceId[11] = LL_GetUID_Word2() & FIRST_BYTE;
}

/**
 * Compute current systick
 *
 * @return Systick in microseconds
 */
// cppcheck-suppress unusedFunction
uint64_t computeSystick(void) {
    return (static_cast<uint64_t>(millis()) * 1000u) + (micros() % 1000u);
}
#endif

// cppcheck-suppress unusedFunction
void initTelemetry(void) {
#if HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
    Serial6.begin(115200);
    computeDeviceId();
#endif
}

// cppcheck-suppress unusedFunction
void sendBootMessage() {
#if HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
    uint8_t value128 = 128u;

    Serial6.write(header, HEADER_SIZE);
    CRC32 crc32;
    Serial6.write("B:");
    crc32.update("B:", 2);
    Serial6.write((uint8_t)1u);
    crc32.update((uint8_t)1u);

    Serial6.write(static_cast<uint8_t>(strlen(VERSION)));
    crc32.update(static_cast<uint8_t>(strlen(VERSION)));
    Serial6.print(VERSION);
    crc32.update(VERSION, strlen(VERSION));
    Serial6.write(deviceId, 12);
    crc32.update(deviceId, 12);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte systick[8];  // 64 bits
    toBytes64(systick, computeSystick());
    Serial6.write(systick, 8);
    crc32.update(systick, 8);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(MODE);
    crc32.update(static_cast<uint8_t>(MODE));

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(value128);
    crc32.update(value128);

    Serial6.print("\n");
    crc32.update("\n", 1);

    byte crc[4];  // 32 bits
    toBytes32(crc, crc32.finalize());
    Serial6.write(crc, 4);
    Serial6.write(footer, FOOTER_SIZE);
#endif
}

// cppcheck-suppress unusedFunction
void sendStoppedMessage() {
#if HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
    Serial6.write(header, HEADER_SIZE);
    CRC32 crc32;
    Serial6.write("O:");
    crc32.update("O:", 2);
    Serial6.write((uint8_t)1u);
    crc32.update((uint8_t)1u);

    Serial6.write(static_cast<uint8_t>(strlen(VERSION)));
    crc32.update(static_cast<uint8_t>(strlen(VERSION)));
    Serial6.print(VERSION);
    crc32.update(VERSION, strlen(VERSION));
    Serial6.write(deviceId, 12);
    crc32.update(deviceId, 12);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte systick[8];  // 64 bits
    toBytes64(systick, computeSystick());
    Serial6.write(systick, 8);
    crc32.update(systick, 8);

    Serial6.print("\n");
    crc32.update("\n", 1);

    byte crc[4];  // 32 bits
    toBytes32(crc, crc32.finalize());
    Serial6.write(crc, 4);
    Serial6.write(footer, FOOTER_SIZE);
#endif
}

// cppcheck-suppress unusedFunction
void sendDataSnapshot(uint16_t centileValue,
                      uint16_t pressureValue,
                      CyclePhases phase,
                      CycleSubPhases subPhase,
                      uint8_t blowerValvePosition,
                      uint8_t patientValvePosition,
                      uint8_t blowerRpm,
                      uint8_t batteryLevel) {
#if HARDWARE_VERSION == 1
    (void)centileValue;
    (void)pressureValue;
    (void)phase;
    (void)subPhase;
    (void)blowerValvePosition;
    (void)patientValvePosition;
    (void)blowerRpm;
    (void)batteryLevel;
#elif HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
    uint8_t phaseValue;
    if ((phase == CyclePhases::INHALATION) && (subPhase == CycleSubPhases::INSPIRATION)) {
        phaseValue = 17u;  // 00010001
    } else if ((phase == CyclePhases::INHALATION)
               && (subPhase == CycleSubPhases::HOLD_INSPIRATION)) {
        phaseValue = 18u;  // 00010010
    } else if ((phase == CyclePhases::EXHALATION) && (subPhase == CycleSubPhases::EXHALE)) {
        phaseValue = 68u;  // 01000100
    } else {
        phaseValue = 0u;
    }

    Serial6.write(header, HEADER_SIZE);
    CRC32 crc32;
    Serial6.write("D:");
    crc32.update("D:", 2);
    Serial6.write((uint8_t)1u);
    crc32.update((uint8_t)1u);

    Serial6.write(static_cast<uint8_t>(strlen(VERSION)));
    crc32.update(static_cast<uint8_t>(strlen(VERSION)));
    Serial6.print(VERSION);
    crc32.update(VERSION, strlen(VERSION));
    Serial6.write(deviceId, 12);
    crc32.update(deviceId, 12);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte systick[8];  // 64 bits
    toBytes64(systick, computeSystick());
    Serial6.write(systick, 8);
    crc32.update(systick, 8);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte centile[2];  // 16 bits
    toBytes16(centile, centileValue);
    Serial6.write(centile, 2);
    crc32.update(centile, 2);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte pressure[2];  // 16 bits
    toBytes16(pressure, pressureValue);
    Serial6.write(pressure, 2);
    crc32.update(pressure, 2);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(phaseValue);
    crc32.update(phaseValue);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(blowerValvePosition);
    crc32.update(blowerValvePosition);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(patientValvePosition);
    crc32.update(patientValvePosition);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(blowerRpm);
    crc32.update(blowerRpm);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(batteryLevel);
    crc32.update(batteryLevel);

    Serial6.print("\n");
    crc32.update("\n", 1);

    byte crc[4];  // 32 bits
    toBytes32(crc, crc32.finalize());
    Serial6.write(crc, 4);
    Serial6.write(footer, FOOTER_SIZE);
#endif
}

// cppcheck-suppress unusedFunction
void sendMachineStateSnapshot(uint32_t cycleValue,
                              uint8_t peakCommand,
                              uint8_t plateauCommand,
                              uint8_t peepCommand,
                              uint8_t cpmCommand,
                              uint16_t previousPeakPressureValue,
                              uint16_t previousPlateauPressureValue,
                              uint16_t previousPeepPressureValue,
                              uint8_t currentAlarmCodes[ALARMS_SIZE],
                              uint16_t volumeValue,
                              uint8_t expiratoryTerm,
                              bool triggerEnabled,
                              uint8_t triggerOffset) {
#if HARDWARE_VERSION == 1
    (void)cycleValue;
    (void)peakCommand;
    (void)plateauCommand;
    (void)peepCommand;
    (void)cpmCommand;
    (void)previousPeakPressureValue;
    (void)previousPlateauPressureValue;
    (void)previousPeepPressureValue;
    (void)currentAlarmCodes;
    (void)volumeValue;
    (void)expiratoryTerm;
    (void)triggerEnabled;
    (void)triggerOffset;
#elif HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
    uint8_t currentAlarmSize = 0;
    for (uint8_t i = 0; i < ALARMS_SIZE; i++) {
        if (currentAlarmCodes[i] != 0u) {
            currentAlarmSize++;
        } else {
            break;
        }
    }

    Serial6.write(header, HEADER_SIZE);
    CRC32 crc32;
    Serial6.write("S:");
    crc32.update("S:", 2);
    Serial6.write((uint8_t)1u);
    crc32.update((uint8_t)1u);

    Serial6.write(static_cast<uint8_t>(strlen(VERSION)));
    crc32.update(static_cast<uint8_t>(strlen(VERSION)));
    Serial6.print(VERSION);
    crc32.update(VERSION, strlen(VERSION));
    Serial6.write(deviceId, 12);
    crc32.update(deviceId, 12);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte systick[8];  // 64 bits
    toBytes64(systick, computeSystick());
    Serial6.write(systick, 8);
    crc32.update(systick, 8);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte cycle[4];  // 32 bits
    toBytes32(cycle, cycleValue);
    Serial6.write(cycle, 4);
    crc32.update(cycle, 4);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(peakCommand);
    crc32.update(peakCommand);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(plateauCommand);
    crc32.update(plateauCommand);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(peepCommand);
    crc32.update(peepCommand);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(cpmCommand);
    crc32.update(cpmCommand);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte previousPeakPressure[2];  // 16 bits
    toBytes16(previousPeakPressure, previousPeakPressureValue);
    Serial6.write(previousPeakPressure, 2);
    crc32.update(previousPeakPressure, 2);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte previousPlateauPressure[2];  // 16 bits
    toBytes16(previousPlateauPressure, previousPlateauPressureValue);
    Serial6.write(previousPlateauPressure, 2);
    crc32.update(previousPlateauPressure, 2);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte previousPeepPressure[2];  // 16 bits
    toBytes16(previousPeepPressure, previousPeepPressureValue);
    Serial6.write(previousPeepPressure, 2);
    crc32.update(previousPeepPressure, 2);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(currentAlarmSize);
    crc32.update(currentAlarmSize);
    Serial6.write(currentAlarmCodes, currentAlarmSize);
    crc32.update(currentAlarmCodes, currentAlarmSize);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte volume[2];  // 16 bits
    toBytes16(volume, volumeValue);
    Serial6.write(volume, 2);
    crc32.update(volume, 2);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(expiratoryTerm);
    crc32.update(expiratoryTerm);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(triggerEnabled);
    crc32.update(triggerEnabled);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(triggerOffset);
    crc32.update(triggerOffset);

    Serial6.print("\n");
    crc32.update("\n", 1);

    byte crc[4];  // 32 bits
    toBytes32(crc, crc32.finalize());
    Serial6.write(crc, 4);
    Serial6.write(footer, FOOTER_SIZE);
#endif
}

// cppcheck-suppress unusedFunction
void sendAlarmTrap(uint16_t centileValue,
                   uint16_t pressureValue,
                   CyclePhases phase,
                   CycleSubPhases subPhase,
                   uint32_t cycleValue,
                   uint8_t alarmCode,
                   AlarmPriority alarmPriority,
                   bool triggered,
                   uint32_t expectedValue,
                   uint32_t measuredValue,
                   uint32_t cyclesSinceTriggerValue) {
#if HARDWARE_VERSION == 1
    (void)centileValue;
    (void)pressureValue;
    (void)phase;
    (void)subPhase;
    (void)cycleValue;
    (void)alarmCode;
    (void)alarmPriority;
    (void)triggered;
    (void)expectedValue;
    (void)measuredValue;
    (void)cyclesSinceTriggerValue;
#elif HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
    uint8_t phaseValue;
    if ((phase == CyclePhases::INHALATION) && (subPhase == CycleSubPhases::INSPIRATION)) {
        phaseValue = 17u;  // 00010001
    } else if ((phase == CyclePhases::INHALATION)
               && (subPhase == CycleSubPhases::HOLD_INSPIRATION)) {
        phaseValue = 18u;  // 00010010
    } else if ((phase == CyclePhases::EXHALATION) && (subPhase == CycleSubPhases::EXHALE)) {
        phaseValue = 68u;  // 01000100
    } else {
        phaseValue = 0u;
    }

    uint8_t triggeredValue;
    if (triggered) {
        triggeredValue = 240u;  // 11110000
    } else {
        triggeredValue = 15u;  // 00001111
    }

    uint8_t alarmPriorityValue;
    if (alarmPriority == AlarmPriority::ALARM_HIGH) {
        alarmPriorityValue = 4u;  // 00000100
    } else if (alarmPriority == AlarmPriority::ALARM_MEDIUM) {
        alarmPriorityValue = 2u;  // 00000010
    } else if (alarmPriority == AlarmPriority::ALARM_LOW) {
        alarmPriorityValue = 1u;  // 00000001
    } else {
        alarmPriorityValue = 0u;  // 00000000
    }

    Serial6.write(header, HEADER_SIZE);
    CRC32 crc32;
    Serial6.write("T:");
    crc32.update("T:", 2);
    Serial6.write((uint8_t)1u);
    crc32.update((uint8_t)1u);

    Serial6.write(static_cast<uint8_t>(strlen(VERSION)));
    crc32.update(static_cast<uint8_t>(strlen(VERSION)));
    Serial6.print(VERSION);
    crc32.update(VERSION, strlen(VERSION));
    Serial6.write(deviceId, 12);
    crc32.update(deviceId, 12);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte systick[8];  // 64 bits
    toBytes64(systick, computeSystick());
    Serial6.write(systick, 8);
    crc32.update(systick, 8);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte centile[2];  // 16 bits
    toBytes16(centile, centileValue);
    Serial6.write(centile, 2);
    crc32.update(centile, 2);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte pressure[2];  // 16 bits
    toBytes16(pressure, pressureValue);
    Serial6.write(pressure, 2);
    crc32.update(pressure, 2);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(phaseValue);
    crc32.update(phaseValue);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte cycle[4];  // 32 bits
    toBytes32(cycle, cycleValue);
    Serial6.write(cycle, 4);
    crc32.update(cycle, 4);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(alarmCode);
    crc32.update(alarmCode);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(alarmPriorityValue);
    crc32.update(alarmPriorityValue);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(triggeredValue);
    crc32.update(triggeredValue);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte expected[4];  // 32 bits
    toBytes32(expected, expectedValue);
    Serial6.write(expected, 4);
    crc32.update(expected, 4);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte measured[4];  // 32 bits
    toBytes32(measured, measuredValue);
    Serial6.write(measured, 4);
    crc32.update(measured, 4);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte cyclesSinceTrigger[4];  // 32 bits
    toBytes32(cyclesSinceTrigger, cyclesSinceTriggerValue);
    Serial6.write(cyclesSinceTrigger, 4);
    crc32.update(cyclesSinceTrigger, 4);

    Serial6.print("\n");
    crc32.update("\n", 1);

    byte crc[4];  // 32 bits
    toBytes32(crc, crc32.finalize());
    Serial6.write(crc, 4);
    Serial6.write(footer, FOOTER_SIZE);
#endif
}

// cppcheck-suppress unusedFunction
void sendControlAck(uint8_t setting, uint16_t valueValue) {
#if HARDWARE_VERSION == 1
    (void)setting;
    (void)valueValue;
#elif HARDWARE_VERSION == 2 || HARDWARE_VERSION == 3
    Serial6.write(header, HEADER_SIZE);
    CRC32 crc32;
    Serial6.write("A:");
    crc32.update("A:", 2);
    Serial6.write((uint8_t)1u);
    crc32.update((uint8_t)1u);

    Serial6.write(static_cast<uint8_t>(strlen(VERSION)));
    crc32.update(static_cast<uint8_t>(strlen(VERSION)));
    Serial6.print(VERSION);
    crc32.update(VERSION, strlen(VERSION));
    Serial6.write(deviceId, 12);
    crc32.update(deviceId, 12);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte systick[8];  // 64 bits
    toBytes64(systick, computeSystick());
    Serial6.write(systick, 8);
    crc32.update(systick, 8);

    Serial6.print("\t");
    crc32.update("\t", 1);

    Serial6.write(setting);
    crc32.update(setting);

    Serial6.print("\t");
    crc32.update("\t", 1);

    byte value[2];  // 16 bits
    toBytes16(value, valueValue);
    Serial6.write(value, 2);
    crc32.update(value, 2);

    Serial6.print("\n");
    crc32.update("\n", 1);

    byte crc[4];  // 32 bits
    toBytes32(crc, crc32.finalize());
    Serial6.write(crc, 4);
    Serial6.write(footer, FOOTER_SIZE);
#endif
}

// cppcheck-suppress unusedFunction
uint8_t mmH2OtoCmH2O(uint16_t pressure) {
    uint8_t result;
    uint16_t lastDigit = pressure % 10u;

    if (lastDigit < 5u) {
        result = (pressure / 10u);
    } else {
        result = (pressure / 10u) + 1u;
    }

    return result;
}
