/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2021 Makers For Life
 * @file mass_flow_meter.h
 * @brief Mass flow meter management
 *****************************************************************************/

#pragma once

#include <Arduino.h>

// declare here all types of data stored in eeprom
// Beware : this is only a 2Kbit EEPROM. 255 bytes maximum ! Last 5 bytes are reserved to detect
// virgin status and corruption error.
// Beware : do not write too often. 1 million write cycle is the limit during product lifetime.
// Beware : read about C struct alignement and padding on a 32 bits
// system, and check sizeof(struct). Reserved bytes :
// - 0xFF : 0XFF if virgin, 0xAB if not.
// - 0xFB, 0xFC, OxFD, 0xFE : CRC32 of EEPROM_Content
typedef struct {
    int16_t language;
    int32_t makair_minuteCounter;
    int32_t MFM_expi_minuteCounter;
    int32_t MFM_inspi_minuteCounter;
    char stuffc[62];
    char stuffc2[62];
    char stuffc3[62];

} EEProm_Content_t;

// define default values if needed
EEProm_Content_t EEProm_Content = {
    .language = -1,
    .makair_minuteCounter = -1,
    .stuffc = "123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ",
    .stuffc2 = "123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ",
    .stuffc3 = "123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"};

// this works like an assertion. (that does not allocate memory, it is just a typedef)
// it will fail to compile if EEProm_Content size is greater than 250
typedef char assertMaxEepromSize__LINE__[(sizeof(EEProm_Content) < 251) ? 1 : -1];

// error code: read or write function did not succeed
#define EEPROM_ERROR_UNABLE_TO_RW 1

// error code: the eeprom is virgin. EEProm_Content was not overriden.
#define EEPROM_ERROR_READ_VIRGIN 42

// error code: the eeprom is corrupted. EEProm_Content contains what was read.
#define EEPROM_ERROR_CORRUPTED 0xAA
