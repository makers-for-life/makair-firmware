/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2021 Makers For Life
 * @file eeprom.cpp
 * @brief I2C eeprom management
 *
 * The EEPROM share the I2C bus with massflowmeter:
 * Time to read or write could reach 200ms, up to structure size
 * The eeprom release the bus during 7 ms every 16 bytes read or written.
 * The massflowmeter will get some values in between.
 * So, the error on the flow measurement will be invisible, and the error on the volume measurement
 * will have a very very slight error.
 *
 * Reading:
 * The EEProm_Content is updated with EEPROM actual content.
 * When the EEPROM is virgin, the EEProm_Content structure is overriden
 * When the I2C fails, the EEProm_Content structure is overriden
 * When the EEPROM is corrupted, the EEProm_Content will contain some corrupted data.
 *
 * Writing:
 * The EEProm_Content is copied to EEPROM.
 *
 *****************************************************************************/

// INCLUDES ===================================================================

// Associated header
#include "../includes/eeprom.h"

#include <math.h>

// Internal
#include "../includes/config.h"
#include "../includes/mass_flow_meter.h"
#include "../includes/parameters.h"

// External
#include "CRC32.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <IWatchdog.h>
#include <OneButton.h>
#include <Wire.h>

#define EEPROM_I2C_ADDRESS 0x51

unsigned char EEPROM_Buffer[256];

// cppcheck-suppress misra-c2012-19.2 ; union correctly used
union {
    uint32_t crc;
    unsigned char c[4];
    // cppcheck-suppress misra-c2012-19.2 ; union correctly used
} eeprom_crc32;

#define EEPROM_VIRGIN_NOTINITIALIZED 0xFA
#define EEPROM_VIRGIN_TRUE 0xFF
#define EEPROM_VIRGIN_FALSE 0xAB
uint8_t eeprom_virgin = EEPROM_VIRGIN_NOTINITIALIZED;

inline void eeprom_wire_begin(void) {
    // pause massflowmeter interrupt before opening I2C bus.
    MFM_force_release_I2C = MFM_FORCE_RELEASE_I2C_TRUE;
    Wire.begin();
}

inline void eeprom_wire_end(void) {
    // resume mass flow meter after releasing I2C bus.
    Wire.end();
    MFM_force_release_I2C = MFM_FORCE_RELEASE_I2C_FALSE;
}

int32_t eeprom_read(void) {
    int32_t totalReadCount = 0;
    uint8_t readCount;

    // read last 5 bytes (status bytes), check for virgin status. early exit.
    eeprom_wire_begin();
    readCount = Wire.requestFrom(EEPROM_I2C_ADDRESS, 5, 0XFB, 1, true);
    eeprom_crc32.c[0] = Wire.read();
    eeprom_crc32.c[1] = Wire.read();
    eeprom_crc32.c[2] = Wire.read();
    eeprom_crc32.c[3] = Wire.read();
    eeprom_virgin = Wire.read();
    eeprom_wire_end();

    if (readCount != 5) {
        return EEPROM_ERROR_UNABLE_TO_RW;
    }

    if (EEPROM_VIRGIN_TRUE == eeprom_virgin) {
        return EEPROM_ERROR_READ_VIRGIN;
    }

    // read bytes 16 by 16.
    for (int page = 0; page < ((sizeof(EEProm_Content) / 16) + 1); page++) {
        eeprom_wire_begin();
        // requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t
        // sendStop)
        readCount = Wire.requestFrom(EEPROM_I2C_ADDRESS, 16, page * 16, 1, true);
        totalReadCount += readCount;
        for (int i = 0; i < 16; i++) {
            EEPROM_Buffer[page * 16 + i] = Wire.read();
        }
        eeprom_wire_end();
        delay(7);
    }

    int32_t returnVal = 0;
    if (totalReadCount >= sizeof(EEProm_Content)) {
        // read was successfull, copy from intermediate buffer to the real EEProm_Content
        memcpy(&EEProm_Content, &EEPROM_Buffer, sizeof(EEProm_Content));
    } else {
        returnVal = EEPROM_ERROR_UNABLE_TO_RW;
    }

    // compute CRC32, compare to the one in eeprom
    uint32_t crc = CRC32::calculate((unsigned char*)&EEProm_Content, sizeof(EEProm_Content));
    if (crc != eeprom_crc32.crc) {
        return EEPROM_ERROR_CORRUPTED;
    }

    return returnVal;
}

int32_t eeprom_write(void) {
    int32_t totalWriteErrors = 0;

    // write 16 bytes by 16.
    for (int page = 0; page < ((sizeof(EEProm_Content) / 16) + 1); page++) {
        eeprom_wire_begin();
        Wire.beginTransmission(EEPROM_I2C_ADDRESS);
        Wire.write(page * 16);  // address
        // don't ask me why "min" function is so clunky...
        int remainingBytesInPage =
            sizeof(EEProm_Content) - (page * 16) >= 16 ? 16 : sizeof(EEProm_Content) - (page * 16);
        Wire.write(&((reinterpret_cast<uint8_t*>(&EEProm_Content))[page * 16]),
                   remainingBytesInPage);
        totalWriteErrors += Wire.endTransmission();
        eeprom_wire_end();
        delay(7);
    }

    // also write CRC
    eeprom_crc32.crc = CRC32::calculate((unsigned char*)&EEProm_Content, sizeof(EEProm_Content));
    eeprom_wire_begin();
    Wire.beginTransmission(EEPROM_I2C_ADDRESS);
    Wire.write(0xFB);
    Wire.write(eeprom_crc32.c[0]);
    Wire.write(eeprom_crc32.c[1]);
    Wire.write(eeprom_crc32.c[2]);
    Wire.write(eeprom_crc32.c[3]);
    totalWriteErrors += Wire.endTransmission();
    eeprom_wire_end();
    delay(7);

    // also write virgin status, if needed
    if (EEPROM_VIRGIN_NOTINITIALIZED == eeprom_virgin || EEPROM_VIRGIN_TRUE == eeprom_virgin) {
        eeprom_wire_begin();
        Wire.beginTransmission(EEPROM_I2C_ADDRESS);
        Wire.write(0xFF);
        Wire.write(EEPROM_VIRGIN_FALSE);
        totalWriteErrors += Wire.endTransmission();
        delay(7);
        eeprom_wire_end();
    }

    return (totalWriteErrors == 0 ? 0 : EEPROM_ERROR_UNABLE_TO_RW);
}

#if MODE == MODE_EEPROM_TESTS
// use for tests only
// write 0xFF everywhere.
void eeprom_virginize(void) {
    // write 16 bytes by 16.
    for (int page = 0; page < 16; page++) {
        eeprom_wire_begin();
        Wire.beginTransmission(EEPROM_I2C_ADDRESS);
        Wire.write(page * 16);  // address
        for (int j = 0; j < 16; j++) {
            Wire.write(0xFF);
        }
        Wire.endTransmission();
        eeprom_wire_end();
        delay(7);
    }
}

void setup(void) {
    Serial.begin(115200);
    Serial.print("EEprom test program. size of struct =");
    Serial.println(sizeof(EEProm_Content));
    Serial.println("commands available :");
    Serial.println("r - Read eeprom");
    Serial.println("w - Write eeprom");
    Serial.println("d - Dump eeprom (I2C read)");
    Serial.println("v - Virginize eeprom");
    Serial.println("b - dump EEProm_Content buffer");

    MFM_init();
}

void loop(void) {
    char inChar;
    char buffer[100];
    if (Serial.available()) {
        inChar = static_cast<char>(Serial.read());
        // dump and print eeprom content
        if (inChar == 'd') {
            Serial.println("dump EEPROM");
            for (int page = 0; page < 16; page++) {
                eeprom_wire_begin();
                int readCount = Wire.requestFrom(EEPROM_I2C_ADDRESS, 16, page * 16, 1, true);
                (void)snprintf(buffer, sizeof(buffer), "@%02X | ", page * 16);
                Serial.print(buffer);
                for (int i = 0; i < 16; i++) {
                    (void)snprintf(buffer, sizeof(buffer), "%02X ", Wire.read());
                    Serial.print(buffer);
                }
                Serial.println("");
                eeprom_wire_end();
            }
            Serial.println("====");
        }
        // dump and print EEPROM buffer
        if (inChar == 'b') {
            Serial.println("dump EEProm_Content");
            for (int page = 0; page < 16; page++) {
                (void)snprintf(buffer, sizeof(buffer), "@%02X | ", page * 16);
                Serial.print(buffer);
                for (int i = 0; i < 16; i++) {
                    (void)snprintf(buffer, sizeof(buffer), "%02X ",
                                   ((unsigned char*)(&EEProm_Content))[page * 16 + i]);
                    Serial.print(buffer);
                }
                Serial.println("");
            }
            Serial.println("====");
        }
        // virginize eeprom
        if (inChar == 'v') {
            Serial.println("VIRGINIZING EEPROM ...");
            eeprom_virginize();
        }
        // write eeprom
        if (inChar == 'w') {
            Serial.print("writing eeprom... ");
            int r = eeprom_write();
            if (EEPROM_ERROR_UNABLE_TO_RW == r) {
                Serial.println("ERROR, unable to write in EEPROM");
            } else {
                Serial.println("EEPROM written");
            }
        }
        // read eeprom
        if (inChar == 'r') {
            Serial.print("reading eeprom... ");
            int r = eeprom_read();
            if (EEPROM_ERROR_CORRUPTED == r) {
                Serial.println("ERROR, eeprom corrupted");
            } else if (EEPROM_ERROR_READ_VIRGIN == r) {
                Serial.println("ERROR, eeprom vierge");
            } else if (EEPROM_ERROR_UNABLE_TO_RW == r) {
                Serial.println("ERROR, unable to read in EEPROM");
            } else if (0 == r) {
                Serial.println("OK");
            }
        }
        // update some bytes
        if (inChar == 'u') {
            Serial.println("update bytes in the EEProm_Content ");
            // increment the first byte.
            (*((unsigned char*)&EEProm_Content))++;
        }
    }
}
#endif
