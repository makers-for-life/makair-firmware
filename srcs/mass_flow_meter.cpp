/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file mass_flow_meter.cpp
 * @brief Mass Flow meter management
 *
 * SFM3300-D sensirion mass flow meter is connected on I2C bus.
 * To perform the integral of the mass flow, I2C polling must be done in a high priority timer.
 *****************************************************************************/

// INCLUDES ===================================================================

// Associated header
#include "../includes/mass_flow_meter.h"

// External
#include <Arduino.h>
#include <HardwareSerial.h>
#include <IWatchdog.h>
#include <OneButton.h>
#include <Wire.h>
#include <math.h>

// Internal
#include "../includes/buzzer_control.h"
#include "../includes/config.h"
#include "../includes/parameters.h"
#include "../includes/screen.h"

// INITIALISATION =============================================================

// Hardware is ensured to be at least v2
#ifdef MASS_FLOW_METER_ENABLED

// 2 kHz => prescaler = 50000 => still OK for a 16 bit timer. it cannnot be slower
// 10 kHz => nice
#define MASS_FLOW_TIMER_FREQ 10000

// The timer period in 100 us multiple (because 10 kHz prescale)
#define MASS_FLOW_PERIOD 100

uint32_t mfmHoneywellHafSerialNumber = 0;

uint32_t mfmSfm3300SerialNumberExpi = 0;

HardwareTimer* massFlowTimer;

volatile bool mfmFaultCondition = false;

// Calibration offset is substracted to MFM instant flow:
// - before volume integral
// - before sending MFM_read_airflow result
volatile int32_t mfmInspiratoryCalibrationOffset = 0;
volatile int32_t mfmInspiratoryAirVolumeSumMilliliters = 0;
volatile int32_t mfmInspiratorySensorDetected = 0;
volatile int32_t mfmInspiratoryInstantAirFlow = 0;

volatile int32_t mfmExpiratoryCalibrationOffset = 0;
volatile int32_t mfmExpiratoryAirVolumeSumMilliliters = 0;
volatile int32_t mfmExpiratorySensorDetected = 0;
volatile int32_t mfmExpiratoryInstantAirFlow = 0;

// Size of the table used to compute average
#define MFM_MEAN_SAMPLES 40
volatile bool mfmInspiratoryInstantAirFlowRecord = false;
volatile int32_t mfmInspiratoryInstantAirFlowLastValues[MFM_MEAN_SAMPLES];
// cppcheck-suppress misra-c2012-5.1
volatile int16_t mfmInspiratoryInstantAirFlowLastValuesIndex = 0;

int32_t mfmInspiratoryLastValue = 0;
volatile int32_t mfmInspiratoryLastValueFixedFloat = 0;

volatile int32_t mfmExpiratoryLastValueFixedFloat = 0;

// Time to reset the sensor after I2C restart, in periods => 100 ms
// the restart time is 50 ms (warm up time in the datasheet)
// the power off time is 50 ms. enough to discharge capacitors
#define MFM_WAIT_RESET_PERIODS 13
#define MFM_WAIT_WARMUP_PERIODS 8
#define MFM_WAIT_SOFTRESET_PERIODS 3
#define MFM_WAIT_READSERIALR1_PERIODS 1

int32_t mfmResetStateMachine = MFM_WAIT_RESET_PERIODS;

uint16_t mfmExpiSFM3300FailCounter = 0;

// cppcheck-suppress misra-c2012-19.2 ; union correctly used
union {
    uint16_t i;
    int16_t si;
    unsigned char c[2];
    // cppcheck-suppress misra-c2012-19.2 ; union correctly used
} mfmLastData;

// FUNCTIONS ==================================================================

// API update since version 1.9.0 of Arduino_Core_STM32
#if (STM32_CORE_VERSION < 0x01090000)
// cppcheck-suppress misra-c2012-2.7 ; valid unused parameter
void MFM_Timer_Callback(HardwareTimer*)  // NOLINT(readability/casting)
#else
void MFM_Timer_Callback(void)
#endif
{
#if MODE == MODE_MFM_TESTS
    // cppcheck-suppress misra-c2012-12.3
    digitalWrite(PIN_LED_START, HIGH);
    // it takes typically 350 µs to read the value.
#endif
    if (!mfmFaultCondition) {
#if MASS_FLOW_METER_SENSOR == MFM_SFM3019
        Wire.begin();
        uint8_t readCountbis = Wire.requestFrom(0x2E, 3);
        mfmLastData.c[1] = Wire.read();
        mfmLastData.c[0] = Wire.read();
        Wire.end();
        int32_t expi = (1000 * (mfmLastData.si + 24576)) / 200;
#endif
#if MASS_FLOW_METER_SENSOR == MFM_HONEYWELL_HAF

        // begin() and end() everytime you read... the lib never free buffers if you don't do this.
        Wire.begin();
        uint8_t readCount = Wire.requestFrom(MFM_SENSOR_I2C_ADDRESS, 2);
        mfmLastData.c[0] = Wire.read();
        mfmLastData.c[1] = Wire.read();
        // Wire.endTransmission() send a new write order followed by a stop. Useless and the sensor
        // often nack it.
        Wire.end();

        // Hardware reset if not able to read two bytes.
        if (readCount != 2u) {
            mfmFaultCondition = true;
            mfmResetStateMachine = MFM_WAIT_RESET_PERIODS;
            mfmInspiratoryAirVolumeSumMilliliters = 1000000000;  // 1e9
        }

        mfmInspiratoryLastValue = (uint32_t)(mfmLastData.c[1] & 0xFFu);
        mfmInspiratoryLastValue |= (((uint32_t)mfmLastData.c[0]) << 8) & 0x0000FF00u;

        // Theorical formula: Flow(slpm) = 200*((rawvalue/16384)-0.1)/0.8
        // float implementation, 1 liter per minute unit
        // mfmLastValueFloat =
        //    MFM_RANGE * (((uint32_t)mfmInspiratoryLastValue / 16384.0) - 0.1) / 0.8;
        // (Output value in SLPM)

        // fixed float implementation, 1 milliliter per minute unit
        mfmInspiratoryLastValueFixedFloat =
            (((10 * mfmInspiratoryLastValue) - 16384) * 1526) / 1000;

        // 100 value per second, 100 slpm during 10 minutes: sum will be 1.2e9. it fits in a int32
        // int32 max with milliliters = 2e6 liters.

        // The sensor (100 SPLM version anyway) tends to output spurrious values located at around
        // 500 SLM, which are obviously not correct. Let's filter them out based on the range of the
        // sensor + 10%.
        if (mfmInspiratoryLastValueFixedFloat < (MFM_RANGE * 1100)) {
            mfmInspiratoryInstantAirFlow = mfmInspiratoryLastValueFixedFloat;
            if (mfmInspiratoryLastValueFixedFloat > 500) {  // less than 0.5 SPLM is noise
                mfmInspiratoryAirVolumeSumMilliliters +=
                    (mfmInspiratoryLastValueFixedFloat - mfmInspiratoryCalibrationOffset);
            }
            // also fill the last values table used to compute average, and update the index
            if (mfmInspiratoryInstantAirFlowRecord) {
                mfmInspiratoryInstantAirFlowLastValues
                    [mfmInspiratoryInstantAirFlowLastValuesIndex] =
                        mfmInspiratoryLastValueFixedFloat;
                mfmInspiratoryInstantAirFlowLastValuesIndex++;
                if (MFM_MEAN_SAMPLES == mfmInspiratoryInstantAirFlowLastValuesIndex) {
                    mfmInspiratoryInstantAirFlowLastValuesIndex = 0;
                    // table full, stops
                    mfmInspiratoryInstantAirFlowRecord = false;
                }
            }
        }
#endif

#if MASS_FLOW_METER_SENSOR_EXPI == MFM_SFM_3300D
        // begin() and end() everytime you read... the lib never free buffers if you don't do this.
        Wire.begin();
        // do not request crc, only two bytes
        uint8_t readCountExpi = Wire.requestFrom(MFM_SENSOR_EXPI_I2C_ADDRESS, 2);
        mfmLastData.c[1] = Wire.read();
        mfmLastData.c[0] = Wire.read();
        Wire.end();

        // conversion in milliter per minute flow: ((int32_t)(mfmLastData.i) - 32768) * 1000 / 120
        // but 1000/120 = 8.333. So  *8 and *1/3
        mfmExpiratoryLastValueFixedFloat =
            (((int32_t)(mfmLastData.i) - 32768) * 8) + (((int32_t)(mfmLastData.i) - 32768) / 3);

        if (readCountExpi != 2u) {
            mfmExpiSFM3300FailCounter++;
            // sfm 3300d needs 100ms after start of measurement before sending data.
            // in case of bus failure, mfmFaultCondition is already true at this point
            if ((mfmExpiSFM3300FailCounter > 12u) || mfmFaultCondition) {
                mfmFaultCondition = true;
                mfmResetStateMachine = MFM_WAIT_RESET_PERIODS;
                mfmExpiratoryAirVolumeSumMilliliters = 1000000000;  // 1e9
            }
        } else {
            // valid data
            mfmExpiratoryInstantAirFlow = mfmExpiratoryLastValueFixedFloat;
            if ((mfmExpiratoryLastValueFixedFloat > 500)
                || (mfmExpiratoryLastValueFixedFloat < -500)) {  // less than 0.5 SPLM is noise
                mfmExpiratoryAirVolumeSumMilliliters +=
                    (mfmExpiratoryLastValueFixedFloat - mfmExpiratoryCalibrationOffset);
            }
        }

#endif
    } else {
        if (mfmResetStateMachine == MFM_WAIT_RESET_PERIODS) {
            // Reset attempt
            // I2C sensors
            Wire.flush();
            Wire.end();

            // Set power off (available since hw3)
            digitalWrite(MFM_POWER_CONTROL, MFM_POWER_OFF);
            // also set SDA and SCL to 0 to avoid sensor to be powered by I2C bus.
            pinMode(PIN_I2C_SDA, OUTPUT);
            pinMode(PIN_I2C_SCL, OUTPUT);
            digitalWrite(PIN_I2C_SDA, LOW);
            digitalWrite(PIN_I2C_SCL, LOW);
        }

        mfmResetStateMachine--;

        // x period before end of reset cycle, power on again
        if (mfmResetStateMachine == MFM_WAIT_WARMUP_PERIODS) {
            // Set power on (available since hw v3)
            digitalWrite(MFM_POWER_CONTROL, MFM_POWER_ON);
            pinMode(PIN_I2C_SDA, INPUT);
            pinMode(PIN_I2C_SCL, INPUT);
        }

#if MASS_FLOW_METER_SENSOR == MFM_HONEYWELL_HAF
        if (mfmResetStateMachine == MFM_WAIT_SOFTRESET_PERIODS) {
            Wire.begin();
            Wire.beginTransmission(MFM_SENSOR_I2C_ADDRESS);
            Wire.write(0x02);                         // Force reset
            uint8_t status = Wire.endTransmission();  // actually send the data
            Wire.end();
            if (status != 0u) {  // still a problem
                mfmResetStateMachine = MFM_WAIT_RESET_PERIODS;
            }
        }
        if (mfmResetStateMachine == MFM_WAIT_READSERIALR1_PERIODS) {
            Wire.begin();
            // read first serial number register
            uint8_t rxcount = Wire.requestFrom(MFM_SENSOR_I2C_ADDRESS, 2);
            Wire.end();
            if (rxcount != 2u) {  // still a problem
                mfmResetStateMachine = MFM_WAIT_RESET_PERIODS;
            }
        }
#endif

        if (mfmResetStateMachine == 0) {
            mfmFaultCondition = false;
            // MFM_WAIT_RESET_PERIODS cycles later, try again to init the sensor

#if MASS_FLOW_METER_SENSOR == MFM_HONEYWELL_HAF
            Wire.begin();
            // read second serial number register
            uint8_t rxcount = Wire.requestFrom(MFM_SENSOR_I2C_ADDRESS, 2);
            Wire.end();
            mfmFaultCondition = (rxcount != 2u) || mfmFaultCondition;
#endif

#if MASS_FLOW_METER_SENSOR_EXPI == MFM_SFM_3300D
            Wire.begin();
            Wire.beginTransmission(MFM_SENSOR_EXPI_I2C_ADDRESS);
            Wire.write(0x10);
            Wire.write(0x00);
            mfmFaultCondition = (0 != Wire.endTransmission()) || mfmFaultCondition;
            mfmExpiSFM3300FailCounter = 0;
            Wire.end();
#endif

            if (mfmFaultCondition) {
                mfmResetStateMachine = MFM_WAIT_RESET_PERIODS;
            }
        }
    }

#if MODE == MODE_MFM_TESTS
    digitalWrite(PIN_LED_START, LOW);
    digitalWrite(PIN_LED_GREEN, mfmFaultCondition ? HIGH : LOW);
#endif
}

bool MFM_init(void) {
    mfmInspiratoryAirVolumeSumMilliliters = 0;

    // Set power on (hardware v3)
    pinMode(MFM_POWER_CONTROL, OUTPUT);
    digitalWrite(MFM_POWER_CONTROL, MFM_POWER_ON);
    delay(100);  // sfm3300 worst case boot time.

    // Set the timer
    massFlowTimer = new HardwareTimer(MASS_FLOW_TIMER);

    // Prescaler; stm32f411 clock is 100 mHz
    massFlowTimer->setPrescaleFactor((massFlowTimer->getTimerClkFreq() / MASS_FLOW_TIMER_FREQ) - 1);

    // Set the period
    massFlowTimer->setOverflow(MASS_FLOW_PERIOD);
    massFlowTimer->setMode(MASS_FLOW_CHANNEL, TIMER_OUTPUT_COMPARE, NC);
    massFlowTimer->attachInterrupt(MFM_Timer_Callback);

    // Interrupt priority is documented here:
    // https://stm32f4-discovery.net/2014/05/stm32f4-stm32f429-nvic-or-nested-vector-interrupt-controller/
    // WARNING : since 1.9.0 lib, I2C is on level 2. must be under...
    massFlowTimer->setInterruptPriority(3, 0);

    // default Wire instance is on PB8 BP9, anyway
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    // Wire.setClock(400000); // honeywell do support, but no information about sfm3300d

#if MASS_FLOW_METER_SENSOR == MFM_SFM3019
    Wire.begin();
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    Wire.endTransmission();
    Wire.end();
    delay(4);

    // start air continuous measurement
    Wire.begin();
    Wire.beginTransmission(0x2E);
    Wire.write(0x36);
    Wire.write(0x08);
    Wire.endTransmission();
    Wire.end();

    delay(2);
    // //Stop continuous measurement 0x3FF9
    // Wire.begin();
    // Wire.beginTransmission(0x2E);
    // Wire.write(0x3F);
    // Wire.write(0xF9);
    // Wire.endTransmission();
    // Wire.end();

    // delay(2);
    //  //Read Scale Factor, Offset, and Flow Unit 0x3661
    // Wire.begin();
    // Wire.beginTransmission(0x2E);
    // Wire.write(0x36);
    // Wire.write(0x61);
    // Wire.endTransmission();
    // Wire.end();

    // Wire.begin();
    // int ccc = Wire.requestFrom(0x2E, 8);
    // Serial.print(ccc);
    // Serial.print(" ");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");

    // Wire.end();

    // delay(5);
    //  //Read serial number
    // Wire.begin();
    // Wire.beginTransmission(0x2E);
    // Wire.write(0xE1);
    // Wire.write(0x02);
    // Wire.endTransmission();
    // Wire.end();

    // Wire.begin();
    // int ccc2 = Wire.requestFrom(0x2E, 20);
    // Serial.print(ccc2);
    // Serial.print(" ");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.print(Wire.read()); Serial.print(",");
    // Serial.println(".");
    // Wire.end();

    // delay(10000);
#endif

#if MASS_FLOW_METER_SENSOR_EXPI == MFM_SFM_3300D
    Wire.begin();  // Join I2C bus (address is optional for master)
    Wire.beginTransmission(MFM_SENSOR_EXPI_I2C_ADDRESS);
    Wire.write(0x20);  // 0x2000 soft reset
    Wire.write(0x00);
    uint32_t errorCount = Wire.endTransmission();
    delay(5);  // end of reset

    Wire.beginTransmission(MFM_SENSOR_EXPI_I2C_ADDRESS);
    Wire.write(0x31);  // 0x31AE read serial
    Wire.write(0xAE);
    errorCount += Wire.endTransmission();

    errorCount += ((6u == Wire.requestFrom(MFM_SENSOR_EXPI_I2C_ADDRESS, 6)) ? 0u : 1u);
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
        mfmSfm3300SerialNumberExpi = sn_expi;
    }
    delay(10);
    Wire.beginTransmission(MFM_SENSOR_EXPI_I2C_ADDRESS);
    Wire.write(0x10);  // 0x1000 start measurement
    Wire.write(0x00);
    errorCount += Wire.endTransmission();
    Wire.end();
    delay(100);  // wait 100ms before having available data.

    if (errorCount != 0u) {
        mfmFaultCondition = true;
        mfmResetStateMachine = MFM_WAIT_RESET_PERIODS;
    }
#endif

#if MASS_FLOW_METER_SENSOR == MFM_HONEYWELL_HAF

    /*
    Init sequence for Honeywell Zephyr mass flow sensor:
    1st read operation: the sensor will send 0x0000
    2nd read operation: the sensor will send the first part of the serial number
    3rd read operation: the sensor will send the second part of the serial number
    Subsequent read operations: the sensor will send calibrated mass air flow values with two
    leading 0
    */
    Wire.begin();
    Wire.beginTransmission(MFM_SENSOR_I2C_ADDRESS);
    Wire.write(0x02);  // Force reset
    uint8_t txOk = Wire.endTransmission();
    Wire.end();
    delay(30);

    u_int32_t sn = 0;
    Wire.begin();
    Wire.beginTransmission(MFM_SENSOR_I2C_ADDRESS);
    uint8_t rxcount = Wire.requestFrom(MFM_SENSOR_I2C_ADDRESS, 2);
    sn = Wire.read();
    sn <<= 8;
    sn |= Wire.read();  // first transmission is serial number register 0
    sn <<= 8;
    delay(2);  // if you do not wait, sensor will send again register 0
    rxcount += Wire.requestFrom(MFM_SENSOR_I2C_ADDRESS, 2);
    sn |= Wire.read();
    sn <<= 8;
    sn |= Wire.read();  // second transmission is serial number register 1

    if ((txOk != 0u) || (rxcount != 4u)) {  // If transmission failed
        mfmFaultCondition = true;
        mfmResetStateMachine = MFM_WAIT_RESET_PERIODS;
    } else {
        mfmHoneywellHafSerialNumber = sn;
    }
    Wire.end();

#if MODE == MODE_MFM_TESTS
    Serial.println("Read 1");
    Serial.println(mfmLastData.i);
    Serial.println("fault condition:");
    Serial.println(mfmFaultCondition ? "failure" : "no failure");
#endif
    delay(100);
#endif

    massFlowTimer->resume();
    return !mfmFaultCondition;
}

int32_t MFM_read_airflow(void) {
    int32_t r;
    if (mfmFaultCondition) {
        r = MASS_FLOW_ERROR_VALUE;
    } else {
        r = mfmInspiratoryInstantAirFlow - mfmInspiratoryCalibrationOffset;
    }
    return r;
}

// cppcheck-suppress unusedFunction
int32_t MFM_expi_read_airflow(void) {
    int32_t r;
    if (mfmFaultCondition) {
        r = MASS_FLOW_ERROR_VALUE;
    } else {
        r = mfmExpiratoryInstantAirFlow - mfmExpiratoryCalibrationOffset;
    }
    return r;
}

void MFM_reset(void) { mfmInspiratoryAirVolumeSumMilliliters = 0; }

void MFM_expi_reset(void) { mfmExpiratoryAirVolumeSumMilliliters = 0; }

// cppcheck-suppress unusedFunction
uint32_t MFM_read_serial_number(void) {
#if MASS_FLOW_METER_SENSOR == MFM_HONEYWELL_HAF
    return mfmHoneywellHafSerialNumber;
#endif
    return 0;
}

#if MASS_FLOW_METER_SENSOR_EXPI == MFM_SFM_3300D
/**
 * return the serial number of the expiratory flow meter
 *
 * @note returns 0 before init, or if init failed.
 */
// cppcheck-suppress unusedFunction
uint32_t MFM_expi_read_serial_number(void) { return mfmSfm3300SerialNumberExpi; }

#endif

/**
 *  If the massflow meter needs to be calibrated, this function will be usefull.
 */
int8_t MFM_calibrateZero(void) {
    int8_t ret = MFM_CALIBRATION_OK;
    // activate table fill with last valid value
    mfmInspiratoryInstantAirFlowLastValuesIndex = 0;
    mfmInspiratoryInstantAirFlowRecord = true;
    // wait for the table to fill in
    delay(2 + (MFM_MEAN_SAMPLES * (MASS_FLOW_PERIOD / 10)));
    // Check that table is full (record must be false)
    // If it is not, there is a sensor problem
    // In case of problem, do not update mfmInspiratoryCalibrationOffset
    if (!mfmInspiratoryInstantAirFlowRecord) {
        int32_t zeroFlow = 0;
        for (int16_t i = 0; i < MFM_MEAN_SAMPLES; i++) {
            zeroFlow += mfmInspiratoryInstantAirFlowLastValues[i];
        }
        zeroFlow /= MFM_MEAN_SAMPLES;
        // check that value is credible: [-10 10] SLPM
        if ((zeroFlow < 10000) && (zeroFlow > -10000)) {
            mfmInspiratoryCalibrationOffset = zeroFlow;
        } else {
            ret = MFM_CALIBRATION_OUT_OF_RANGE;
        }
    } else {
        ret = MFM_CALIBRATION_IMPOSSIBLE;
    }
    return ret;
}

/**
 *  Get massflow meter offset
 */
int32_t MFM_getOffset(void) { return mfmInspiratoryCalibrationOffset; }

int32_t MFM_read_milliliters(bool reset_after_read) {
    int32_t result = MASS_FLOW_ERROR_VALUE;

#if MASS_FLOW_METER_SENSOR == MFM_HONEYWELL_HAF
    // period is MASS_FLOW_PERIOD / 10000  (100 µs prescaler)
    result = mfmFaultCondition
                 ? MASS_FLOW_ERROR_VALUE
                 : ((mfmInspiratoryAirVolumeSumMilliliters * MASS_FLOW_PERIOD) / (60 * 10000));
#endif

    if (reset_after_read) {
        MFM_reset();
    }

    return result;
}
// cppcheck-suppress unusedFunction
int32_t MFM_expi_read_milliliters(bool reset_after_read) {
    int32_t result = MASS_FLOW_ERROR_VALUE;

#if MASS_FLOW_METER_SENSOR_EXPI == MFM_SFM_3300D
    result = mfmFaultCondition
                 ? MASS_FLOW_ERROR_VALUE
                 : ((mfmExpiratoryAirVolumeSumMilliliters * MASS_FLOW_PERIOD) / (60 * 10000));
#endif
    if (reset_after_read) {
        MFM_expi_reset();
    }

    return result;
}

#if MODE == MODE_MFM_TESTS

void onStartClick() {
    MFM_reset();
    MFM_expi_reset();
}
void onPauseClick() { MFM_calibrateZero(); }
char buffer[30];

OneButton btn_alarmoff(PIN_BTN_ALARM_OFF, false, false);
OneButton btn_pause(PIN_BTN_STOP, false, false);

void setup(void) {
    Serial.begin(115200);
    Serial.println("init mass flow meter");
    pinMode(PIN_LED_START, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);

    boolean ok = MFM_init();
    int calib = MFM_calibrateZero();

    startScreen();
    resetScreen();
    screen.setCursor(0, 0);
    screen.print("debug prog");
    screen.setCursor(0, 1);
    screen.print("mass flow sensor");
    screen.setCursor(0, 2);
    screen.print(ok ? "sensor OK" : "sensor not OK");
    screen.setCursor(0, 3);
    screen.print(calib == MFM_CALIBRATION_OK ? "calib OK" : "calibration error");

    (void)snprintf(buffer, sizeof(buffer), "serial=%08x ", MFM_read_serial_number());
    Serial.println(buffer);

#ifdef MASS_FLOW_METER_SENSOR_EXPI
    (void)snprintf(buffer, sizeof(buffer), "serial expi=%08d ", MFM_expi_read_serial_number());
    Serial.println(buffer);
#endif

    Serial.print("calibration status=");
    Serial.println(calib);
    Serial.print("offset calibration=");
    Serial.println(mfmInspiratoryCalibrationOffset);

    btn_alarmoff.attachClick(onStartClick);
    btn_alarmoff.setDebounceTicks(0);
    btn_pause.attachClick(onPauseClick);
    btn_pause.setDebounceTicks(0);
    mfmInspiratoryAirVolumeSumMilliliters = 0;
    Serial.println("init done");
}

int loopcounter = 0;

void loop(void) {
    delay(10);
    loopcounter++;
    if (loopcounter == 50) {
        loopcounter = 0;

        int32_t volume = MFM_read_milliliters(false);
        int32_t volumeExpi = MFM_expi_read_milliliters(false);

        resetScreen();
        screen.setCursor(0, 0);
        screen.print("mass flow sensor");
        screen.setCursor(0, 1);

        if (volume == MASS_FLOW_ERROR_VALUE) {
            screen.print("sensor not OK");
        } else {
            screen.print("sensor OK");
        }
        // screen.print(mfmInspiratoryLastValue);
        screen.setCursor(0, 2);
        (void)snprintf(buffer, sizeof(buffer), "->vol=%dmL %dmLpm ", volume, MFM_read_airflow());
        screen.print(buffer);

        screen.setCursor(0, 3);
        (void)snprintf(buffer, sizeof(buffer), "<-vol=%dmL %dmLpm ", volumeExpi,
                       MFM_expi_read_airflow());
        screen.print(buffer);

        // Serial.print(mfmLastValueFloat*1000);
        // Serial.print(",");

        // Serial.print("volume = ");
        // Serial.print(volume);
        // Serial.println("mL");
    }
    Serial.print(MFM_read_airflow());
    Serial.print(",");
    Serial.println(MFM_expi_read_airflow());

    btn_alarmoff.tick();
    btn_pause.tick();
}
#endif

#endif
