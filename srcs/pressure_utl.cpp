/******************************************************************************
 * @file pressure_utl.cpp
 * @copyright Copyright (c) 2020 Makers For Life
 * @author Makers For Life
 * @brief Pressure computing utility function
 *****************************************************************************/

// INCLUDES ===================================================================

#include <algorithm>

#include "../includes/pressure_utl.h"

// INITIALISATION =============================================================

static int32_t filteredRawPressure = 0;

static const int32_t RAW_PRESSURE_FILTER_DIVIDER = 5;

// From the datasheet:
// Transfer Function (kPa): Vout = VS×(0.09×P + 0.04)
// VS is 5.05 volt typical on our boards
// Transfer function for P in mmH2O, with 1 kPa = 101.97162129779 mmH2O
// Vout = VS×(0.09×(P/101.9716) + 0.04)
// P = (101.9716/0.09)×(Vout/VS-0.04)
// P(mmH20) = 1133(Vout/VS) - 45
// The offset really depend on the sensor itself with a great variation
// There must be a calibration sequence to set the zero.
//
// VS is 5.05V.
//
// There is a voltage divider in between Vout and Vadc
// Vadc = 68/83 Vout
//
// With current ADC (12 bits), and mean Vref=3.348V:
// Vadc = 3.348 * RawAdc/4096 = 0,000817382 * RawAdc
//
// Vout = 83/68 * 0,000817382 * RawAdc
// Vout = 0,000997686 * RawAdc
//
// Put it together:
// P(mmH20) = 1133((0,000997686 * RawAdc)/5.05) - 45
// P(mmH20) = 0,223837466 * RawAdc - 45

static const int16_t RAW_PRESSURE_TO_MMH20_CONSTANT = 45;
static const int32_t RAW_PRESSURE_TO_MMH20_NUM = 2238;
static const int32_t RAW_PRESSURE_TO_MMH20_DEN = 10000;

// FUNCTIONS ==================================================================

int16_t convertSensor2Pressure(uint32_t sensorValue) {
    int32_t rawPressure = static_cast<int32_t>(sensorValue);
    int32_t delta = rawPressure - filteredRawPressure;

    // Adjust delta so that the division result will be rounded away from zero.
    // This is needed to guaranty that filteredRawPressure will reach
    // rawPressure when it is constant.
    int32_t rounding = RAW_PRESSURE_FILTER_DIVIDER - 1;
    delta += (delta > 0) ? rounding : -rounding;
    filteredRawPressure += delta / RAW_PRESSURE_FILTER_DIVIDER;

    int16_t scaledRawPressure =
        filteredRawPressure * RAW_PRESSURE_TO_MMH20_NUM / RAW_PRESSURE_TO_MMH20_DEN;
    return scaledRawPressure - RAW_PRESSURE_TO_MMH20_CONSTANT;
}

// cppcheck-suppress unusedFunction
void resetFilteredRawPressure() { filteredRawPressure = 0; }
