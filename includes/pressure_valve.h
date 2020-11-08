/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure_valve.h
 * @brief Tools to control pressure valves
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Internal libraries
#include "../includes/config.h"
#include "../includes/parameters.h"

// MACROS =================================================================

/**
 * Convert an angle in degrees to a value in microseconds for the valve controller
 *
 * @param value Angle in degrees
 * @return Value in microsends for the valve controller
 */
uint16_t valveAngle2MicroSeconds(uint16_t value);

// CLASS =================================================================

/// Controls a pressure valve
class PressureValve {
 public:
    /// Default constructor
    PressureValve();

    /**
     * Parameterized constructor
     *
     * @param p_hardwareTimer       Hardware time for this valve
     * @param p_timerChannel        TIM channel for this valve
     * @param p_valvePin            Data pin for this valve
     * @param p_openApertureAngle   Open aperture angle in degrees
     * @param p_closeApertureAngle  Close aperture angle in degrees
     */
    PressureValve(HardwareTimer* p_hardwareTimer,
                  uint16_t p_timerChannel,
                  uint16_t p_valvePin,
                  uint16_t p_openApertureAngle,
                  uint16_t p_closeApertureAngle);
    /**
     * Initialize this valve
     *
     * This must be called once to be able to use this Pressure Valve
     */
    void setup();

    /// Request opening of the Pressure Valve
    void open();

    /**
     * Request opening of the Air Transistor with a given angle
     *
     * @param p_command The angle in degree
     */
    void open(uint16_t p_command);

    /// Request closing of the Pressure Valve
    void close();

    /**
     * Command the valve to go to the requested aperture
     *
     * @note Nothing will happen if this function is not called after requesting a new aperture
     */
    inline void execute() {
        // On évite d'aller plus loin que les limites de la valve
        if (command < minApertureAngle) {
            command = minApertureAngle;
        } else if (command > maxApertureAngle) {
            command = maxApertureAngle;
        } else {
        }

        if (command != position) {
            actuator->setCaptureCompare(timerChannel, valveAngle2MicroSeconds(command),
                                        MICROSEC_COMPARE_FORMAT);
            position = command;
        }
    }

    /// Minimum valve aperture angle in degrees
    inline uint16_t minAperture() const { return minApertureAngle; }

    /// Maximum valve aperture angle in degrees
    inline uint16_t maxAperture() const { return maxApertureAngle; }

    /// Value of the requested aperture
    uint16_t command;

    /// Current aperture
    uint16_t position;

 private:
    /// Minimum valve aperture angle in degrees
    uint16_t minApertureAngle;

    /// Maximum valve aperture angle in degrees
    uint16_t maxApertureAngle;

    /// Open aperture angle in degrees
    uint16_t openApertureAngle;

    /// Close aperture angle in degrees
    uint16_t closeApertureAngle;

    /// Hardware time for this valve
    HardwareTimer* actuator;

    /// TIM channel for this valve
    uint16_t timerChannel;

    /// Data pin for this valve
    uint16_t valvePin;
};

extern PressureValve expiratoryValve;
extern PressureValve inspiratoryValve;
