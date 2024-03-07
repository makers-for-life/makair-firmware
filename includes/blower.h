/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file blower.h
 * @brief Tools to control the blower
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

// Internal libraries
#include "../includes/debug.h"
#include "../includes/parameters.h"

/**
 * Convert a speed to a value in microseconds for the blower controller
 *
 * @note There is an error margin on the max ppm, because the blower do not handle values greater
 * than 2.01 ms, and there is no quartz anywhere
 */
#define BlowerSpeed2MicroSeconds(value) map(value, 0, 1800, 0, 50)

// CLASS =================================================================

/// Controls a blower
class Blower {
 public:
    /// Default constructor
    Blower();

    /**
     * Parameterized constructor
     *
     * @param p_hardwareTimer Hardware time for the blower
     * @param p_timerChannel TIM channel for this blower
     * @param p_blowerPin Data pin for this blower
     */
    Blower(HardwareTimer* p_hardwareTimer, uint16_t p_timerChannel, uint16_t p_blowerPin);

    /// Initialize the hardware timer used to control the blower
    void setup();

    /**
     * Run the blower to a given speed applying a ramp-up to prevent high current drain
     *
     * @param p_targetSpeed Speed between MIN_BLOWER_SPEED and MAX_BLOWER_SPEED
     */
    void runSpeedWithRampUp(uint16_t p_targetSpeed);

    /**
     * Run the blower to a given speed
     *
     * @param p_runSpeed Speed between MIN_BLOWER_SPEED and MAX_BLOWER_SPEED
     */
    void runSpeed(uint16_t p_runSpeed);

    void execute();

    /// Stops the blower
    void stop();

    /// Get speed value
    uint16_t getSpeed() const;

    /// Get target speed value
    uint16_t getTargetSpeed() const;
    /**
     * Given a flow in mL/min, return an estimated pressure just at the output of the blower. This
     * pressure has been determined using the pressure-flow characteristic of the blower
     *
     * @param p_flow inspiratory flow in mL/min
     */
    int32_t getBlowerPressure(int32_t p_flow);

 private:
    /// Hardware timer used to control the blower
    HardwareTimer* actuator;

    /// Channel of the hardware timer used to control the blower
    uint16_t timerChannel;

    /// Pin of the blower
    uint16_t blowerPin;

    /// Current speed
    uint16_t m_speed;

    /// target speed
    uint16_t m_targetSpeed;

    /// Current state
    bool m_stopped;

    // Last call of the runspeed function of the blower
    uint32_t m_lastCallDate;
};

extern Blower blower;
