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
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
void dxl_EnableTorque(bool Enable);
void dxl_LED(bool Power);
void dxl_PWM(int16_t PWM);
void dxl_Position(uint32_t Pos_Pulse);
void dxl_OperatingMode(char Mode[3]);
void dxl_Write_Frame();
void dxl_Read_Data();

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
     * Request opening of the Pressure Valve with a given angle
     *
     * @param p_command The angle in degree
     */
    void open(uint16_t p_command);

    /**
     * Request opening of the Pressure Valve with a given angle with linearization
     *
     * @param p_command The angle in degree
     * @return The linearized angle calculated by this function
     */

    /// Request closing of the Pressure Valve
    void close();

    /**
     * Command the valve to go to the requested aperture
     *
     * @note Nothing will happen if this function is not called after requesting a new aperture
     */
    void execute();
    void openSection(int32_t p_sectionMultiplyBy100);
    uint16_t openLinear(uint16_t p_command);

    /// Minimum valve aperture angle in degrees
    inline uint16_t minAperture() const { return minApertureAngle; }

    /// Maximum valve aperture angle in degrees
    inline uint16_t maxAperture() const { return maxApertureAngle; }

    /// Value of the requested aperture
    uint16_t command;

    /// Current aperture
    uint16_t position;

    /// Current aperture linear
    uint16_t positionLinear;

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
