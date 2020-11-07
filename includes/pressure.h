/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure.h
 * @brief Pressure sensor
 *****************************************************************************/

#pragma once

#include <stdint.h>

/// Offset aware reading class for the pressure sensor
class PressureSensor {
 public:
    /// Default constructor
    PressureSensor();

    /**
     * Read the current pressure for the feedback control
     *
     * @return The current pressure in mmH2O
     */
    int32_t read();

    /**
     * Set the pressure sensor offset
     *
     * @param p_pressureSensorOffest Pressure offset in mmH2O
     */
    void setPressureSensorOffset(int32_t p_pressureSensorOffest) {
        m_PressureSensorOffset = p_pressureSensorOffest;
    }

 private:
    /// Pressure sensor offset in mmH2O
    int32_t m_PressureSensorOffset;
};

extern PressureSensor inspiratoryPressureSensor;
