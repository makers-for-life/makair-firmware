/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file config.h
 * @brief Main configuration
 *****************************************************************************/

#pragma once

// Available modes
#define MODE_PROD 1              ///< Ventilator mode
#define MODE_QUALIFICATION 2     ///< Test electrical wiring mode
#define MODE_INTEGRATION_TEST 3  ///< Test integration mode

/// Defines the current mode
#define MODE MODE_PROD

/**
 * Activates debug traces
 *
 * When DEBUG = 1, additional code is added to send debug traces using serial
 */
#define DEBUG 1

/**
 * Activates pressure sensor simulation
 *
 * When SIMULATION = 1, the current pressure value will not be read from the sensor but computed by
 * a basic and deterministic model
 */
#define SIMULATION 0

// Available pneumatic versions
#define PHW_PIGGY 0
#define PHW_CHU 1
#define PHW_FAULHABER 2

/// Defines which preset to use for controlling pressure
#define PNEUMATIC_HARDWARE_VERSION PHW_CHU

// Available valves
#define VT_SERVO_V1 0
#define VT_EMERSON_ASCO 1
#define VT_FAULHABER 2

// Defines which valves are fitted
#define VALVE_TYPE VT_SERVO_V1

// Available Mass Flow Meters
#define MFM_SFM_3300D 1
#define MFM_SDP703_02 2
#define MFM_OMRON_D6F 3
#define MFM_HONEYWELL_HAF 4

// Defines if the device has a Mass Flow Meter or not. Comment out when no sensor.
// Note Hardware v1 cannot support any mass flow meter sensor.
// #define MASS_FLOW_METER

// Defines the type and the range of the mass flow meter.
#define MASS_FLOW_METER_SENSOR MFM_HONEYWELL_HAF
#define MFM_RANGE 100


/**
 * Defines which hardware preset to use
 *
 * This changes which hardware is used, to which pins it is connected, as well as various parameters
 */
#define HARDWARE_VERSION 1
