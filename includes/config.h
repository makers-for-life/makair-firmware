/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file config.h
 * @brief Main configuration
 *****************************************************************************/

#pragma once

// Available modes
#define MODE_PROD 1       ///< Ventilator mode
#define MODE_MFM_TESTS 4  ///< Mass Flow Meter debugging mode

/// Defines the current mode
#define MODE MODE_PROD

/**
 * Activates debug traces
 *
 * When DEBUG = 1, additional code is added to send debug traces using serial
 * When DEBUG = 2, information are sent in serial for PID debug
 */
#define DEBUG 0

// Simulation mode allow to run main controller on a computer
#define SIMULATION false

// Defines if the device has a Mass Flow Meter or not
// Comment out when no sensor
#define MASS_FLOW_METER

// Available Mass Flow Meters
#define MFM_SFM_3300D 1
#define MFM_SDP703_02 2
#define MFM_HONEYWELL_HAF 4

// Defines the type and the range of the mass flow meter
#define MASS_FLOW_METER_SENSOR MFM_HONEYWELL_HAF
#define MFM_RANGE 200  // in SLM (standard liter per minute)
