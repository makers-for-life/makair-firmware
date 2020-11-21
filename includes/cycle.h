/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file cycle.h
 * @brief Static data linked to the breathing cycle
 *****************************************************************************/

#pragma once
#define NUMBER_OF_VENTILATION_MODES 5

// ENUMS =================================================================

/// Defines the 2 main phases of the respiratory cycle
enum CyclePhases {
    /// Inspiration and inspiration holding
    INHALATION,
    /// Exhalation and pause
    EXHALATION
};

/// Supported ventilation modes
enum VentilationModes {
    /// PC-CMV (default)
    PC_CMV = 1,
    /// PC-AC
    PC_AC = 2,
    /// VC-CMV
    VC_CMV = 3,
    /// PC-VSAI
    PC_VSAI = 4,
    /// VC-AC
    VC_AC = 5,
};
