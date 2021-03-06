/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file vc_ac_controller.cpp
 * @brief PID for AC volume control
 *****************************************************************************/

// INCLUDES ==================================================================

// Associated header
#include "../includes/vc_ac_controller.h"

// External
#include "Arduino.h"
#include <algorithm>

// Internal

#include "../includes/main_controller.h"
#include "../includes/pressure_valve.h"

// INITIALISATION =============================================================

VC_AC_Controller vcAcController;

// FUNCTIONS ==================================================================

void VC_AC_Controller::exhale() {
    VC_CMV_Controller::exhale();

    // Calculate max pressure for the last samples
    int32_t maxPressureValue = mainController.lastPressureValues()[0];
    for (uint8_t i = 0; i < MAX_PRESSURE_SAMPLES; i++) {
        if (mainController.lastPressureValues()[i] > maxPressureValue) {
            maxPressureValue = mainController.lastPressureValues()[i];
        }
    }

    // In case the pressure trigger mode is enabled, check if inspiratory trigger is raised
    if ((mainController.tick()
         > (mainController.ticksPerInhalation() + (700u / MAIN_CONTROLLER_COMPUTE_PERIOD_MS)))) {
        // m_peakPressure > CONST_MIN_PEAK_PRESSURE ensures that the patient is plugged on the
        // machine
        if (((mainController.pressure())
                 < (maxPressureValue - (mainController.pressureTriggerOffsetCommand()))
             && (mainController.peakPressureMeasure() > CONST_MIN_PEAK_PRESSURE))
            || mainController.pressure() < -mainController.pressureTriggerOffsetCommand()) {
            mainController.setTrigger(true);
        }
    }
}
