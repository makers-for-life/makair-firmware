/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file ventilation_controller.h
 * @brief Abstract class for ventilation controllers
 *****************************************************************************/

#pragma once

#include "../includes/alarm_controller.h"
#include "../includes/parameters.h"

/// Abstract class for ventilation controllers
// = 0 means that the method is not implemented in this class
class VentilationController {
 public:
    /// Initialize controller
    virtual void setup() = 0;

    /// Begin a new breathing cycle
    virtual void initCycle() = 0;

    /// Control the inhalation
    virtual void inhale() = 0;

    /// Control the exhalation
    virtual void exhale() = 0;

    /// End the current breathing cycle
    virtual void endCycle() = 0;

    /// List of alarms that must be enabled for this mode
    struct Alarms enabledAlarms() const {
        struct Alarms a = {RCM_SW_1,  RCM_SW_2,  RCM_SW_3,  RCM_SW_11, RCM_SW_12,
                           RCM_SW_14, RCM_SW_15, RCM_SW_16, RCM_SW_18, RCM_SW_19};
        return a;
    }

 private:
};
