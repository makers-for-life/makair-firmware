/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file PC_AC_Controller.h
 * @brief PID for AC pressure control
 *****************************************************************************/

#pragma once

#include "../includes/parameters.h"
#include "../includes/pc_cmv_controller.h"

/// Controller for the AC mode
class PC_AC_Controller final : public PC_CMV_Controller {
 public:
    /// Control the exhalation
    void exhale() override;

    /// List of alarms that must be enabled for this mode
    struct Alarms enabledAlarms() const override {
        struct Alarms a = {RCM_SW_1,  RCM_SW_2,  RCM_SW_3,  RCM_SW_4,  RCM_SW_5,
                           RCM_SW_6,  RCM_SW_7,  RCM_SW_8,  RCM_SW_9,  0u,
                           RCM_SW_11, RCM_SW_12, RCM_SW_14, RCM_SW_15, RCM_SW_16,
                           RCM_SW_18, RCM_SW_19, RCM_SW_20, RCM_SW_21, RCM_SW_22};
        return a;
    }
};

extern PC_AC_Controller pcAcController;
