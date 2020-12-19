/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file VC_AC_Controller.h
 * @brief PID for AC volume control
 *****************************************************************************/

#pragma once

#include "../includes/parameters.h"
#include "../includes/vc_cmv_controller.h"

/// Controller for the AC mode
class VC_AC_Controller final : public VC_CMV_Controller {
 public:
    /// Control the exhalation
    void exhale() override;

    /// List of alarms that must be enabled for this mode
    struct Alarms enabledAlarms() const override {
        struct Alarms a = {0u,        RCM_SW_2,  RCM_SW_3, RCM_SW_4,  RCM_SW_5,
                           RCM_SW_6,  RCM_SW_7,  RCM_SW_8, RCM_SW_9,  RCM_SW_10,
                           RCM_SW_11, RCM_SW_12, 0u,       RCM_SW_15, RCM_SW_16,
                           RCM_SW_18, RCM_SW_19, 0u,       0u};
        return a;
    }
};

extern VC_AC_Controller vcAcController;
