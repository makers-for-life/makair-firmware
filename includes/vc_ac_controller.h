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
};

extern VC_AC_Controller vcAcController;
