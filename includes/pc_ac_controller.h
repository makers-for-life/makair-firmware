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

};

extern PC_AC_Controller pcAcController;
