/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file pressure.h
 * @brief abstract class for a ventilation controller
 *****************************************************************************/

#pragma once

#include "../includes/parameters.h"

// = 0 means that the method is not implemented in this class
class VentilationController {

	public:
	virtual void setup() = 0;
    virtual void initCycle() = 0;
    virtual void inhale() = 0;
    virtual void exhale() = 0;
    virtual void endCycle() = 0;

    private:
};