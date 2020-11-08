/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file mass_flow_meter.h
 * @brief Mass flow meter management
 *****************************************************************************/

#pragma once

/**
 * CPU load in percent. Only available in production software (MODE_PROD)
 * 
 * @note If this raise to 100%, there is some blocking code somewhere.
 */
int32_t readCpuLoadPercent(void);

