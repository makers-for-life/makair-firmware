/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file serial_control.h
 * @brief Handle control protocol on the serial input
 *****************************************************************************/

#pragma once

/**
 * Parse input and handle changes of settings
 *
 * @warning This must be used after `initTelemetry()`
 */
void serialControlLoop();
