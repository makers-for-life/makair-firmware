/******************************************************************************
 * @author Makers For Life
 * @copyright Copyright (c) 2020 Makers For Life
 * @file debug.h
 * @brief Debug helpers
 *****************************************************************************/

#pragma once

// INCLUDES ===================================================================

#include "../includes/config.h"

// FUNCTIONS ==================================================================

/**
 * Expand arbitrary code only when in debug mode
 *
 * @param statement  A statement or a block of statements
 */
#if DEBUG == 1
#define DBG_DO(statement) statement
#else
#define DBG_DO(statement)
#endif
