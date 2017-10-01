/*
 * Smart UPS Addon: temperature controller
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef _TEMP_H_
#define _TEMP_H_

#include <stdbool.h>
#include <stdint.h>

#include "../lib/timekeeping.h"

/*
 * should be called from time to time
 * (at least when the time returned by temp_get_next_poll_time() comes)
 */
void temp_poll(void);

/*
 * should be called with interrupts disabled because enabling interrupts at any
 * time later invalidates the returned value
 *
 * this function doesn't change any internal state (it is purely read-only)
 */
void temp_get_next_poll_time(timestamp *next_poll);

/* get temperature sensors count */
uint8_t temp_get_count(void);

/*
 * get temperature sensor idx temperatures: current, min and max
 * (all output paramaters are optional so any can be NULL if not needed)
 *
 * doesn't reset sensor min / max values
 */
bool temp_get(uint8_t idx, int8_t *cur, int8_t *min, int8_t *max);

/* reset temperature sensor idx min / max values */
bool temp_reset_minmax(uint8_t idx);

/*
 * setup the temperature controller: must be called before any other temp
 * function, must be called with interrupts disabled, uses timekeeping and tc74
 * functions, sets up the fan controller
 */
void temp_setup(void);

#endif
