/*
 * Smart UPS Addon: fan controller
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef _FAN_H_
#define _FAN_H_

#include <stdbool.h>
#include <stdint.h>

#include "../lib/timekeeping.h"

/*
 * recalculate and return fan RPM - takes a relatively long time to run so
 * avoid calling it too often
 */
uint16_t fan_rpm(void);

/*
 * check whether the fan has failed - it was supposed to be running
 * but it is not
 *
 * a disabled fan is not considered to have failed
 */
bool fan_has_failed(void);

/*
 * should be called from time to time
 * (at least when the time returned by fan_get_next_poll_time() comes)
 */
void fan_poll(void);
void fan_get_next_poll_time(timestamp *next_poll);

/* request a particular fan mode */
void fan_disable(void);
void fan_enable_low(void);
void fan_enable_high(void);

/*
 * setup the fan controller: must be called before any other fan function,
 * must be called with interrupts disabled, uses timekeeping functions
 */
void fan_setup(void);

#endif
