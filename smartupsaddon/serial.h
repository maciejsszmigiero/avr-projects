/*
 * Smart UPS Addon: serial ports data processor
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "../lib/timekeeping.h"

/*
 * both should be called from time to time
 * (at least when the time returned by serial_get_next_poll_time() comes)
 *
 * serial_poll_atomic() needs interrupts disabled, serial_poll() does not
 */
void serial_poll(void);
void serial_poll_atomic(void);

/*
 * returns the maximum allowed µC sleep period (the sleep needs to have
 * interrupts enabled) with respect to serial ports
 *
 * before calling this function disable interrupts and call
 * serial_poll_atomic(), then this function, do not enable interrupts between
 * them
 *
 * enabling interrupts at any time later (before the actual sleep) invalidates
 * the returned value
 *
 * this function doesn't change any internal state (it is purely read-only)
 */
void serial_get_next_poll_time(timestamp *next_poll);

/*
 * setup serial ports: must be called before any other serial function,
 * must be called with interrupts disabled, uses timekeeping functions,
 * sets up serial ports 0 and 1
 */
void serial_setup(void);

#endif
