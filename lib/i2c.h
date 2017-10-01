/*
 * AVR Library: I2C subsystem
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef _LIB_I2C_H_
#define _LIB_I2C_H_

#include <stdbool.h>
#include <stdint.h>

#include "timekeeping.h"

/*
 * if success is true then rdlen_actual contains the length of data
 * that was actually read
 */
typedef void (*i2c_completion_fun)(void *data, bool success,
				   uint8_t rdlen_actual);

/*
 * add an i2c transaction to the transaction queue
 *
 * wrbuf / wrlen control a first write (wrbuf can be NULL if wrlen = 0, that is,
 * there should be no write),
 * rdbuf / rdlen control a second read (rdbuf can be NULL if rdlen = 0, that is,
 * there should be no read).
 *
 * there should be either a write or a read or both.
 *
 * completion is an optional completion notification callback (called with
 * comp_data provided as the first parameter).
 *
 * if this function returns false the transaction wasn't queued and so the
 * completion notification is not going to be called.
 */
bool i2c_transaction(uint8_t addr,
		     const uint8_t *wrbuf, uint8_t wrlen,
		     uint8_t *rdbuf, uint8_t rdlen,
		     i2c_completion_fun completion, void *comp_data);

/*
 * should be called with interrupts disabled from time to time
 * (at least when the time returned by i2c_get_next_poll_time() comes)
 */
void i2c_poll_atomic(void);

/*
 * returns the maximum allowed µCU sleep period (the sleep needs to have
 * interrupts enabled) with respect to the i2c subsystem
 *
 * before calling this function disable interrupts and call i2c_poll_atomic(),
 * then this function, do not enable interrupts between them
 *
 * enabling interrupts at any time later (before the actual sleep) invalidates
 * the returned value
 *
 * this function doesn't change any internal state (it is purely read-only)
 */
void i2c_get_next_poll_time(timestamp *next_poll);

/*
 * setup the i2c subsystem: must be called before any other i2c function,
 * must be called with interrupts disabled
 */
void i2c_setup(void);

#endif
