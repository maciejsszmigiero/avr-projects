/*
 * AVR Library: Microchip TC74 driver
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef _LIB_TC74_H_
#define _LIB_TC74_H_

#include <stdbool.h>
#include <stdint.h>

#include "timekeeping.h"

typedef enum { TC74_IDLE, TC74_CONFIG_READ_DO, TC74_CONFIG_READ,
	       TC74_CONFIG_WRITE,
	       TC74_CONFIG_WRITE_CONFIG_READ_DO, TC74_CONFIG_WRITE_CONFIG_READ,
	       TC74_DATA_READY_WAIT_INIT, TC74_DATA_READY_WAIT,
	       TC74_DATA_READY_CONFIG_READ_DO, TC74_DATA_READY_CONFIG_READ,
	       TC74_TEMP_READ, TC74_TEMP_READ_OK } tc74_states;

typedef struct {
	uint8_t addr;

	/* tc74_states */ uint8_t state;
	bool state_changed;

	bool i2c_trans_complete;
	bool i2c_trans_success;
	uint8_t i2c_rdlen_actual;

	timestamp next_data_ready_poll;
	uint8_t data_ready_polls;

	bool get_temp_result;

	uint8_t config;
	int8_t temp;
} tc74_data;

/*
 * check if given tc74 instance is busy
 * busy status won't change if interrupts are disabled and no other
 * functions on this instance are called (other than read-only ones like
 * tc74_get_next_poll_time())
 */
static inline bool tc74_is_busy(tc74_data *data)
{
	return data->state != TC74_IDLE;
}

/*
 * start a temperature read on given tc74 instance
 * can only be called successfully if this instance isn't busy
 * (will return false otherwise)
 *
 * if the function returned true then the caller should wait until
 * this instance is no longer busy, then could get the result of
 * the read via tc74_get_temperature_result()
 */
bool tc74_get_temperature(tc74_data *data);

/*
 * returned temperature is only valid if this function returned true
 * (which means that the last temperature read was successful)
 */
bool tc74_get_temperature_result(tc74_data *data, int8_t *temperature);

/*
 * should be called from time to time on each instance
 * (at least when the time returned by tc74_get_next_poll_time() comes)
 */
void tc74_poll(tc74_data *data);

/*
 * should be called with interrupts disabled because enabling interrupts at any
 * time later invalidates the returned value
 *
 * this function doesn't change any internal state (it is purely read-only)
 */
void tc74_get_next_poll_time(tc74_data *data, timestamp *next_poll);

/*
 * init an tc74 instance: must be called before any other tc74 function
 * on this instance.
 * data is a caller-allocated variable, addr is an i2c address of this instance
 */

void tc74_init(tc74_data *data, uint8_t addr);

#endif
