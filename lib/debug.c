/*
 * AVR Library: debug subsystem
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <avr/cpufunc.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include "debug.h"

#ifdef ENABLE_DEBUG_LOG

#ifndef DEBUG_BUF_SIZE
#define DEBUG_BUF_SIZE UINT8_MAX
#endif

#if DEBUG_BUF_SIZE <= UINT8_MAX
#define DEBUG_BUF_LEN_T uint8_t
#else
#if DEBUG_BUF_SIZE > UINT16_MAX
#error DEBUG_BUF_SIZE too big
#endif

#define DEBUG_BUF_LEN_T uint16_t
#endif

static uint8_t debug_buf[DEBUG_BUF_SIZE];
static DEBUG_BUF_LEN_T debug_buf_first_element;
static DEBUG_BUF_LEN_T debug_buf_valid_length;

static debug_output_fun debug_output_notify;

static int debug_putc(char c, FILE *stream);

static FILE dbgout = FDEV_SETUP_STREAM(debug_putc, NULL, _FDEV_SETUP_WRITE);

static void debug_buf_reset_atomic(void)
{
	debug_buf_first_element = debug_buf_valid_length = 0;
}

bool debug_buf_is_empty_atomic(void)
{
	return debug_buf_valid_length == 0;
}

static void debug_put_atomic(uint8_t data)
{
	DEBUG_BUF_LEN_T idx;
	idx = (debug_buf_first_element +
	       debug_buf_valid_length) % DEBUG_BUF_SIZE;

	debug_buf[idx] = data;

	if (debug_buf_valid_length < DEBUG_BUF_SIZE)
		debug_buf_valid_length++;
}

uint8_t debug_buf_get_atomic(void)
{
	DEBUG_BUF_LEN_T idx = debug_buf_first_element;
	uint8_t data = debug_buf[idx];

	debug_buf_first_element++;
	debug_buf_first_element %= DEBUG_BUF_SIZE;

	debug_buf_valid_length--;

	return data;
}

static void debug_output(void)
{
	if (debug_output_notify != NULL)
		debug_output_notify();
}

bool debug_buf_is_empty(void)
{
	bool ret;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();
		ret = debug_buf_is_empty_atomic();
		_MemoryBarrier();
	}

	return ret;
}

void debug_put(uint8_t in)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();

		debug_put_atomic(in);

		_MemoryBarrier();
	}

	debug_output();
}

static int debug_putc(char c, FILE *stream)
{
	debug_put((uint8_t)c);

	return 0;
}

void debug_put_str(uint8_t *in, uint8_t len)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();

		for (uint8_t ctr = 0; ctr < len; ctr++)
			debug_put_atomic(in[ctr]);

		_MemoryBarrier();
	}

	debug_output();
}

void debug_put_str_P(PGM_P *in, uint8_t len)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();

		for (uint8_t ctr = 0; ctr < len; ctr++)
			debug_put_atomic(pgm_read_byte(&in[ctr]));

		_MemoryBarrier();
	}

	debug_output();
}

bool debug_buf_get(uint8_t *out)
{
	bool ret;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();

		if (!debug_buf_is_empty_atomic()) {
			*out = debug_buf_get_atomic();
			ret = true;
		} else
			ret = false;

		_MemoryBarrier();
	}

	return ret;
}

void debug_set_output_notify(debug_output_fun callback)
{
	debug_output_notify = callback;
}

#endif

void debug_setup(void)
{
#ifdef ENABLE_DEBUG_LOG
	debug_buf_reset_atomic();

	debug_output_notify = NULL;

	stderr = &dbgout;
#endif
}
