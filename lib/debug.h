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

#ifndef _LIB_DEBUG_H_
#define _LIB_DEBUG_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef void (*debug_output_fun)(void);

static inline bool debug_enabled(void)
{
	return
#ifdef ENABLE_DEBUG_LOG
		true
#else
		false
#endif
		;
}

#define dprintf(fmt, ...)					\
	do							\
		if (debug_enabled())				\
			fprintf(stderr, fmt,  ##__VA_ARGS__);	\
	while (0)

#define dprintf_P(fmt, ...)					\
	do							\
		if (debug_enabled())				\
			fprintf_P(stderr, fmt,  ##__VA_ARGS__); \
	while (0)

/* atomic functions can only be called with interrupts disabled */
bool debug_buf_is_empty_atomic(void);
uint8_t debug_buf_get_atomic(void);

/*
 * sets notification callback that some debug text had been output to the
 * debug buffer
 */
void debug_set_output_notify(debug_output_fun callback);

/*
 * setup the debug subsystem: must be called before any other debug function,
 * must be called with interrupts disabled
 */
void debug_setup(void);

#endif
