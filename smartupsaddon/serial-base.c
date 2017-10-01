/*
 * Smart UPS Addon: serial ports instantiation
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <stdbool.h>
#include <stdint.h>

#include "serial-base.h"

#define SERIAL1_BAUD 2400

#define SERIAL_BUF_SIZE_DEFAULT 64
#define SERIAL0_BUF_SIZE_RX SERIAL_BUF_SIZE_DEFAULT

#ifndef ENABLE_DEBUG_LOG
#define SERIAL0_BAUD SERIAL1_BAUD
#define SERIAL0_BUF_SIZE_TX SERIAL_BUF_SIZE_DEFAULT
#else
#define SERIAL0_BAUD 115200
#define SERIAL0_BUF_SIZE_TX UINT8_MAX
#define SERIAL_DEBUG_OUT_PORT 0
#endif

#define SERIAL1_BUF_SIZE_RX SERIAL_BUF_SIZE_DEFAULT
#define SERIAL1_BUF_SIZE_TX SERIAL_BUF_SIZE_DEFAULT

#define BAUD SERIAL0_BAUD
#include <util/setbaud.h>
static inline uint16_t serial0_get_ubrr(void)
{
	return UBRR_VALUE;
}

static inline bool serial0_get_2x(void)
{
	return USE_2X;
}
#undef BAUD

#define BAUD SERIAL1_BAUD
#include <util/setbaud.h>
static inline uint16_t serial1_get_ubrr(void)
{
	return UBRR_VALUE;
}

static inline bool serial1_get_2x(void)
{
	return USE_2X;
}
#undef BAUD

#include "../lib/serial.impl.c"

SERIAL_IMPL(0, SERIAL0_BUF_SIZE_RX, SERIAL0_BUF_SIZE_TX)

SERIAL_IMPL(1, SERIAL1_BUF_SIZE_RX, SERIAL1_BUF_SIZE_TX)
