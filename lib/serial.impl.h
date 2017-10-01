/*
 * AVR Library: Serial port support
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

/*
 * expand this macro to generate header file declarations for a serial port
 * with (zero-based) index of num
 */
#define SERIAL_IMPL_HEADER(num)				\
	/* setup the serial port: must be called before any */	\
	/* other function for this port, uses debug */		\
	/* functions if this port is a debug port, */		\
	/* must be called with interrupts disabled */		\
	void serial ## num ##_setup(void);			\
								\
	/* check serial RX buffer emptiness / data length, */	\
	/* enabling interrupts at any time invalidates the */	\
	/* returned value */					\
	bool serial ## num ##_rx_is_empty(void);		\
	uint8_t serial ## num ##_rx_len(void);			\
								\
	/* get the next byte in serial RX buffer and */	\
	/* remove it from the buffer */			\
	bool serial ## num ##_rx_get(uint8_t *out);		\
								\
	/* get count bytes from serial RX buffer (at the */	\
	/* beginning or at index idx), return how many were */	\
	/* actually read in act_count, do not remove them */	\
	/* from the buffer */					\
	void serial ## num ##_rx_peek(uint8_t *out,		\
				      uint8_t count,		\
				      uint8_t *act_count);	\
	void serial ## num ##_rx_peek_at(uint8_t *out,		\
					 uint8_t idx,		\
					 uint8_t count,	\
					 uint8_t *act_count);	\
								\
	/* check serial TX buffer emptiness, */		\
	/* enabling interrupts at any time invalidates */	\
	/* the returned value */				\
	bool serial ## num ##_tx_is_empty(void);		\
								\
	/* add a byte to the serial TX buffer */		\
	void serial ## num ##_tx_put(uint8_t in);
