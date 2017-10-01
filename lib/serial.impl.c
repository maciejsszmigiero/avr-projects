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
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#include "debug.h"

/*
 * if defined then this serial port at (zero-based) index will be a debug port:
 * debug buffer content will be output there with the normal application
 * output printed inside square brackets ('[' and ']')
 */
#ifdef SERIAL_DEBUG_OUT_PORT
static bool serial_app_mode;
#else
#define SERIAL_DEBUG_OUT_PORT -1
extern bool serial_app_mode;
#endif

#define SERIAL_BUF_DEFS(num, dir, size)					\
	static uint8_t serial ## num ## _buf_ ## dir [size];			\
	static uint8_t serial ## num ## _buf_ ## dir ## _first_element;	\
	static uint8_t serial ## num ## _buf_ ## dir ## _valid_length;		\
	_Static_assert(size <= UINT8_MAX, "serial " #num " " #dir " buffer size too big");

#define SERIAL_BUF_FUNCS(num, dir, size)					\
	static void serial ## num ##_buf_ ## dir ## _reset(void)		\
	{									\
		serial ## num ## _buf_ ## dir ## _first_element =		\
			serial ## num ## _buf_ ## dir ## _valid_length = 0;	\
	}									\
										\
	static bool serial ## num ##_buf_ ## dir ## _is_empty(void)		\
	{									\
		return serial ## num ## _buf_ ## dir ## _valid_length == 0;	\
	}									\
										\
	static void serial ## num ##_buf_ ## dir ## _put(uint8_t data)		\
	{									\
		uint8_t idx;							\
		idx = (serial ## num ## _buf_ ## dir ## _first_element +	\
		       serial ## num ## _buf_ ## dir ## _valid_length) % size;	\
		serial ## num ## _buf_ ## dir [idx] = data;			\
										\
		if (serial ## num ## _buf_ ## dir ## _valid_length < size)	\
			serial ## num ## _buf_ ## dir ## _valid_length++;	\
	}									\
										\
	static uint8_t serial ## num ##_buf_ ## dir ## _get(void)		\
	{									\
		uint8_t idx = serial ## num ## _buf_ ## dir ## _first_element;	\
		uint8_t data = serial ## num ## _buf_ ## dir [idx];		\
										\
		serial ## num ## _buf_ ## dir ## _first_element++;		\
		serial ## num ## _buf_ ## dir ## _first_element %= size;	\
										\
		serial ## num ## _buf_ ## dir ## _valid_length--;		\
										\
		return data;							\
	}									\
										\
	static __attribute__((unused))						\
	void serial ## num ##_buf_ ## dir ## _peek(uint8_t *out, uint8_t count) \
	{									\
		uint8_t idx = serial ## num ## _buf_ ## dir ## _first_element;	\
		for (uint8_t ctr = 0; ctr < count; ctr++) {			\
			*out++ = serial ## num ## _buf_ ## dir [idx];		\
										\
			idx++;							\
			idx %= size;						\
		}								\
	}									\
										\
	static __attribute__((unused))						\
	void serial ## num ##_buf_ ## dir ## _peek_at(uint8_t *out,		\
						      uint8_t idx,		\
						      uint8_t count)		\
	{									\
		uint8_t rem = size -						\
			serial ## num ## _buf_ ## dir ## _first_element - 1;	\
		if (idx <= rem)						\
			idx += serial ## num ## _buf_ ## dir ## _first_element; \
		else								\
			idx -= (rem + 1);					\
										\
		for (uint8_t ctr = 0; ctr < count; ctr++) {			\
			*out++ = serial ## num ## _buf_ ## dir [idx];		\
										\
			idx++;							\
			idx %= size;						\
		}								\
	}

#define SERIAL_INTERRUPTS(num)						\
	ISR(USART ## num ## _RX_vect)					\
	{								\
		while (bit_is_set(UCSR ## num ## A, RXC ## num))	\
			serial ## num ## _buf_rx_put(UDR ## num);	\
	}								\
									\
	ISR(USART ## num ## _UDRE_vect)				\
	{								\
		bool debug_port = num == SERIAL_DEBUG_OUT_PORT;	\
		bool debug_data_present;				\
		debug_data_present = debug_port &&			\
			!debug_buf_is_empty_atomic();			\
									\
		if (!debug_data_present && serial ## num ##		\
		    _buf_tx_is_empty()) {				\
			UCSR ## num ## B &= ~_BV(UDRIE ## num);	\
			return;					\
		}							\
									\
		if (bit_is_clear(UCSR ## num ## A, UDRE ## num))	\
			return;					\
									\
		if (!serial ## num ##_buf_tx_is_empty()) {		\
			if (debug_port && !serial_app_mode) {		\
				UDR ## num = '[';			\
				serial_app_mode = true;		\
				return;				\
			}						\
									\
			UDR ## num = serial ## num ## _buf_tx_get();	\
		} else {						\
			if (debug_port && serial_app_mode) {		\
				UDR ## num = ']';			\
				serial_app_mode = false;		\
				return;				\
			}						\
									\
			UDR ## num = debug_buf_get_atomic();		\
		}							\
	}

#define SERIAL_SETUP(num)							\
	void serial ## num ##_setup(void)					\
	{									\
		power_usart ## num ## _enable();				\
										\
		UCSR ## num ## B &= ~(_BV(RXCIE ## num) | _BV(TXCIE ## num) |	\
				      _BV(UDRIE ## num) | _BV(RXEN ## num) |	\
				      _BV(TXEN ## num));			\
										\
		wdt_reset();							\
		while (bit_is_set(UCSR ## num ## A, RXC ## num)) {		\
			uint8_t tmp __attribute__((unused)) =			\
				UDR ## num;					\
		}								\
		loop_until_bit_is_set(UCSR ## num ## A, UDRE ## num);		\
										\
		UCSR ## num ## C &= ~(_BV(UMSEL ## num ## 0) |			\
				      _BV(UMSEL ## num ## 1) |			\
				      _BV(UPM ## num ## 0) |			\
				      _BV(UPM ## num ## 1) |			\
				      _BV(USBS ## num) |			\
				      _BV(UCPOL ## num) |			\
				      _BV(UCSZ ## num ## 2));			\
		UCSR ## num ## C |= _BV(UCSZ ## num ## 1) |			\
			_BV(UCSZ ## num ## 0);					\
										\
		UBRR ## num = serial ## num ##_get_ubrr();			\
		if (serial ## num ##_get_2x())					\
			UCSR ## num ## A |= _BV(U2X ## num);			\
		else								\
			UCSR ## num ## A &= ~_BV(U2X ## num);			\
										\
		serial ## num ## _buf_rx_reset();				\
		serial ## num ## _buf_tx_reset();				\
										\
		if (num == SERIAL_DEBUG_OUT_PORT) {				\
			serial_app_mode = false;				\
			debug_set_output_notify(serial ## num ##		\
						  _debug_notify);		\
		}								\
										\
		UCSR ## num ## B |= _BV(RXCIE ## num);				\
		UCSR ## num ## B |= _BV(RXEN ## num);				\
	}

#define SERIAL_METHODS(num)						\
	bool serial ## num ##_rx_is_empty(void)			\
	{								\
		bool ret;						\
									\
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {			\
			_MemoryBarrier();				\
			ret = serial ## num ## _buf_rx_is_empty();	\
			_MemoryBarrier();				\
		}							\
									\
		return ret;						\
	}								\
									\
	uint8_t serial ## num ##_rx_len(void)				\
	{								\
		uint8_t ret;						\
									\
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {			\
			_MemoryBarrier();				\
			ret = serial ## num ## _buf_rx_valid_length;	\
			_MemoryBarrier();				\
		}							\
									\
		return ret;						\
	}								\
									\
	bool serial ## num ##_rx_get(uint8_t *out)			\
	{								\
		bool ret;						\
									\
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {			\
			_MemoryBarrier();				\
									\
			if (!serial ## num ## _buf_rx_is_empty()) {	\
				*out = serial ## num ## _buf_rx_get();	\
				ret = true;				\
			} else						\
				ret = false;				\
									\
			_MemoryBarrier();				\
		}							\
									\
		return ret;						\
	}								\
									\
	void serial ## num ##_rx_peek(uint8_t *out, uint8_t count,	\
				      uint8_t *act_count)		\
	{								\
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {			\
			_MemoryBarrier();				\
									\
			if (count > serial ## num ##			\
			    _buf_rx_valid_length)			\
				count = serial ## num ##		\
					_buf_rx_valid_length;		\
									\
			serial ## num ## _buf_rx_peek(out, count);	\
			*act_count = count;				\
									\
			_MemoryBarrier();				\
		}							\
	}								\
									\
	void serial ## num ##_rx_peek_at(uint8_t *out, uint8_t idx,	\
					 uint8_t count,		\
					 uint8_t *act_count)		\
	{								\
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {			\
			_MemoryBarrier();				\
									\
			if (idx >= serial ## num ##			\
			    _buf_rx_valid_length)			\
				*act_count = 0;			\
			else {						\
				uint16_t end = (uint16_t)idx +		\
					count;				\
				if (end > serial ## num ##		\
				    _buf_rx_valid_length) {		\
					uint8_t adj = end -		\
						serial ## num ##	\
						_buf_rx_valid_length;	\
					count -= adj;			\
				}					\
									\
				serial ## num ## _buf_rx_peek_at(out,	\
								 idx,	\
								 count);\
				*act_count = count;			\
			}						\
									\
			_MemoryBarrier();				\
		}							\
	}								\
									\
	bool serial ## num ##_tx_is_empty(void)			\
	{								\
		bool ret;						\
									\
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {			\
			_MemoryBarrier();				\
			ret = serial ## num ## _buf_tx_is_empty();	\
			_MemoryBarrier();				\
		}							\
									\
		return ret;						\
	}								\
									\
	void serial ## num ##_tx_put(uint8_t in)			\
	{								\
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {			\
			_MemoryBarrier();				\
									\
			serial ## num ## _buf_tx_put(in);		\
			UCSR ## num ## B |= _BV(UDRIE ## num) |	\
				_BV(TXEN ## num);			\
									\
			_MemoryBarrier();				\
		}							\
	}								\
									\
	static void serial ## num ##_debug_notify(void)		\
	{								\
		bool debug_port = num == SERIAL_DEBUG_OUT_PORT;	\
		if (!debug_port)					\
			return;					\
									\
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {			\
			_MemoryBarrier();				\
									\
			if (!debug_buf_is_empty_atomic())		\
				UCSR ## num ## B |= _BV(UDRIE ## num) | \
					_BV(TXEN ## num);		\
									\
			_MemoryBarrier();				\
		}							\
	}

/*
 * expand this macro to implement a serial port with (zero-based) index
 * of num and with given RX, TX buffer sizes (in bytes)
 */
#define SERIAL_IMPL(num, bufsizerx, bufsizetx)		\
	SERIAL_BUF_DEFS(num, rx, bufsizerx)		\
	SERIAL_BUF_DEFS(num, tx, bufsizetx)		\
							\
	SERIAL_BUF_FUNCS(num, rx, bufsizerx)		\
	SERIAL_BUF_FUNCS(num, tx, bufsizetx)		\
							\
	SERIAL_INTERRUPTS(num)				\
							\
	SERIAL_METHODS(num)				\
							\
	SERIAL_SETUP(num)
