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

#include <stddef.h>
#include <stdlib.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/twi.h>

#include "debug.h"
#include "i2c.h"
#include "misc.h"

/* bus clock in Hz */
#ifndef I2C_BUS_CLOCK
#define I2C_BUS_CLOCK ((uint32_t)100 * 1000)
#endif

#define I2C_RESET_POLL_PERIOD 5
#define I2C_TRANS_TIMEOUT 5000

#ifdef I2C_DEBUG_LOG_DISABLE
#undef dprintf
#undef dprintf_P
#define dprintf(...)
#define dprintf_P(...)
#endif

typedef enum { I2C_IDLE, I2C_RESET,
	       I2C_START_DO, I2C_START_TX,
	       I2C_ADDR,
	       I2C_WRITE_FIRST, I2C_WRITE,
	       I2C_REPEATED_START_DO, I2C_REPEATED_START_TX,
	       I2C_READ_FIRST, I2C_READ,
	       I2C_TRANS_OK_STOP_DO, I2C_TRANS_OK_STOP_TX,
	       I2C_TRANS_FAILED_RESET } i2c_states;

typedef struct _i2c_transaction_list {
	struct _i2c_transaction_list *next;

	uint8_t addr;
	const uint8_t *wrbuf;
	uint8_t wrlen;
	uint8_t *rdbuf;
	uint8_t rdlen;

	i2c_completion_fun fun;
	void *data;

	uint8_t rdlen_actual;
} i2c_transaction_list;

static /* i2c_states */ uint8_t i2c_state;
static bool i2c_state_changed;
static timestamp i2c_next_reset_idle_poll;

static timestamp i2c_transaction_deadline;
static i2c_transaction_list *i2c_transaction_list_head;

#define I2C_SETSTATE(state_new)					\
	do								\
		if (i2c_state != state_new) {				\
			dprintf_P(PSTR_M("%S: *%S\n"), PSTR_M("i2c"),	\
				  PSTR_M(#state_new));			\
			i2c_state_changed = true;			\
			i2c_set_state_do(state_new);			\
		}							\
	while (0)

static void i2c_twcr_set_bits_atomic(uint8_t bits)
{
	uint8_t val = TWCR;
	val &= ~_BV(TWINT);
	val |= bits;
	TWCR = val;
}

static void i2c_twcr_clear_bits_atomic(uint8_t bits)
{
	uint8_t val = TWCR;
	val &= ~_BV(TWINT);
	val &= ~bits;
	TWCR = val;
}

static void i2c_twcr_set_cmd_bits_atomic(uint8_t bits)
{
	uint8_t val = TWCR;
	val &= ~(_BV(TWINT) | _BV(TWEA) | _BV(TWSTA) | _BV(TWSTO));
	val |= bits;
	TWCR = val;
}

static void i2c_twcr_set_bits(uint8_t bits)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();
		i2c_twcr_set_bits_atomic(bits);
		_MemoryBarrier();
	}
}

static void i2c_twcr_clear_bits(uint8_t bits)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();
		i2c_twcr_clear_bits_atomic(bits);
		_MemoryBarrier();
	}
}

static void i2c_twcr_set_cmd_bits(uint8_t bits)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();
		i2c_twcr_set_cmd_bits_atomic(bits);
		_MemoryBarrier();
	}
}

static bool i2c_is_reset_idle_poll_state(void)
{
	return i2c_state == I2C_RESET || i2c_state == I2C_TRANS_OK_STOP_TX;
}

static bool i2c_is_transaction_wait_deadline_state(void)
{
	return i2c_state == I2C_START_TX ||
		i2c_state == I2C_REPEATED_START_TX || i2c_state == I2C_ADDR ||
		i2c_state == I2C_WRITE || i2c_state == I2C_READ ||
		i2c_state == I2C_TRANS_OK_STOP_TX;
}

static void i2c_set_state_do(i2c_states state_new)
{
	bool was_reset_idle_poll_state = i2c_is_reset_idle_poll_state();

	i2c_state = state_new;

	if (i2c_state == I2C_TRANS_OK_STOP_DO ||
	    i2c_state == I2C_TRANS_FAILED_RESET) {
		i2c_transaction_list *tr = i2c_transaction_list_head;
		i2c_transaction_list *tr_next = tr->next;

		if (tr->fun != NULL)
			tr->fun(tr->data, i2c_state == I2C_TRANS_OK_STOP_DO,
				tr->rdlen_actual);

		free(tr);
		i2c_transaction_list_head = tr_next;
	} else if (i2c_state == I2C_START_DO) {
		const timestamp_interval transaction_timeout =
			TIMESTAMPI_FROM_MS(I2C_TRANS_TIMEOUT);

		timestamp now;
		timekeeping_now_timestamp(&now);
		timestamp_add(&now, &transaction_timeout,
			      &i2c_transaction_deadline);
	}

	if (!was_reset_idle_poll_state && i2c_is_reset_idle_poll_state())
		timekeeping_now_timestamp(&i2c_next_reset_idle_poll);
}

static void i2c_reset(void)
{
	i2c_twcr_clear_bits(_BV(TWEA) | _BV(TWSTA) | _BV(TWSTO) | _BV(TWEN) |
			    _BV(TWIE));
	i2c_twcr_set_bits(_BV(TWINT) | _BV(TWEN));

	I2C_SETSTATE(I2C_RESET);
}

ISR(TWI_vect)
{
	i2c_twcr_clear_bits_atomic(_BV(TWIE));
}

bool i2c_transaction(uint8_t addr,
		     const uint8_t *wrbuf, uint8_t wrlen,
		     uint8_t *rdbuf, uint8_t rdlen,
		     i2c_completion_fun completion, void *comp_data)
{
	if (wrlen == 0 && rdlen == 0)
		return false;

	if (wrlen > 0 && wrbuf == NULL)
		return false;

	if (rdlen > 0 && rdbuf == NULL)
		return false;

	i2c_transaction_list *nelem;

	nelem = malloc(sizeof(i2c_transaction_list));
	if (nelem == NULL)
		return false;

	nelem->next = NULL;

	nelem->addr = addr;
	nelem->wrbuf = wrbuf;
	nelem->wrlen = wrlen;
	nelem->rdbuf = rdbuf;
	nelem->rdlen = rdlen;
	nelem->fun = completion;
	nelem->data = comp_data;

	nelem->rdlen_actual = 0;

	if (i2c_transaction_list_head == NULL)
		i2c_transaction_list_head = nelem;
	else {
		i2c_transaction_list *elem = i2c_transaction_list_head;
		while (elem->next != NULL)
			elem = elem->next;

		elem->next = nelem;
	}

	if (i2c_state == I2C_IDLE)
		I2C_SETSTATE(I2C_START_DO);

	return true;
}

/*
 * the I2C hardware can get stuck for example when there is a lot of noise
 * on SDA / SCL lines
 *
 * it seems the only way to unstuck it is to disable and then reenable
 * (TWEN bit) the whole module - this action resets it back into a normal
 * operation
 */
static bool i2c_transaction_maybe_timedout(void)
{
	timestamp now;
	timekeeping_now_timestamp(&now);

	if (timestamp_temporal_cmp(&now, &i2c_transaction_deadline, <))
		return false;

	dprintf_P(PSTR_M("i2c: transaction timed out (STATUS %x, CR %x)\n"),
		  (unsigned)TW_STATUS, (unsigned)TWCR);

	if (i2c_state != I2C_TRANS_OK_STOP_TX)
		I2C_SETSTATE(I2C_TRANS_FAILED_RESET);
	else
		i2c_reset();

	return true;
}

void i2c_poll_atomic(void)
{
	const timestamp_interval reset_idle_poll_period =
		TIMESTAMPI_FROM_MS(I2C_RESET_POLL_PERIOD);

	i2c_state_changed = false;

	if (i2c_is_reset_idle_poll_state()) {
		if (bit_is_set(TWCR, TWSTO)) {
			timestamp now;
			timekeeping_now_timestamp(&now);
			timestamp_add(&now, &reset_idle_poll_period,
				      &i2c_next_reset_idle_poll);

			if (i2c_state == I2C_TRANS_OK_STOP_TX)
				i2c_transaction_maybe_timedout();

			return;
		}

		if (i2c_transaction_list_head != NULL)
			I2C_SETSTATE(I2C_START_DO);
		else
			I2C_SETSTATE(I2C_IDLE);
	} else if (i2c_state == I2C_START_DO ||
		   i2c_state == I2C_REPEATED_START_DO) {
		i2c_twcr_set_cmd_bits(_BV(TWINT) | _BV(TWSTA) | _BV(TWIE));

		if (i2c_state == I2C_START_DO)
			I2C_SETSTATE(I2C_START_TX);
		else /* I2C_REPEATED_START_DO */
			I2C_SETSTATE(I2C_REPEATED_START_TX);
	} else if (i2c_state == I2C_START_TX ||
		   i2c_state == I2C_REPEATED_START_TX) {
		if (bit_is_clear(TWCR, TWINT)) {
			i2c_transaction_maybe_timedout();

			return;
		}

		uint8_t status = TW_STATUS;
		if (status != TW_START && status != TW_REP_START) {
			dprintf_P(PSTR_M("i2c: STATUS %x\n"),
				  (unsigned)status);
			I2C_SETSTATE(I2C_TRANS_FAILED_RESET);
			return;
		}

		uint8_t data = i2c_transaction_list_head->addr << 1;
		if (i2c_transaction_list_head->wrlen == 0)
			data |= TW_READ;
		else
			data |= TW_WRITE;

		TWDR = data;

		i2c_twcr_set_cmd_bits(_BV(TWINT) | _BV(TWIE));

		I2C_SETSTATE(I2C_ADDR);
	} else if (i2c_state == I2C_ADDR) {
		if (bit_is_clear(TWCR, TWINT)) {
			i2c_transaction_maybe_timedout();

			return;
		}

		uint8_t status = TW_STATUS;
		if ((status != TW_MT_SLA_ACK ||
		     i2c_transaction_list_head->wrlen == 0) &&
		    status != TW_MR_SLA_ACK) {
			dprintf_P(PSTR_M("i2c: STATUS %x\n"),
				  (unsigned)status);
			I2C_SETSTATE(I2C_TRANS_FAILED_RESET);
			return;
		}

		if (status == TW_MT_SLA_ACK)
			I2C_SETSTATE(I2C_WRITE_FIRST);
		else
			I2C_SETSTATE(I2C_READ_FIRST);
	} else if (i2c_state == I2C_WRITE_FIRST ||
		   i2c_state == I2C_WRITE) {
		if (i2c_state == I2C_WRITE) {
			if (bit_is_clear(TWCR, TWINT)) {
				i2c_transaction_maybe_timedout();

				return;
			}

			uint8_t status = TW_STATUS;
			if (status != TW_MT_DATA_ACK &&
			    (status != TW_MT_DATA_NACK ||
			     i2c_transaction_list_head->wrlen != 0)) {
				dprintf_P(PSTR_M("i2c: STATUS %x\n"),
					  (unsigned)status);
				I2C_SETSTATE(I2C_TRANS_FAILED_RESET);
				return;
			}
		}

		if (i2c_transaction_list_head->wrlen == 0) {
			if (i2c_transaction_list_head->rdlen > 0)
				I2C_SETSTATE(I2C_REPEATED_START_DO);
			else
				I2C_SETSTATE(I2C_TRANS_OK_STOP_DO);
		} else {
			TWDR = *i2c_transaction_list_head->wrbuf;
			i2c_transaction_list_head->wrbuf++;
			i2c_transaction_list_head->wrlen--;

			i2c_twcr_set_cmd_bits(_BV(TWINT) | _BV(TWIE));

			I2C_SETSTATE(I2C_WRITE);
		}
	} else if (i2c_state == I2C_READ_FIRST ||
		   i2c_state == I2C_READ) {
		if (i2c_state == I2C_READ) {
			if (bit_is_clear(TWCR, TWINT)) {
				i2c_transaction_maybe_timedout();

				return;
			}

			uint8_t status = TW_STATUS;
			if (status != TW_MR_DATA_ACK &&
			    status != TW_MR_DATA_NACK) {
				dprintf_P(PSTR_M("i2c: STATUS %x\n"),
					  (unsigned)status);
				I2C_SETSTATE(I2C_TRANS_FAILED_RESET);
				return;
			}

			*i2c_transaction_list_head->rdbuf = TWDR;
			i2c_transaction_list_head->rdbuf++;
			i2c_transaction_list_head->rdlen--;
			i2c_transaction_list_head->rdlen_actual++;
		}

		if (i2c_transaction_list_head->rdlen > 0) {
			uint8_t ackbit =
				i2c_transaction_list_head->rdlen > 1 ?
				_BV(TWEA) : 0;

			i2c_twcr_set_cmd_bits(_BV(TWINT) | ackbit | _BV(TWIE));

			I2C_SETSTATE(I2C_READ);
		} else
			I2C_SETSTATE(I2C_TRANS_OK_STOP_DO);
	} else if (i2c_state == I2C_TRANS_OK_STOP_DO) {
		i2c_twcr_set_cmd_bits(_BV(TWINT) | _BV(TWSTO));

		I2C_SETSTATE(I2C_TRANS_OK_STOP_TX);
	} else if (i2c_state == I2C_TRANS_FAILED_RESET)
		i2c_reset();
}

void i2c_get_next_poll_time(timestamp *next_poll)
{
	if (i2c_state_changed)
		timekeeping_now_timestamp(next_poll);
	else if (i2c_is_reset_idle_poll_state())
		*next_poll = i2c_next_reset_idle_poll;
	/* assume that reset poll period is much shorter than tx deadline */
	else if (i2c_is_transaction_wait_deadline_state())
		*next_poll = i2c_transaction_deadline;
	else
		timekeeping_timestamp_max_future(next_poll);
}

static uint32_t i2c_speed_settings_2_clock(uint8_t twbr, uint8_t prescaler)
{
	return F_CPU / ((uint32_t)2 * twbr * prescaler + 16);
}

static uint8_t i2c_clock_get_twbr(uint32_t clock, uint8_t prescaler)
{
	uint8_t ret = (F_CPU - 16 * clock) / (2 * clock * prescaler);
	if ((F_CPU - 16 * clock) % (2 * clock * prescaler) != 0)
		ret++;

	return ret;
}

#define I2C_GET_SPEED_ERROR(clock, prescaler)			      \
	({							      \
		uint32_t tmp_i2c_get_speed_error_actual_clock =       \
			i2c_speed_settings_2_clock(		      \
				i2c_clock_get_twbr(clock, prescaler), \
				prescaler);			      \
		uint32_t tmp_i2c_get_speed_error_diff;		      \
								      \
		if (tmp_i2c_get_speed_error_actual_clock <= clock)    \
			tmp_i2c_get_speed_error_diff = clock -	      \
				tmp_i2c_get_speed_error_actual_clock; \
		else						      \
			tmp_i2c_get_speed_error_diff =		      \
				tmp_i2c_get_speed_error_actual_clock -\
				clock;				      \
								      \
		tmp_i2c_get_speed_error_diff;			      \
	})

static void i2c_get_speed_settings(uint32_t clock, uint8_t *twbr,
				   uint8_t *prescaler)
{
	uint32_t error_1 = I2C_GET_SPEED_ERROR(clock, 1);
	uint32_t error_4 = I2C_GET_SPEED_ERROR(clock, 4);
	uint32_t error_16 = I2C_GET_SPEED_ERROR(clock, 16);
	uint32_t error_64 = I2C_GET_SPEED_ERROR(clock, 64);

	uint32_t error_cur = error_1;
	*prescaler = 1;

	if (error_4 < error_cur) {
		*prescaler = 4;
		error_cur = error_4;
	}

	if (error_16 < error_cur) {
		*prescaler = 16;
		error_cur = error_16;
	}

	if (error_64 < error_cur) {
		*prescaler = 64;
		error_cur = error_64;
	}

	*twbr = i2c_clock_get_twbr(clock, *prescaler);
}

void i2c_setup(void)
{
	power_twi_enable();

	i2c_twcr_clear_bits_atomic(_BV(TWEA) | _BV(TWSTA) | _BV(TWSTO) |
				   _BV(TWEN) | _BV(TWIE));

	do {
		uint8_t twbr;
		uint8_t prescaler;
		i2c_get_speed_settings(I2C_BUS_CLOCK, &twbr, &prescaler);

		TWBR = twbr;

		/* prescaler = 1 */
		TWSR &= ~(_BV(TWPS0) | _BV(TWPS1));
		if (prescaler == 4)
			TWSR |= _BV(TWPS0);
		else if (prescaler == 16)
			TWSR |= _BV(TWPS1);
		else if (prescaler == 64)
			TWSR |= _BV(TWPS0) | _BV(TWPS1);
	} while (0);

	wdt_reset();
	i2c_twcr_set_bits_atomic(_BV(TWEN));

	i2c_state = I2C_IDLE;
	i2c_state_changed = false;
}
