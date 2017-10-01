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

#include <inttypes.h>
#include <stddef.h>
#include <util/atomic.h>

#include "debug.h"
#include "i2c.h"
#include "misc.h"
#include "tc74.h"

#define TC74_DATA_READY_POLL_PERIOD (250 / 2)
#define TC74_DATA_READY_POLL_COUNT 3

#ifdef TC74_DEBUG_LOG_DISABLE
#undef dprintf
#undef dprintf_P
#define dprintf(...)
#define dprintf_P(...)
#endif

#define TC74_REG_TEMP 0
#define TC74_REG_CONFIG 1
#define TC74_REG_CONFIG_STANDBY ((uint8_t)_BV(7))
#define TC74_REG_CONFIG_DATA_READY ((uint8_t)_BV(6))
#define TC74_REG_CONFIG_ZERO_MASK ((uint8_t)(_BV(5) | _BV(4) | _BV(3) | \
					     _BV(2) | _BV(1) | _BV(0)))

static uint8_t tc74_temp_read_wr[] = { TC74_REG_TEMP };
static uint8_t tc74_config_read_wr[] = { TC74_REG_CONFIG };
static uint8_t tc74_config_write_wr[] = { TC74_REG_CONFIG, 0 };

#define TC74_SETSTATE(data, state_new)					\
	do								\
		if (data->state != state_new) {			\
			dprintf_P(PSTR_M("%S: *%S\n"), PSTR_M("tc74"),	\
				  PSTR_M(#state_new));			\
			data->state_changed = true;			\
			tc74_set_state_do(data, state_new);		\
		}							\
	while (0)

static bool tc74_is_i2c_transfer_state(tc74_data *data)
{
	return data->state == TC74_CONFIG_READ ||
		data->state == TC74_CONFIG_WRITE ||
		data->state == TC74_CONFIG_WRITE_CONFIG_READ ||
		data->state == TC74_DATA_READY_CONFIG_READ ||
		data->state == TC74_TEMP_READ;
}

static void tc74_set_state_do(tc74_data *data, tc74_states state_new)
{
	data->state = state_new;

	if (data->state == TC74_CONFIG_READ_DO)
		data->get_temp_result = false;
	else if (data->state == TC74_DATA_READY_WAIT_INIT) {
		const timestamp_interval poll_period =
			TIMESTAMPI_FROM_MS(TC74_DATA_READY_POLL_PERIOD);

		timestamp now;

		timekeeping_now_timestamp(&now);
		timestamp_add(&now, &poll_period, &data->next_data_ready_poll);

		/* already did check once before we arrived in this state */
		data->data_ready_polls = 1;
	} else if (data->state == TC74_TEMP_READ_OK) {
		data->get_temp_result = true;

		dprintf_P(PSTR("tc74: temperature %"PRId8" dC\n"),
			  data->temp);
	}
}

static void tc74_i2c_complete(void *data_v, bool success, uint8_t rdlen_actual)
{
	tc74_data *data = data_v;

	data->i2c_trans_complete = true;
	data->i2c_trans_success = success;
	data->i2c_rdlen_actual = rdlen_actual;
}

static bool tc74_i2c_transaction(tc74_data *data,
				 const uint8_t *wrbuf, uint8_t wrlen,
				 uint8_t *rdbuf, uint8_t rdlen)
{
	data->i2c_trans_complete = false;

	return i2c_transaction(data->addr,
			       wrbuf, wrlen,
			       rdbuf, rdlen,
			       tc74_i2c_complete, data);
}

bool tc74_get_temperature(tc74_data *data)
{
	if (tc74_is_busy(data))
		return false;

	TC74_SETSTATE(data, TC74_CONFIG_READ_DO);

	return true;
}

bool tc74_get_temperature_result(tc74_data *data, int8_t *temperature)
{
	if (!data->get_temp_result)
		return false;

	if (temperature != NULL)
		*temperature = data->temp;

	return true;
}

void tc74_poll(tc74_data *data)
{
	data->state_changed = false;

	if (data->state == TC74_CONFIG_READ_DO ||
	    data->state == TC74_DATA_READY_CONFIG_READ_DO ||
	    data->state == TC74_CONFIG_WRITE_CONFIG_READ_DO) {
		if (!tc74_i2c_transaction(data,
					  tc74_config_read_wr,
					  sizeof(tc74_config_read_wr),
					  &data->config, 1))
			goto idle;

		if (data->state == TC74_CONFIG_READ_DO)
			TC74_SETSTATE(data, TC74_CONFIG_READ);
		else if (data->state == TC74_DATA_READY_CONFIG_READ_DO)
			TC74_SETSTATE(data, TC74_DATA_READY_CONFIG_READ);
		else /* TC74_CONFIG_WRITE_CONFIG_READ_DO */
			TC74_SETSTATE(data, TC74_CONFIG_WRITE_CONFIG_READ);
	} else if (data->state == TC74_CONFIG_READ ||
		   data->state == TC74_DATA_READY_CONFIG_READ ||
		   data->state == TC74_CONFIG_WRITE_CONFIG_READ) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			_MemoryBarrier();

			if (!data->i2c_trans_complete)
				return;

			_MemoryBarrier();
		}

		if (!data->i2c_trans_success || data->i2c_rdlen_actual != 1)
			goto idle;

		if ((data->config & TC74_REG_CONFIG_ZERO_MASK) != 0) {
			dprintf_P(PSTR("tc74: reserved bits set (%"PRIx8") in CONFIG\n"),
				  data->config);

			goto idle;
		}

		if ((data->config & TC74_REG_CONFIG_STANDBY) != 0) {
			dprintf_P(PSTR("tc74: STANDBY bit set (%"PRIx8") in CONFIG\n"),
				  data->config);

			if (data->state != TC74_CONFIG_READ)
				goto idle;

			if (!tc74_i2c_transaction(data,
						  tc74_config_write_wr,
						  sizeof(tc74_config_write_wr),
						  NULL, 0))
				goto idle;

			TC74_SETSTATE(data, TC74_CONFIG_WRITE);
			return;
		}

		if (!(data->config & TC74_REG_CONFIG_DATA_READY)) {
			if (data->state == TC74_DATA_READY_CONFIG_READ) {
				if (++data->data_ready_polls >=
				    TC74_DATA_READY_POLL_COUNT)
					goto idle;

				TC74_SETSTATE(data, TC74_DATA_READY_WAIT);
			} else
				TC74_SETSTATE(data, TC74_DATA_READY_WAIT_INIT);

			return;
		}

		if (!tc74_i2c_transaction(data,
					  tc74_temp_read_wr,
					  sizeof(tc74_temp_read_wr),
					  (uint8_t *)&data->temp, 1))
			goto idle;

		TC74_SETSTATE(data, TC74_TEMP_READ);
	} else if (data->state == TC74_DATA_READY_WAIT_INIT)
		TC74_SETSTATE(data, TC74_DATA_READY_WAIT);
	else if (data->state == TC74_DATA_READY_WAIT) {
		const timestamp_interval poll_period =
			TIMESTAMPI_FROM_MS(TC74_DATA_READY_POLL_PERIOD);

		timestamp now;

		timekeeping_now_timestamp(&now);
		if (timestamp_temporal_cmp(&now, &data->next_data_ready_poll,
					   <))
			return;

		timestamp_add(&now, &poll_period, &data->next_data_ready_poll);

		TC74_SETSTATE(data, TC74_DATA_READY_CONFIG_READ_DO);
	} else if (data->state == TC74_CONFIG_WRITE) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			_MemoryBarrier();

			if (!data->i2c_trans_complete)
				return;

			_MemoryBarrier();
		}

		if (!data->i2c_trans_success)
			goto idle;

		TC74_SETSTATE(data, TC74_CONFIG_WRITE_CONFIG_READ_DO);
	} else if (data->state == TC74_TEMP_READ) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			_MemoryBarrier();

			if (!data->i2c_trans_complete)
				return;

			_MemoryBarrier();
		}

		if (!data->i2c_trans_success || data->i2c_rdlen_actual != 1)
			goto idle;

		TC74_SETSTATE(data, TC74_TEMP_READ_OK);
	} else if (data->state == TC74_TEMP_READ_OK)
		goto idle;

	return;

idle:
	TC74_SETSTATE(data, TC74_IDLE);
}

void tc74_get_next_poll_time(tc74_data *data, timestamp *next_poll)
{
	if (data->state_changed || (tc74_is_i2c_transfer_state(data) &&
				    data->i2c_trans_complete))
		timekeeping_now_timestamp(next_poll);
	else if (data->state == TC74_DATA_READY_WAIT)
		*next_poll = data->next_data_ready_poll;
	else
		timekeeping_timestamp_max_future(next_poll);
}

void tc74_init(tc74_data *data, uint8_t addr)
{
	data->addr = addr;

	data->state = TC74_IDLE;
	data->state_changed = false;
}
