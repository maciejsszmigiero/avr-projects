/*
 * Smart UPS Addon: temperature controller
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <stddef.h>

#include "../lib/debug.h"
#include "../lib/misc.h"
#include "../lib/tc74.h"
#include "fan.h"
#include "temp.h"

/* temperature limits */
#define TEMP_FAN_DISABLED_TO_LOW 42
#define TEMP_FAN_LOW_TO_HIGH 46
#define TEMP_FAN_HIGH_TO_LOW TEMP_FAN_DISABLED_TO_LOW
#define TEMP_FAN_LOW_TO_DISABLED 38
#define TEMP_CRITICAL 75

/* how often (in ms) sensors should be updated? */
#ifndef ENABLE_DEBUG_LOG
#define TEMP_POLL_PERIOD 600
#else
/* don't flood the log */
#define TEMP_POLL_PERIOD 2000
#endif

/*
 * how many times in a row a sensor needs to fail an update attempt before its
 * temperature is considered stale?
 */
#define TEMP_FAILED_UPDATES_FOR_STALE_DATA 3

/* sensor definitions: count */
#define TEMP_NUM_SENSORS 3

/* sensor definitions: i2c addresses */
#define TEMP_IDX2ADDR(idx)			\
	(idx == 0 ? 0x48 : idx == 1 ? 0x4b : 0x4f)

/* sensor definitions: temperature offsets for limits (excluding Tcritical) */
#define TEMP_IDX2TOFFSET(idx)			\
	(idx == 2 ? -20 : 0)

#ifdef TEMP_DEBUG_LOG_DISABLE
#undef dprintf
#undef dprintf_P
#define dprintf(...)
#define dprintf_P(...)
#endif

typedef enum { TEMP_IDLE,
	       TEMP_GET_INIT, TEMP_GET, TEMP_GET_OK, TEMP_GET_FAILED,
	       TEMP_GET_NEXT,
	       TEMP_UPDATE_FANS } temp_states;

typedef enum { FAN_DISABLED, FAN_LOW, FAN_HIGH } fan_states;

typedef struct _tc74_temp {
	int8_t cur;
	int8_t min;
	int8_t max;
} tc74_temp;

static /* temp_states */ uint8_t temp_state;
static bool temp_state_changed;

static /* fan_states */ uint8_t fan_state;

static timestamp temp_next_poll;

static tc74_data tc74[TEMP_NUM_SENSORS];
static tc74_temp tc74_temps[TEMP_NUM_SENSORS];
static uint8_t tc74_failed_updates[TEMP_NUM_SENSORS];
static uint8_t tc74_cur_idx;
static uint8_t tc74_debug_ctr;

static bool temp_enable_debug_data(void)
{
	return
#ifdef TEMP_ENABLE_DEBUG_DATA
		true
#else
		false
#endif
		;
}

static bool temp_only_critical_limit(void)
{
	return
#ifdef TEMP_ONLY_CRITICAL_LIMIT
		true
#else
		false
#endif
		;
}

#define TEMP_STALE(idx)		       \
	(tc74_failed_updates[idx] >=	       \
	 TEMP_FAILED_UPDATES_FOR_STALE_DATA)

#define TEMP_FAILED_INC(idx)				\
	do						\
		if (!TEMP_STALE(idx))			\
			tc74_failed_updates[idx]++;	\
	while (0)

#define TEMP_SETSTATE(state_new)					\
	do								\
		if (temp_state != state_new) {				\
			dprintf_P(PSTR_M("%S: *%S\n"), PSTR_M("temp"),	\
				  PSTR_M(#state_new));			\
			temp_state_changed = true;			\
			temp_set_state_do(state_new);			\
		}							\
	while (0)

static void temp_set_state_do(temp_states state_new)
{
	temp_state = state_new;

	if (temp_state == TEMP_GET_INIT)
		tc74_cur_idx = 0;
	else if (temp_state == TEMP_GET_NEXT)
		tc74_cur_idx++;
	else if (temp_state == TEMP_GET_OK)
		tc74_failed_updates[tc74_cur_idx] = 0;
	else if (temp_state == TEMP_GET_FAILED)
		TEMP_FAILED_INC(tc74_cur_idx);
	else if (temp_state == TEMP_UPDATE_FANS) {
		const timestamp_interval poll_period =
			TIMESTAMPI_FROM_MS(TEMP_POLL_PERIOD);

		timestamp now;
		timekeeping_now_timestamp(&now);
		timestamp_add(&now, &poll_period, &temp_next_poll);
	}
}

#define TEMP_POLL_UPDATE_FAN_TEMP(idx)					\
	do {								\
		if (TEMP_STALE(idx))					\
			break;						\
									\
		int8_t temp_poll_update_fan_temp_cm =			\
			tc74_temps[idx].cur >= TEMP_CRITICAL ?		\
			0 : TEMP_CRITICAL - tc74_temps[idx].cur;	\
									\
		if (temp_poll_update_fan_temp_cm < temp_critical_margin)\
			temp_critical_margin =				\
				temp_poll_update_fan_temp_cm;		\
									\
		int8_t temp_poll_update_fan_temp_t =			\
			tc74_temps[idx].cur + TEMP_IDX2TOFFSET(idx);	\
									\
		if (temp_set && temp >= temp_poll_update_fan_temp_t)	\
			break;						\
									\
		temp = temp_poll_update_fan_temp_t;			\
		temp_set = true;					\
	} while (0)

void temp_poll(void)
{
	temp_state_changed = false;

	fan_poll();
	for (uint8_t ctr = 0; ctr < TEMP_NUM_SENSORS; ctr++)
		tc74_poll(&tc74[ctr]);

	if (temp_state == TEMP_IDLE) {
		timestamp now;
		timekeeping_now_timestamp(&now);
		if (timestamp_temporal_cmp(&now, &temp_next_poll,
					   <))
			return;

		TEMP_SETSTATE(TEMP_GET_INIT);
	} else if (temp_state == TEMP_GET_INIT ||
		   temp_state == TEMP_GET_NEXT) {
		if (tc74_cur_idx >= TEMP_NUM_SENSORS) {
			TEMP_SETSTATE(TEMP_UPDATE_FANS);
			return;
		}

		if (!tc74_get_temperature(&tc74[tc74_cur_idx])) {
			TEMP_SETSTATE(TEMP_GET_FAILED);
			return;
		}

		TEMP_SETSTATE(TEMP_GET);
	} else if (temp_state == TEMP_GET) {
		if (tc74_is_busy(&tc74[tc74_cur_idx]))
			return;

		int8_t temp;
		if (!tc74_get_temperature_result(&tc74[tc74_cur_idx], &temp)) {
			TEMP_SETSTATE(TEMP_GET_FAILED);
			return;
		}

		if (temp_enable_debug_data()) {
			if (tc74_cur_idx == 0)
				tc74_debug_ctr++;

			if (tc74_debug_ctr < 5) {
				if (tc74_cur_idx == 0 &&
				    temp < TEMP_FAN_LOW_TO_HIGH) {
					temp = TEMP_FAN_LOW_TO_HIGH;
					dprintf_P(PSTR_M("temp: faking %S temp at %d\n"),
						  PSTR_M("high"), 0);
				}
			} else if (tc74_debug_ctr < 10) {
				if (tc74_cur_idx == 1 &&
				    temp < TEMP_FAN_LOW_TO_HIGH) {
					temp = TEMP_FAN_LOW_TO_HIGH;
					dprintf_P(PSTR_M("temp: faking %S temp at %d\n"),
						  PSTR_M("high"), 1);
				}
			} else if (tc74_debug_ctr < 20) {
				if (tc74_cur_idx == 1 &&
				    temp < TEMP_FAN_DISABLED_TO_LOW) {
					temp = TEMP_FAN_DISABLED_TO_LOW;
					dprintf_P(PSTR_M("temp: faking %S temp at %d\n"),
						  PSTR_M("low"), 1);
				}
			} else if (tc74_debug_ctr < 30) {
				if (tc74_cur_idx == 0 &&
				    temp < TEMP_FAN_DISABLED_TO_LOW) {
					temp = TEMP_FAN_DISABLED_TO_LOW;
					dprintf_P(PSTR_M("temp: faking %S temp at %d\n"),
						  PSTR_M("low"), 0);
				} else if (tc74_cur_idx == 1 &&
					   temp < TEMP_FAN_LOW_TO_HIGH) {
					temp = TEMP_FAN_LOW_TO_HIGH;
					dprintf_P(PSTR_M("temp: faking %S temp at %d\n"),
						  PSTR_M("high"), 1);
				}
			} else
				tc74_debug_ctr = 0;
		}

		tc74_temps[tc74_cur_idx].cur = temp;
		if (temp < tc74_temps[tc74_cur_idx].min)
			tc74_temps[tc74_cur_idx].min = temp;
		if (temp > tc74_temps[tc74_cur_idx].max)
			tc74_temps[tc74_cur_idx].max = temp;

		TEMP_SETSTATE(TEMP_GET_OK);
	} else if (temp_state == TEMP_GET_OK ||
		   temp_state == TEMP_GET_FAILED)
		TEMP_SETSTATE(TEMP_GET_NEXT);
	else if (temp_state == TEMP_UPDATE_FANS) {
		bool temp_set = false;
		int8_t temp_critical_margin = INT8_MAX;
		int8_t temp;
		typeof(fan_state) fan_state_old = fan_state;

		for (uint8_t ctr = 0; ctr < TEMP_NUM_SENSORS; ctr++)
			TEMP_POLL_UPDATE_FAN_TEMP(ctr);

		if (temp_set) {
			if (!temp_only_critical_limit()) {
				if (fan_state == FAN_HIGH) {
					if (temp <= TEMP_FAN_HIGH_TO_LOW)
						fan_state = FAN_LOW;
				}
				if (fan_state == FAN_LOW) {
					if (temp <= TEMP_FAN_LOW_TO_DISABLED)
						fan_state = FAN_DISABLED;
				}
				if (fan_state == FAN_DISABLED) {
					if (temp >= TEMP_FAN_DISABLED_TO_LOW)
						fan_state = FAN_LOW;
				}
				if (fan_state == FAN_LOW) {
					if (temp >= TEMP_FAN_LOW_TO_HIGH)
						fan_state = FAN_HIGH;
				}
			} else if (temp_critical_margin >= 10)
				fan_state = FAN_DISABLED;

			bool any_stale = false;
			for (uint8_t ctr = 0; ctr < TEMP_NUM_SENSORS; ctr++)
				if (TEMP_STALE(ctr)) {
					any_stale = true;
					break;
				}

			if (any_stale && fan_state == FAN_DISABLED)
				fan_state = FAN_LOW;

			if (temp_critical_margin <= 0)
				fan_state = FAN_HIGH;
		} else
			fan_state = FAN_HIGH;

		if (fan_state != fan_state_old) {
			if (fan_state == FAN_HIGH) {
				dprintf_P(PSTR_M("temp: want %S fan\n"),
					  PSTR_M("high"));

				fan_enable_high();
			} else if (fan_state == FAN_LOW) {
				dprintf_P(PSTR_M("temp: want %S fan\n"),
					  PSTR_M("low"));

				fan_enable_low();
			} else { /* FAN_DISABLED */
				dprintf_P(PSTR_M("temp: want %S fan\n"),
					  PSTR_M("no"));

				fan_disable();
			}
		}

		TEMP_SETSTATE(TEMP_IDLE);
	}
}

#define TEMP_GET_TIMEOUT(funname, ...)					\
	do {								\
		timestamp module_next_poll_time;			\
		funname(__VA_ARGS__ &module_next_poll_time);		\
		if (!next_poll_time_set ||				\
		    timestamp_temporal_cmp(&module_next_poll_time,	\
					   next_poll,			\
					   <)) {			\
			*next_poll = module_next_poll_time;		\
			next_poll_time_set = true;			\
		}							\
	} while (0)

#define TEMP_ADD_TIMEOUT(tvar)						\
	do {								\
		if (!next_poll_time_set ||				\
		    timestamp_temporal_cmp(&tvar,			\
					   next_poll,			\
					   <)) {			\
			*next_poll = tvar;				\
			next_poll_time_set = true;			\
		}							\
	} while (0)

void temp_get_next_poll_time(timestamp *next_poll)
{
	if (temp_state_changed || (temp_state == TEMP_GET &&
				   !tc74_is_busy(&tc74[tc74_cur_idx])))
		timekeeping_now_timestamp(next_poll);
	else {
		bool next_poll_time_set = false;

		TEMP_GET_TIMEOUT(fan_get_next_poll_time);
		for (uint8_t ctr = 0; ctr < TEMP_NUM_SENSORS; ctr++)
			TEMP_GET_TIMEOUT(tc74_get_next_poll_time, &tc74[ctr],);

		if (temp_state == TEMP_IDLE)
			TEMP_ADD_TIMEOUT(temp_next_poll);

		if (!next_poll_time_set)
			timekeeping_timestamp_max_future(next_poll);
	}
}

uint8_t temp_get_count(void)
{
	return TEMP_NUM_SENSORS;
}

bool temp_get(uint8_t idx, int8_t *cur, int8_t *min, int8_t *max)
{
	if (idx >= TEMP_NUM_SENSORS)
		return false;

	if (TEMP_STALE(idx))
		return false;

	if (cur != NULL)
		*cur = tc74_temps[idx].cur;

	if (min != NULL)
		*min = tc74_temps[idx].min;

	if (max != NULL)
		*max = tc74_temps[idx].max;

	return true;
}

bool temp_reset_minmax(uint8_t idx)
{
	if (idx >= TEMP_NUM_SENSORS)
		return false;

	if (TEMP_STALE(idx)) {
		tc74_temps[idx].min = INT8_MAX;
		tc74_temps[idx].max = INT8_MIN;
	} else
		tc74_temps[idx].min = tc74_temps[idx].max = tc74_temps[idx].cur;

	return true;
}

void temp_setup(void)
{
	for (uint8_t ctr = 0; ctr < TEMP_NUM_SENSORS; ctr++) {
		tc74_init(&tc74[ctr], TEMP_IDX2ADDR(ctr));
		tc74_failed_updates[ctr] = TEMP_FAILED_UPDATES_FOR_STALE_DATA;
		tc74_temps[ctr].min = INT8_MAX;
		tc74_temps[ctr].max = INT8_MIN;
	}

	fan_setup();

	timekeeping_now_timestamp(&temp_next_poll);

	temp_state = TEMP_IDLE;
	temp_state_changed = false;

	fan_state = FAN_HIGH;
	fan_enable_high();
}
