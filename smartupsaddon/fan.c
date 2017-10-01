/*
 * Smart UPS Addon: fan controller
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
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "../lib/debug.h"
#include "../lib/misc.h"
#include "fan.h"

/* minimum and maximum measurable RPM */
#define FAN_RPM_MIN 100
#define FAN_RPM_MAX 3600
/*
 * absolute maximum RPM possible for the fan, measurements containing samples
 * with higher RPM will be rejected as invalid
 */
#define FAN_RPM_MAX_ABSOLUTE 6000

/* minimum RPM at low setting */
#define FAN_RPM_LOW_MIN 1000
/* minimum RPM at high setting */
#define FAN_RPM_HIGH_MIN 2000

/* how long (in ms) it takes for fan to spin up from zero RPM to high RPM */
#define FAN_SPINUP_MAX_TIME 5000

/* number of fan pulses (rising or falling edges) per a rotation */
#define FAN_PULSES_PER_ROT 4

/* count of timestamps (samples) of last fan pulses to keep */
#define FAN_TIMESTAMPS 8

/*
 * time period (in ms) between recalculations of fan RPM using
 * samples that were gathered
 */
#define FAN_POLL_PERIOD 750

#ifdef FAN_DEBUG_LOG_DISABLE
#undef dprintf
#undef dprintf_P
#define dprintf(...)
#define dprintf_P(...)
#endif

typedef enum { FAN_INIT, FAN_DISABLED, FAN_FAIL,
	       FAN_LOW_START, FAN_LOW_RUN,
	       FAN_HIGH_START, FAN_HIGH_RUN } fan_states;

typedef enum { FAN_OFF, FAN_LOW, FAN_HIGH } fan_target_states;

static /* fan_states */ uint8_t fan_state;
static bool fan_state_changed;
static /* fan_target_states */ uint8_t fan_target_state;

static timestamp fan_next_rpm_check;
static timestamp fan_spinup_deadline;

static timestamp fan_timestamps[FAN_TIMESTAMPS];
static bool fan_timestamps_dirty;
static uint8_t fan_timestamp_last_element;

static bool fan_debug_log_timediffs(void)
{
	return
#ifdef FAN_DEBUG_LOG_TIMEDIFFS
		true
#else
		false
#endif
		;
}

static bool fan_output_always_off(void)
{
	return
#ifdef FAN_OUTPUT_ALWAYS_OFF
		true
#else
		false
#endif
		;
}

#define FAN_SETSTATE(state_new)					\
	do								\
		if (fan_state != state_new) {				\
			dprintf_P(PSTR_M("%S: *%S\n"), PSTR_M("fan"),	\
				  PSTR_M(#state_new));			\
			fan_state_changed = true;			\
			fan_set_state_do(state_new);			\
		}							\
	while (0)

ISR(PCINT1_vect)
{
	timestamp now;

	fan_timestamp_last_element++;
	fan_timestamp_last_element %= FAN_TIMESTAMPS;

	timekeeping_now_timestamp(&now);
	fan_timestamps[fan_timestamp_last_element] = now;

	fan_timestamps_dirty = true;
}

static void fan_output_disable(void)
{
	PORTC &= ~(_BV(PORTC6) | _BV(PORTC7));
	DDRC |= _BV(DD6) | _BV(DD7);
}

static void fan_output_enable_low(void)
{
	DDRC &= ~_BV(DD7);
	PORTC |= _BV(PORTC7);

	PORTC &= ~_BV(PORTC6);
	DDRC |= _BV(DD6);
}

static void fan_output_enable_high(void)
{
	DDRC &= ~_BV(DD6);
	PORTC &= ~_BV(PORTC6);

	PORTC &= ~_BV(PORTC7);
	DDRC |= _BV(DD7);
}

static void fan_timediff_max(timestamp_interval *out)
{
	const timestamp_interval timediff_max = {
		.ticks = (uint32_t)TIMEKEEPING_HZ * 60 /
		((uint32_t)FAN_RPM_MIN * FAN_PULSES_PER_ROT),

		.counts = timekeeping_counts_per_tick() * TIMEKEEPING_HZ * 60 /
		((uint32_t)FAN_RPM_MIN * FAN_PULSES_PER_ROT) %
		timekeeping_counts_per_tick()
	};

	*out = timediff_max;
}

static void fan_recalc_rpm(const timestamp timestamps_tmp[],
			   uint8_t timestamps_tmp_last_element,
			   uint16_t *rpm, uint8_t *rpm_div)
{
	const timestamp_interval timediff_min = {
		.ticks = (uint32_t)TIMEKEEPING_HZ * 60 /
		((uint32_t)FAN_RPM_MAX * FAN_PULSES_PER_ROT),

		.counts = timekeeping_counts_per_tick() * TIMEKEEPING_HZ * 60 /
		((uint32_t)FAN_RPM_MAX * FAN_PULSES_PER_ROT) %
		timekeeping_counts_per_tick()
	};

	const timestamp_interval timediff_min_absolute = {
		.ticks = (uint32_t)TIMEKEEPING_HZ * 60 /
		((uint32_t)FAN_RPM_MAX_ABSOLUTE * FAN_PULSES_PER_ROT),

		.counts = timekeeping_counts_per_tick() * TIMEKEEPING_HZ * 60 /
		((uint32_t)FAN_RPM_MAX_ABSOLUTE * FAN_PULSES_PER_ROT) %
		timekeeping_counts_per_tick()
	};

	timestamp_interval timediff_max;
	fan_timediff_max(&timediff_max);

	*rpm = 0;
	*rpm_div = 0;

	/* these checks below should really be static asserts */

	/* need some minimum resolution */
	if (timediff_min.ticks == 0 && timediff_min.counts < 10)
		return;

	/* make sure we don't overflow countsdiff and co. later */
	if (((uint64_t)timediff_max.ticks * timekeeping_counts_per_tick() +
	     timediff_max.counts) * FAN_PULSES_PER_ROT > UINT32_MAX)
		return;

	uint8_t idx = timestamps_tmp_last_element;

	const timestamp *prev_timestamp = &timestamps_tmp[idx];

	timestamp_interval timediff;
	do {
		timestamp now;
		timekeeping_now_timestamp(&now);

		timestamp_diff(&now, prev_timestamp, &timediff);
		if (timestampi_cmp(&timediff, &timediff_max, >))
			return;
	} while (0);

	for (uint8_t ctr = 0; ctr < FAN_TIMESTAMPS - 1; ctr++) {
		if (idx == 0)
			idx = FAN_TIMESTAMPS - 1;
		else
			idx--;

		const timestamp *cur_timestamp = &timestamps_tmp[idx];

		timestamp_diff(prev_timestamp, cur_timestamp, &timediff);

		if (fan_debug_log_timediffs()) {
			dprintf_P(PSTR("fan: t1(%"PRIu32", %"PRIu16"), t2(%"PRIu32", %"PRIu16")\n"),
				  prev_timestamp->ticks, prev_timestamp->counts,
				  cur_timestamp->ticks, cur_timestamp->counts);
			dprintf_P(PSTR("fan: d(%"PRIu32", %"PRIu16")\n"),
				  timediff.ticks, timediff.counts);
		}

		if (timestampi_cmp(&timediff, &timediff_min_absolute, <))
			goto invalid_rpm;
		else if (timestampi_cmp(&timediff, &timediff_min, <))
			timediff = timediff_min;
		else if (timestampi_cmp(&timediff, &timediff_max, >))
			break;

		uint32_t countsdiff = timediff.ticks *
			timekeeping_counts_per_tick() + timediff.counts;

		_Static_assert(FAN_RPM_MAX * ((uint64_t)FAN_TIMESTAMPS - 1) <=
			       UINT16_MAX,
			       "too many fan timestamps or too high max fan RPM");

		*rpm += timekeeping_counts_per_tick() * TIMEKEEPING_HZ * 60 /
			(countsdiff * FAN_PULSES_PER_ROT);

		_Static_assert(FAN_TIMESTAMPS - 1 <= UINT8_MAX,
			       "too many fan timestamps");

		++*rpm_div;

		prev_timestamp = cur_timestamp;
	}

	return;

invalid_rpm:
	*rpm = 0;
	*rpm_div = 0;
}

uint16_t fan_rpm(void)
{
	static uint16_t rpm;
	static timestamp timestamp_last;

	uint8_t rpm_div;
	timestamp timestamps_tmp[FAN_TIMESTAMPS];
	uint8_t timestamps_tmp_last_element;

	timestamps_tmp_last_element = UINT8_MAX;
	_Static_assert(FAN_TIMESTAMPS <= UINT8_MAX,
		       "too many fan timestamps");

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();

		if (fan_timestamps_dirty) {
			memcpy(timestamps_tmp, fan_timestamps,
			       sizeof(timestamps_tmp));
			timestamps_tmp_last_element =
				fan_timestamp_last_element;

			timestamp_last =
				timestamps_tmp[timestamps_tmp_last_element];

			fan_timestamps_dirty = false;
		}

		_MemoryBarrier();
	}

	if (timestamps_tmp_last_element < FAN_TIMESTAMPS) {
		fan_recalc_rpm(timestamps_tmp, timestamps_tmp_last_element,
			       &rpm, &rpm_div);

		if (rpm_div > 1)
			rpm /= rpm_div;
	} else if (rpm > 0) {
		timestamp_interval timediff_max;
		fan_timediff_max(&timediff_max);

		timestamp now;
		timekeeping_now_timestamp(&now);

		timestamp_interval timediff;
		timestamp_diff(&now, &timestamp_last, &timediff);
		if (timestampi_cmp(&timediff, &timediff_max, >)) {
			dprintf_P(PSTR_M("fan: no pulse for too long\n"));
			rpm = 0;
		}
	}

	return rpm;
}

static bool fan_is_off_state(void)
{
	return fan_state == FAN_DISABLED;
}

static bool fan_is_low_state(void)
{
	return fan_state == FAN_LOW_START || fan_state == FAN_LOW_RUN;
}

static bool fan_is_low_output_state(void)
{
	return fan_state == FAN_LOW_RUN;
}

static bool fan_is_high_state(void)
{
	return fan_state == FAN_HIGH_START || fan_state == FAN_HIGH_RUN;
}

#if 0
static bool fan_is_high_output_state(void)
{
	return fan_state == FAN_LOW_START || fan_state == FAN_HIGH_START ||
		fan_state == FAN_HIGH_RUN || fan_state == FAN_FAIL ||
		fan_state == FAN_INIT;
}

static bool fan_is_running_state(void)
{
	return fan_is_low_state() || fan_is_high_state();
}
#endif

static bool fan_is_spinup_state(void)
{
	return fan_state == FAN_LOW_START || fan_state == FAN_HIGH_START;
}

static void fan_set_state_do(fan_states state_new)
{
	bool was_init_state = fan_state == FAN_INIT;
	bool was_off_state = fan_is_off_state();

	fan_state = state_new;

	if ((was_init_state || was_off_state) && !fan_is_off_state()) {
		const timestamp_interval poll_period =
			TIMESTAMPI_FROM_MS(FAN_POLL_PERIOD);

		/* delay first check by poll period to let RPM pulses settle */
		timestamp now;
		timekeeping_now_timestamp(&now);
		timestamp_add(&now, &poll_period, &fan_next_rpm_check);
	}

	if (fan_is_spinup_state()) {
		const timestamp_interval spinup_max_time =
			TIMESTAMPI_FROM_MS(FAN_SPINUP_MAX_TIME);

		timestamp now;
		timekeeping_now_timestamp(&now);
		timestamp_add(&now, &spinup_max_time, &fan_spinup_deadline);
	}

	if (fan_is_off_state() || fan_output_always_off())
		fan_output_disable();
	else if (fan_is_low_output_state())
		fan_output_enable_low();
	else /* high output */
		fan_output_enable_high();
}

void fan_poll(void)
{
	fan_state_changed = false;

	if (fan_target_state == FAN_OFF && !fan_is_off_state())
		FAN_SETSTATE(FAN_DISABLED);
	else if (fan_state != FAN_FAIL) {
		if (fan_target_state == FAN_LOW && !fan_is_low_state())
			FAN_SETSTATE(FAN_LOW_START);
		else if (fan_target_state == FAN_HIGH && !fan_is_high_state())
			FAN_SETSTATE(FAN_HIGH_START);
	}

	if (fan_is_off_state())
		return;

	timestamp now;
	timekeeping_now_timestamp(&now);
	if (timestamp_temporal_cmp(&now, &fan_next_rpm_check, <))
		return;

	uint16_t rpm = fan_rpm();
	dprintf_P(PSTR("fan: %"PRIu16" RPM\n"), rpm);

	if (fan_state == FAN_FAIL) {
		if (rpm >= FAN_RPM_HIGH_MIN)
			FAN_SETSTATE(FAN_HIGH_RUN);
		else if (rpm >= FAN_RPM_LOW_MIN)
			FAN_SETSTATE(FAN_LOW_RUN);
	} else if (fan_is_spinup_state()) {
		if (rpm >= FAN_RPM_HIGH_MIN ||
		    (fan_is_low_state() && rpm >= FAN_RPM_LOW_MIN)) {
			if (fan_is_low_state())
				FAN_SETSTATE(FAN_LOW_RUN);
			else /* high state */
				FAN_SETSTATE(FAN_HIGH_RUN);
		} else if (timestamp_temporal_cmp(&now, &fan_spinup_deadline,
						  >=))
			FAN_SETSTATE(FAN_FAIL);
	} else if (fan_is_low_state() && rpm < FAN_RPM_LOW_MIN)
		FAN_SETSTATE(FAN_FAIL);
	else if (fan_is_high_state() && rpm < FAN_RPM_HIGH_MIN)
		FAN_SETSTATE(FAN_FAIL);

	do {
		const timestamp_interval poll_period =
			TIMESTAMPI_FROM_MS(FAN_POLL_PERIOD);

		timekeeping_now_timestamp(&now);
		timestamp_add(&now, &poll_period, &fan_next_rpm_check);
	} while (0);
}

void fan_get_next_poll_time(timestamp *next_poll)
{
	if (fan_state_changed)
		timekeeping_now_timestamp(next_poll);
	else if (!fan_is_off_state())
		*next_poll = fan_next_rpm_check;
	else
		timekeeping_timestamp_max_future(next_poll);
}

static void fan_set_target_state(fan_target_states state_new)
{
	if (fan_target_state == state_new)
		return;

	fan_target_state = state_new;
	fan_state_changed = true;
}

void fan_disable(void)
{
	fan_set_target_state(FAN_OFF);
}

void fan_enable_low(void)
{
	fan_set_target_state(FAN_LOW);
}

void fan_enable_high(void)
{
	fan_set_target_state(FAN_HIGH);
}

bool fan_has_failed(void)
{
	return fan_state == FAN_FAIL;
}

void fan_setup(void)
{
	timestamp now, now_opposite;
	timekeeping_now_timestamp(&now);

	timestamp_opposite(&now, &now_opposite);

	bool timestamp_cnt_even = FAN_TIMESTAMPS % 2 == 0;
	for (uint8_t ctr = 0; ctr < FAN_TIMESTAMPS; ctr++)
		if (ctr % 2 == (timestamp_cnt_even ? 0 : 1))
			fan_timestamps[ctr] = now;
		else
			fan_timestamps[ctr] = now_opposite;

	fan_timestamp_last_element = FAN_TIMESTAMPS - 1;

	/* so fan_rpm() will recalc rpm */
	fan_timestamps_dirty = true;

	PCMSK1 |= _BV(PCINT8);
	PCICR |= _BV(PCIE1);

	if (fan_output_always_off())
		fan_output_disable();
	else
		fan_output_enable_high();

	fan_target_state = FAN_HIGH;
	fan_state = FAN_INIT;

	/* so fan_get_next_poll_time() will return now */
	fan_state_changed = true;
}
