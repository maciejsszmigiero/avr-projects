/*
 * AVR Library: Timekeeping subsystem and functions
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef _LIB_TIMEKEEPING_H_
#define _LIB_TIMEKEEPING_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <avr/cpufunc.h>
#include <avr/io.h>
#include <util/atomic.h>

/*
 * HZ, that is, ticks per second
 * must be <= 1000
 */
#ifndef TIMEKEEPING_HZ
#define TIMEKEEPING_HZ 8
#endif

/* prescaler value for the timekeeping timer */
#ifndef TIMEKEEPING_DIV
#define TIMEKEEPING_DIV 64
#endif

/* define to allow inexact timer frequency */
/* #define TIMEKEEPING_ALLOW_INEXACT_FREQ */

/* absolute timestamp */
typedef struct {
	uint32_t ticks;
	uint16_t counts;
} timestamp;

/* interval (relative timestamp) */
typedef struct {
	uint32_t ticks;
	uint16_t counts;
} timestamp_interval;

/* max input UINT32_MAX msecs = ~49 days 17 hours */
#define TIMESTAMPI_FROM_MS(value)					\
	{								\
		.ticks = (uint64_t)(value) * TIMEKEEPING_HZ / 1000,	\
		.counts = (uint64_t)(value) * TIMEKEEPING_HZ *		\
			timekeeping_counts_per_tick() / 1000 %		\
			timekeeping_counts_per_tick()			\
	}

/* max input UINT32_MAX usecs = ~1 hour 11 minutes */
#define TIMESTAMPI_FROM_US(value)					\
	{								\
		.ticks = (uint32_t)(value) / 1000 * TIMEKEEPING_HZ /	\
			1000,						\
		.counts = (uint64_t)(value) * TIMEKEEPING_HZ *		\
			timekeeping_counts_per_tick() /		\
			((uint32_t)1000 * 1000) %			\
			timekeeping_counts_per_tick()			\
	}

/* don't directly use this variable */
extern uint32_t timekeeping_ticks;

#define timestamp_check_type(in)				\
	do {							\
		const timestamp tmp_timestamp_check_type	\
			__attribute__((unused)) = *(in);	\
	} while (0)

#define timestampi_check_type(in)					\
	do {								\
		const timestamp_interval tmp_timestampi_check_type	\
			__attribute__((unused)) = *(in);		\
	} while (0)

/* all timestamp values are (possibly) valid */
#undef timestamp_clear
#undef timestamp_isset

#define timestampi_zero(in)			\
	do {					\
		timestampi_check_type(in);	\
						\
		(in)->ticks = 0;		\
		(in)->counts = 0;		\
	} while (0)

#define timestampi_iszero(in)				\
	({						\
		timestampi_check_type(in);		\
							\
		(in)->ticks == 0 && (in)->counts == 0;	\
	})

#define timestamp_cmp_internal(in1, in2, oper)				   \
	((in1)->ticks == (in2)->ticks ? (in1)->counts oper (in2)->counts : \
	 (in1)->ticks oper (in2)->ticks)

/*
 * raw compare of timestamp values
 *
 * when comparing absolute timestamps remember about possiblity of
 * a wraparound - usually it is better to use timestamp_temporal_cmp()
 * instead of this macro
 */
#define timestamp_value_cmp(in1, in2, oper)		\
	({						\
		timestamp_check_type(in1);		\
		timestamp_check_type(in2);		\
							\
		timestamp_cmp_internal(in1, in2, oper); \
	})

/*
 * computes time elapsed from in2 to in1 taking into
 * consideration a possible wraparound in meantime
 *
 * the result can be directly used as long as in1 is not earlier than in2
 */
#define timestamp_diff(in1, in2, result)					\
	do {									\
		timestamp_check_type(in1);					\
		timestamp_check_type(in2);					\
		timestampi_check_type(result);					\
										\
		(result)->ticks = (in1)->ticks - (in2)->ticks;			\
										\
		if ((in1)->counts >= (in2)->counts)				\
			(result)->counts = (in1)->counts - (in2)->counts;	\
		else {								\
			(result)->counts = timekeeping_counts_per_tick() - 1;	\
			(result)->counts -= ((in2)->counts - 1);		\
			(result)->counts += (in1)->counts;			\
										\
			(result)->ticks--;					\
		}								\
	} while (0)

/*
 * returns a timestamp that will have the maximum difference (time elapsed)
 * from an input one when compared by timestamp_diff()
 */
#define timestamp_opposite(in, result)				\
	do {							\
		timestamp_check_type(in);			\
		timestamp_check_type(result);			\
								\
		(result)->ticks = (in)->ticks + UINT32_MAX / 2; \
		(result)->counts = (in)->counts;		\
	} while (0)

/*
 * compares absolute timestamp in1 with in2 treating in1's that lie forward
 * within UINT32_MAX / 2 ticks (inclusive) as in the future with regard to in2
 */
#define timestamp_temporal_cmp(in1, in2, oper)					\
	({									\
		const timestamp_interval timestamp_temporal_cmp_limit = {	\
			.ticks = UINT32_MAX / 2,				\
			.counts = 0						\
		};								\
										\
		bool timestamp_temporal_cmp_is_equal = 0 oper 0;		\
		bool timestamp_temporal_cmp_is_greater = 1 oper 0;		\
		bool timestamp_temporal_cmp_is_less = 0 oper 1;		\
		bool timestamp_temporal_cmp_is_inequal =			\
			timestamp_temporal_cmp_is_greater &&			\
			timestamp_temporal_cmp_is_less;			\
		timestamp_interval timestamp_temporal_cmp_d;			\
		bool timestamp_temporal_cmp_result;				\
										\
		timestamp_diff(in1, in2, &timestamp_temporal_cmp_d);		\
		if (timestamp_temporal_cmp_is_inequal)				\
			timestamp_temporal_cmp_result =			\
				!timestampi_iszero(&timestamp_temporal_cmp_d);	\
		else if (timestampi_iszero(&timestamp_temporal_cmp_d))		\
			timestamp_temporal_cmp_result =			\
				timestamp_temporal_cmp_is_equal;		\
		else if (timestamp_temporal_cmp_is_greater)			\
			timestamp_temporal_cmp_result =			\
				timestampi_cmp(&timestamp_temporal_cmp_d,	\
					       &timestamp_temporal_cmp_limit,	\
					       <=);				\
		else if (timestamp_temporal_cmp_is_less)			\
			timestamp_temporal_cmp_result =			\
				timestampi_cmp(&timestamp_temporal_cmp_d,	\
					       &timestamp_temporal_cmp_limit,	\
					       >);				\
		else								\
			timestamp_temporal_cmp_result = false;			\
										\
		timestamp_temporal_cmp_result;					\
	})

/* comparison of timestampi values */
#define timestampi_cmp(in1, in2, oper)					\
	({								\
		timestampi_check_type(in1);				\
		timestampi_check_type(in2);				\
									\
		timestamp_cmp_internal(in1, in2, oper);		\
	})

/* add an interval (timestampi) to an absolute timestamp */
#define timestamp_add(in, interval, result)				\
	do {								\
		uint32_t tmp_timestamp_add_counts;			\
									\
		timestamp_check_type(in);				\
		timestampi_check_type(interval);			\
		timestamp_check_type(result);				\
									\
		(result)->ticks = (in)->ticks + (interval)->ticks;	\
									\
		tmp_timestamp_add_counts = (in)->counts;		\
		tmp_timestamp_add_counts += (interval)->counts;	\
		if (tmp_timestamp_add_counts >=			\
		    timekeeping_counts_per_tick()) {			\
			tmp_timestamp_add_counts -=			\
				timekeeping_counts_per_tick();		\
			(result)->ticks++;				\
		}							\
									\
		(result)->counts = tmp_timestamp_add_counts;		\
	} while (0)

/* returns the current time (just ticks) */
static inline uint32_t timekeeping_now_ticks(void)
{
	uint32_t val;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();
		val = timekeeping_ticks;
		_MemoryBarrier();
	}

	return val;
}

/* returns the current time (the whole timestamp) */
void timekeeping_now_timestamp(timestamp *out);

/*
 * returns a timestamp that will be considered "in the past" for as long as
 * possible when compared temporally in the future with then current time
 */
#define timekeeping_timestamp_max_past(out)		 \
	do {						 \
		timestamp_check_type(out);		 \
							 \
		timekeeping_now_timestamp(out);	 \
							 \
		/* see comment in */			 \
		/* timekeeping_timestamp_max_future() */ \
		(out)->ticks -= UINT32_MAX / 4;	 \
	} while (0)

/* like timekeeping_timestamp_max_past(), just in the opposite direction */
#define timekeeping_timestamp_max_future(out)		\
	do {						\
		timestamp_check_type(out);		\
							\
		timekeeping_now_timestamp(out);	\
							\
		/* compromise between the furthest */	\
		/* future and the widest time window */ \
		/* until three timestamps: max past, */ \
		/* now and max future no longer */	\
		/* correctly compare temporally */	\
		(out)->ticks += UINT32_MAX / 4;	\
	} while (0)

static inline uint32_t timekeeping_counts_per_tick_internal(uint16_t *top_out)
{
	const uint64_t counts =
		F_CPU / ((uint64_t)TIMEKEEPING_DIV * TIMEKEEPING_HZ);
	const uint64_t counts_rem __attribute__((unused)) =
		F_CPU % ((uint64_t)TIMEKEEPING_DIV * TIMEKEEPING_HZ);
	const uint64_t top =
		counts > 0 ? counts - 1 : 0;

	_Static_assert(top > 0,
		       "too high TIMEKEEPING_DIV or TIMEKEEPING_HZ for CPU freq");
	_Static_assert(top <= UINT16_MAX,
		       "too low TIMEKEEPING_DIV or TIMEKEEPING_HZ for CPU freq");

#ifndef TIMEKEEPING_ALLOW_INEXACT_FREQ
	_Static_assert(counts_rem == 0,
		       "inexact timer frequency");
#endif

	if (top_out != NULL)
		*top_out = top;

	return counts;
}

/* returns how many counts are in a one tick */
static inline uint32_t timekeeping_counts_per_tick(void)
{
	return timekeeping_counts_per_tick_internal(NULL);
}

/*
 * setup the timekeeping subsystem: must be called before any other timekeeping
 * function and with interrupts disabled
 */
void timekeeping_setup(void);

#endif
