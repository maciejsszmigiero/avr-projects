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

#include <avr/interrupt.h>
#include <avr/power.h>

#include "timekeeping.h"

uint32_t timekeeping_ticks;

ISR(TIMER3_COMPA_vect)
{
	timekeeping_ticks++;
}

void timekeeping_now_timestamp(timestamp *out)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		_MemoryBarrier();

		while (1) {
			if (bit_is_set(TIFR3, OCF3A)) {
				timekeeping_ticks++;

				TIFR3 = _BV(OCF3A);
			}

			out->counts = TCNT3;

			if (bit_is_set(TIFR3, OCF3A))
				continue;

			out->ticks = timekeeping_ticks;

			break;
		}

		_MemoryBarrier();
	}
}

static uint16_t timekeeping_calc_timer_top(void)
{
	uint16_t top;

	timekeeping_counts_per_tick_internal(&top);

	return top;
}

void timekeeping_setup(void)
{
	power_timer3_enable();

	TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
	TIMSK3 &= ~(_BV(ICIE3) | _BV(OCIE3B) | _BV(OCIE3A) | _BV(TOIE3));
	TCCR3A &= ~(_BV(COM3A1) | _BV(COM3A0) | _BV(COM3B1) | _BV(COM3B0) |
		    _BV(WGM31) | _BV(WGM30));
	TCCR3B &= ~_BV(WGM33);
	TCCR3B |= _BV(WGM32);

	/*
	 * make ticks overflow in ~3 minutes so bugs related to
	 * wraparound handling are caught earlier
	 */
	timekeeping_ticks = UINT32_MAX - (3 * 60 * TIMEKEEPING_HZ -
					  3 * TIMEKEEPING_HZ);
	TCNT3 = 0;
	OCR3A = timekeeping_calc_timer_top();

	TIFR3 = _BV(OCF3A);
	TIMSK3 |= _BV(OCIE3A);

#if TIMEKEEPING_DIV == 1
	TCCR3B |= _BV(CS30);
#elif TIMEKEEPING_DIV == 8
	TCCR3B |= _BV(CS31);
#elif TIMEKEEPING_DIV == 64
	TCCR3B |= _BV(CS30) | _BV(CS31);
#elif TIMEKEEPING_DIV == 256
	TCCR3B |= _BV(CS32);
#elif TIMEKEEPING_DIV == 1024
	TCCR3B |= _BV(CS32) | _BV(CS30);
#else
#error unknown TIMEKEEPING_DIV value
#endif
}
