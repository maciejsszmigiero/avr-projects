/*
 * Smart UPS Addon: general setup (watchdog, ports, etc.) and the main loop
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
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/signature.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "../lib/debug.h"
#include "../lib/i2c.h"
#include "../lib/misc.h"
#include "../lib/timekeeping.h"
#include "serial.h"
#include "temp.h"

LOCKBITS = LB_MODE_1 & BLB0_MODE_1 & BLB1_MODE_2;

FUSES = {
	.low = FUSE_SUT_CKSEL3 & FUSE_SUT_CKSEL5,
	.high = FUSE_SPIEN & FUSE_WDTON & FUSE_EESAVE & FUSE_BOOTSZ0 & FUSE_BOOTRST,
	.extended = FUSE_BODLEVEL1
};

static void wdt_setup(void)
{
	wdt_reset();
	wdt_enable(WDTO_4S);
	wdt_reset();
}

static void ports_pullup_disable(void)
{
	MCUCR |= _BV(PUD);
}

static void ports_pullup_enable(void)
{
	MCUCR &= ~_BV(PUD);
}

#define PORTS_DEFAULT_SETUP_PORT(port)	\
	do {				\
		DDR ## port = 0;	\
		PORT ## port = 0xff;	\
	} while (0)
static void ports_default_setup(void)
{
	PORTS_DEFAULT_SETUP_PORT(A);
	PORTS_DEFAULT_SETUP_PORT(B);
	PORTS_DEFAULT_SETUP_PORT(C);
	PORTS_DEFAULT_SETUP_PORT(D);
}
#undef PORTS_DEFAULT_SETUP_PORT

static void powerdown_all(void)
{
	power_all_disable();
}

static void bootloader_ports_setup(void)
{
	DDRD &= ~_BV(DD7);
	PORTD &= ~_BV(PORTD7);
}

static void fan_ports_setup(void)
{
	PORTC &= ~(_BV(PORTC6) | _BV(PORTC7));
	DDRC &= ~_BV(DD6);
	DDRC |= _BV(DD7);

	DDRB &= ~_BV(PORTB0);
	PORTB |= _BV(PORTB0);
}

static void serial0_ports_setup(void)
{
	DDRD &= ~(_BV(DD0) | _BV(DD5));
	PORTD &= ~(_BV(PORTD0) | _BV(PORTD5));
	PORTD |= _BV(PORTD1);
	DDRD |= _BV(DD1);
}

static void serial1_ports_setup(void)
{
	DDRD &= ~_BV(DD2);
	PORTD &= ~_BV(PORTD2);
	PORTD |= _BV(PORTD3);
	DDRD |= _BV(DD3);
}

static void serial01_ports_passthrough(bool enable)
{
	if (enable) {
		PORTD |= _BV(PORTD5);
		DDRD |= _BV(DD5);
	} else {
		PORTD &= ~_BV(PORTD5);
		DDRD |= _BV(DD5);
	}
}

static void i2c_ports_setup(void)
{
	DDRC &= ~(_BV(DD0) | _BV(DD1));
	PORTC &= ~(_BV(PORTC0) | _BV(PORTC1));
}

static void setup(void)
{
	wdt_setup();

	powerdown_all();

	ports_pullup_disable();
	ports_default_setup();
	bootloader_ports_setup();
	fan_ports_setup();
	serial0_ports_setup();
	serial1_ports_setup();
	i2c_ports_setup();
	ports_pullup_enable();

	serial01_ports_passthrough(true);

	timekeeping_setup();

	debug_setup();

	i2c_setup();

	temp_setup();

	serial_setup();
	serial01_ports_passthrough(false);

	wdt_reset();
}

#define MAIN_GET_TIMEOUT(funname, ...)					\
	do {								\
		timestamp module_next_poll_time;			\
		funname(__VA_ARGS__ &module_next_poll_time);		\
		if (!next_poll_time_set ||				\
		    timestamp_temporal_cmp(&module_next_poll_time,	\
					   &next_poll_time,		\
					   <)) {			\
			next_poll_time = module_next_poll_time;	\
			next_poll_time_set = true;			\
		}							\
	} while (0)

int main(void)
{
	/* a tick will always wake us up */
	const timestamp_interval worst_sleep_interval = {
		.ticks = 1,
		.counts = 0
	};

	cli();
	setup();
	sei();

	dprintf_P(PSTR_M("BOOTED UP\n"));

	set_sleep_mode(SLEEP_MODE_IDLE);
	while (1) {
		temp_poll();
		serial_poll();

		cli();

		i2c_poll_atomic();
		serial_poll_atomic();

		bool can_sleep = true;

		timestamp next_poll_time;
		bool next_poll_time_set = false;

		MAIN_GET_TIMEOUT(temp_get_next_poll_time);
		MAIN_GET_TIMEOUT(serial_get_next_poll_time);
		MAIN_GET_TIMEOUT(i2c_get_next_poll_time);

		if (next_poll_time_set) {
			timestamp now, worst_wakeup_time;
			timekeeping_now_timestamp(&now);
			timestamp_add(&now, &worst_sleep_interval,
				      &worst_wakeup_time);

			if (timestamp_temporal_cmp(&next_poll_time,
						   &worst_wakeup_time,
						   <))
				can_sleep = false;
		}

		wdt_reset();

		if (can_sleep) {
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
			wdt_reset();
		} else
			sei();
	}

	return 0;
}
