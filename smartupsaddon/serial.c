/*
 * Smart UPS Addon: serial ports data processor
 *
 * Copyright (C) 2017 Maciej S. Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "../lib/debug.h"
#include "../lib/misc.h"
#include "fan.h"
#include "serial-base.h"
#include "serial.h"
#include "temp.h"

/*
 * how long (in ms) to wait after receiving 'y' command
 *
 * however, we don't wait at all if there was at least that long period of
 * silence from CPU before and after we received that command (taken together)
 */
#define SERIAL_Y_RECV_SILENCE 125

/*
 * beginning of string to match for replacement in 'y' command reply from CPU
 * (match ends at CRLF) and timeout of this match in ms
 */
#define SERIAL_Y_REPLY_MATCH_STR "(C) "
#define SERIAL_Y_REPLY_MATCH_TIMEOUT 1000

#ifdef SERIAL_DEBUG_LOG_DISABLE
#undef dprintf
#undef dprintf_P
#define dprintf(...)
#define dprintf_P(...)
#endif

typedef enum { SERIAL_IDLE,
	       SERIAL_Y_RECV_SILENCE_WAIT, SERIAL_Y_RECV_SILENCE_GAP,
	       SERIAL_Y_RECV_REPLY_MATCH, SERIAL_Y_RECV_REPLY_WAIT_CRLF,
	       SERIAL_Y_RECV_REPLY_PRINT_FAN_HEADER,
	       SERIAL_Y_RECV_REPLY_PRINT_FAN,
	       SERIAL_Y_RECV_REPLY_PRINT_TEMP_HEADER,
	       SERIAL_Y_RECV_REPLY_PRINT_TEMP,
	       SERIAL_Y_RECV_REPLY_PRINT_TEMP_NEXT,
	       SERIAL_Y_RECV_REPLY_PRINT_CRLF,
	       SERIAL_Y_RECV_FAIL_MATCH
} serial_states;

static /* serial_states */ uint8_t serial_state;
static bool serial_state_changed;

#define serialconn_setup serial0_setup
#define serialconn_rx_empty serial0_rx_is_empty
#define serialconn_rx_len serial0_rx_len
#define serialconn_rx_get serial0_rx_get
#define serialconn_rx_peek serial0_rx_peek
#define serialconn_rx_peek_at serial0_rx_peek_at
#define serialconn_tx_empty serial0_tx_is_empty
#define serialconn_tx_put serial0_tx_put
#define serialcpu_setup serial1_setup
#define serialcpu_rx_empty serial1_rx_is_empty
#define serialcpu_rx_len serial1_rx_len
#define serialcpu_rx_get serial1_rx_get
#define serialcpu_rx_peek serial1_rx_peek
#define serialcpu_rx_peek_at serial1_rx_peek_at
#define serialcpu_tx_empty serial1_tx_is_empty
#define serialcpu_tx_put serial1_tx_put

static timestamp serialcpu_last_rx;
static timestamp serialcpu_y_recv_gap_end;
static timestamp serialcpu_y_reply_deadline;

static uint8_t serial_tmp_ctr;

#define SERIAL_SETSTATE(state_new)					\
	do								\
		if (serial_state != state_new) {			\
			dprintf_P(PSTR_M("%S: *%S\n"), PSTR_M("serial"),\
				  PSTR_M(#state_new));			\
			serial_state_changed = true;			\
			serial_set_state_do(state_new);		\
		}							\
	while (0)

static bool serial_is_y_recv_state(void)
{
	return serial_state == SERIAL_Y_RECV_SILENCE_WAIT ||
		serial_state == SERIAL_Y_RECV_SILENCE_GAP ||
		serial_state == SERIAL_Y_RECV_REPLY_MATCH ||
		serial_state == SERIAL_Y_RECV_REPLY_WAIT_CRLF ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN_HEADER ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP_HEADER ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP_NEXT ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_CRLF;
}

static bool serial_is_y_reply_print_state(void)
{
	return serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN_HEADER ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP_HEADER ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP_NEXT ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_CRLF;
}

static bool serial_is_conn_tx_empty_wait_state(void)
{
	return serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN_HEADER ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP_HEADER ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP ||
		serial_state == SERIAL_Y_RECV_REPLY_PRINT_CRLF;
}

#define SERIALCONN_PRINTF(buflen, format, ...)				\
	do {								\
		uint8_t tmp_serialconnf_printf_buf[buflen];		\
		int tmp_serialconnf_printf_ret;			\
									\
		tmp_serialconnf_printf_ret =				\
			snprintf_P((char *)tmp_serialconnf_printf_buf,	\
				   sizeof(tmp_serialconnf_printf_buf),	\
				   format, ##__VA_ARGS__);		\
									\
		if (tmp_serialconnf_printf_ret >			\
		    sizeof(tmp_serialconnf_printf_buf))		\
			tmp_serialconnf_printf_ret =			\
				sizeof(tmp_serialconnf_printf_buf);	\
									\
		for (uint8_t ctr = 0; ctr < tmp_serialconnf_printf_ret; \
		     ctr++)						\
			serialconn_tx_put(tmp_serialconnf_printf_buf	\
					  [ctr]);			\
	} while (0)

static void serial_set_state_do(serial_states state_new)
{
	serial_state = state_new;

	if (serial_state == SERIAL_Y_RECV_SILENCE_WAIT) {
		const timestamp_interval recv_y_silence =
			TIMESTAMPI_FROM_MS(SERIAL_Y_RECV_SILENCE);

		timestamp now;

		timekeeping_now_timestamp(&now);
		timestamp_add(&now, &recv_y_silence,
			      &serialcpu_y_recv_gap_end);
	} else if (serial_state == SERIAL_Y_RECV_REPLY_MATCH) {
		const timestamp_interval recv_y_reply_match_timeout =
			TIMESTAMPI_FROM_MS(SERIAL_Y_REPLY_MATCH_TIMEOUT);

		serialcpu_tx_put('y');

		timestamp now;

		timekeeping_now_timestamp(&now);
		timestamp_add(&now, &recv_y_reply_match_timeout,
			      &serialcpu_y_reply_deadline);

		serial_tmp_ctr = 0;
	} else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN_HEADER) {
		serialconn_tx_put('F');
		serialconn_tx_put('a');
		serialconn_tx_put('n');
		serialconn_tx_put(':');
		serialconn_tx_put(' ');
	} else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN) {
		if (fan_has_failed()) {
			serialconn_tx_put('F');
			serialconn_tx_put('A');
			serialconn_tx_put('I');
			serialconn_tx_put('L');
			serialconn_tx_put(' ');
		} else {
			uint16_t rpm = fan_rpm();

			SERIALCONN_PRINTF(strlen("65535"), PSTR("%" PRIu16),
					  rpm);

			serialconn_tx_put('R');
			serialconn_tx_put('P');
			serialconn_tx_put('M');
			serialconn_tx_put(' ');
		}
	} else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP_HEADER) {
		serialconn_tx_put('T');
		serialconn_tx_put('e');
		serialconn_tx_put('m');
		serialconn_tx_put('p');
		serialconn_tx_put(':');

		serial_tmp_ctr = 0;
	} else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP) {
		if (serial_tmp_ctr > 0)
			serialconn_tx_put(',');
		serialconn_tx_put(' ');
		serialconn_tx_put('T');

		SERIALCONN_PRINTF(strlen("255"), PSTR("%" PRIu8),
				  serial_tmp_ctr);

		serialconn_tx_put(':');
		serialconn_tx_put(' ');

		int8_t temp_c, temp_min, temp_max;
		if (!temp_get(serial_tmp_ctr, &temp_c, &temp_min, &temp_max)) {
			serialconn_tx_put('F');
			serialconn_tx_put('A');
			serialconn_tx_put('I');
			serialconn_tx_put('L');
		} else {
			temp_reset_minmax(serial_tmp_ctr);

			SERIALCONN_PRINTF(strlen("-128"), PSTR("%" PRIi8),
					  temp_c);
			serialconn_tx_put('(');
			SERIALCONN_PRINTF(strlen("-128"), PSTR("%" PRIi8),
					  temp_min);
			serialconn_tx_put('/');
			SERIALCONN_PRINTF(strlen("-128"), PSTR("%" PRIi8),
					  temp_max);
			serialconn_tx_put(')');
			serialconn_tx_put('d');
			serialconn_tx_put('C');
		}
	} else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP_NEXT)
		serial_tmp_ctr++;
	else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_CRLF) {
		serialconn_tx_put('\r');
		serialconn_tx_put('\n');
	}
}

static void serialconn_rx_service(void)
{
	if (serial_is_y_recv_state())
		return;

	uint8_t rxchar;
	if (!serialconn_rx_get(&rxchar))
		return;

	if (rxchar == 'y') {
		SERIAL_SETSTATE(SERIAL_Y_RECV_SILENCE_WAIT);
		return;
	}

	serialcpu_tx_put(rxchar);
}

static void serialcpu_rx_service(void)
{
	if (serial_state == SERIAL_Y_RECV_REPLY_MATCH ||
	    serial_state == SERIAL_Y_RECV_REPLY_WAIT_CRLF ||
	    serial_is_y_reply_print_state())
		return;

	uint8_t rxchar;
	if (!serialcpu_rx_get(&rxchar))
		return;

	timekeeping_now_timestamp(&serialcpu_last_rx);

	serialconn_tx_put(rxchar);

	if (serial_state == SERIAL_Y_RECV_SILENCE_WAIT)
		SERIAL_SETSTATE(SERIAL_Y_RECV_SILENCE_GAP);
}

void serial_poll(void)
{
	serial_state_changed = false;

	/*
	 * order is important for immediate last cpu rx updating when
	 * entering SERIAL_Y_RECV_SILENCE_WAIT state
	 */
	serialconn_rx_service();
	serialcpu_rx_service();

	if (serial_state == SERIAL_Y_RECV_SILENCE_WAIT) {
		const timestamp_interval recv_y_silence =
			TIMESTAMPI_FROM_MS(SERIAL_Y_RECV_SILENCE);
		timestamp now, silence_end;

		timekeeping_now_timestamp(&now);
		timestamp_add(&serialcpu_last_rx, &recv_y_silence,
			      &silence_end);

		if (timestamp_temporal_cmp(&now, &silence_end,
					   <))
			return;

		SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_MATCH);
	} else if (serial_state == SERIAL_Y_RECV_SILENCE_GAP) {
		timestamp now;
		timekeeping_now_timestamp(&now);
		if (timestamp_temporal_cmp(&now, &serialcpu_y_recv_gap_end,
					   <))
			return;

		SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_MATCH);
	} else if (serial_state == SERIAL_Y_RECV_REPLY_MATCH) {
		uint8_t matchbuf[strlen(SERIAL_Y_REPLY_MATCH_STR)];

		serialcpu_rx_peek(matchbuf, sizeof(matchbuf),
				  &serial_tmp_ctr);

		if (memcmp_P(matchbuf, PSTR_M(SERIAL_Y_REPLY_MATCH_STR),
			     serial_tmp_ctr) != 0) {
			SERIAL_SETSTATE(SERIAL_Y_RECV_FAIL_MATCH);
			return;
		} else if (serial_tmp_ctr < sizeof(matchbuf)) {
			timestamp now;
			timekeeping_now_timestamp(&now);
			if (timestamp_temporal_cmp(&now,
						   &serialcpu_y_reply_deadline,
						   <))
				return;

			SERIAL_SETSTATE(SERIAL_Y_RECV_FAIL_MATCH);

			return;
		}

		SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_WAIT_CRLF);
	} else if (serial_state == SERIAL_Y_RECV_REPLY_WAIT_CRLF) {
		timestamp now;
		timekeeping_now_timestamp(&now);
		if (timestamp_temporal_cmp(&now,
					   &serialcpu_y_reply_deadline,
					   <))
			return;

		SERIAL_SETSTATE(SERIAL_Y_RECV_FAIL_MATCH);
	} else if (serial_is_conn_tx_empty_wait_state()) {
		if (!serialconn_tx_empty())
			return;

		if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN_HEADER)
			SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_PRINT_FAN);
		else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_FAN) {
			if (temp_get_count() > 0)
				SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_PRINT_TEMP_HEADER);
			else
				SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_PRINT_CRLF);
		} else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP_HEADER)
			SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_PRINT_TEMP);
		else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP)
			SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_PRINT_TEMP_NEXT);
		else /* SERIAL_Y_RECV_REPLY_PRINT_CRLF */
			SERIAL_SETSTATE(SERIAL_IDLE);
	} else if (serial_state == SERIAL_Y_RECV_REPLY_PRINT_TEMP_NEXT) {
		if (serial_tmp_ctr >= temp_get_count())
			SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_PRINT_CRLF);
		else
			SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_PRINT_TEMP);
	} else if (serial_state == SERIAL_Y_RECV_FAIL_MATCH)
		SERIAL_SETSTATE(SERIAL_IDLE);
}

void serial_poll_atomic(void)
{
	if (serial_state == SERIAL_Y_RECV_REPLY_WAIT_CRLF) {
		/*
		 * must always scan from the beginning since previously
		 * checked positions might have been overwritten in meantime
		 */
		for (uint8_t ctr = 0; true; ctr++) {
			uint8_t matchbuf[2];
			uint8_t matchlen;

			serialcpu_rx_peek_at(matchbuf, ctr, 2, &matchlen);
			if (matchlen < 2)
				return;

			if (matchbuf[0] != '\r' ||
			    matchbuf[1] != '\n')
				continue;

			if (serialcpu_rx_get(&matchbuf[0]))
				do {
					if (!serialcpu_rx_get(&matchbuf[1]))
						break;

					if (matchbuf[0] == '\r' &&
					    matchbuf[1] == '\n')
						break;

					matchbuf[0] = matchbuf[1];
				} while (1);

			SERIAL_SETSTATE(SERIAL_Y_RECV_REPLY_PRINT_FAN_HEADER);
			return;
		}
	}
}

void serial_get_next_poll_time(timestamp *next_poll)
{
	bool serialconn_needs_service =
		!serial_is_y_recv_state() && !serialconn_rx_empty();
	bool serialcpu_needs_service =
		serial_state != SERIAL_Y_RECV_REPLY_MATCH &&
		serial_state != SERIAL_Y_RECV_REPLY_WAIT_CRLF &&
		!serial_is_y_reply_print_state() &&
		!serialcpu_rx_empty();
	bool serialcpu_needs_match =
		serial_state == SERIAL_Y_RECV_REPLY_MATCH &&
		serialcpu_rx_len() > serial_tmp_ctr;
	bool serialconn_tx_empty_wait_finished =
		serial_is_conn_tx_empty_wait_state() &&
		serialconn_tx_empty();

	if (serial_state_changed || serialconn_needs_service ||
	    serialcpu_needs_service || serialcpu_needs_match ||
	    serialconn_tx_empty_wait_finished)
		timekeeping_now_timestamp(next_poll);
	else if (serial_state == SERIAL_Y_RECV_SILENCE_WAIT) {
		const timestamp_interval recv_y_silence =
			TIMESTAMPI_FROM_MS(SERIAL_Y_RECV_SILENCE);

		timestamp_add(&serialcpu_last_rx, &recv_y_silence,
			      next_poll);
	} else if (serial_state == SERIAL_Y_RECV_SILENCE_GAP)
		*next_poll = serialcpu_y_recv_gap_end;
	else if (serial_state == SERIAL_Y_RECV_REPLY_MATCH ||
		serial_state == SERIAL_Y_RECV_REPLY_WAIT_CRLF)
		*next_poll = serialcpu_y_reply_deadline;
	else
		timekeeping_timestamp_max_future(next_poll);
}

void serial_setup(void)
{
	serial0_setup();
	serial1_setup();

	serial_state = SERIAL_IDLE;
	serial_state_changed = false;
	timekeeping_now_timestamp(&serialcpu_last_rx);
}
