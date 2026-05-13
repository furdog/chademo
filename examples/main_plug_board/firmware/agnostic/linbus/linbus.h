/**
 * @file linbus.h
 * @brief P2P LIN bus protocol implementation (Hardware-Agnostic)
 *
 * This file contains the software implementation of the P2P variation of
 * LIN 2.1 protocol. This variation is not official.
 * The design is hardware-agnostic, requiring an external adaptation layer
 * for hardware interaction.
 *
 * **Conventions:**
 * C89, Linux kernel style, MISRA, rule of 10, No hardware specific code,
 * only generic C and some binding layer. Be extra specific about types.
 *
 * Scientific units where posible at end of the names, for example:
 * - timer_10s (timer_10s has a resolution of 10s per bit)
 * - power_150w (power 150W per bit or 0.15kw per bit)
 *
 * Keep variables without units if they're unknown or not specified or hard
 * to define with short notation.
 *
 * ```LICENSE
 * Copyright (c) 2025 furdog <https://github.com/furdog>
 *
 * SPDX-License-Identifier: 0BSD
 * ```
 *
 * Be free, be wise and take care of yourself!
 * With best wishes and respect, furdog
 */

#ifndef LINBUS_HEADER_GUARD
#define LINBUS_HEADER_GUARD

#ifndef LINBUS_LOG
/** Logging macro. Must be defined elsewhere */
#define LINBUS_LOG(e)
#endif /* LINBUS_LOG */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/** linbus fsm emited event */
enum linbus_event {
	/** No events */
	LINBUS_EVENT_NONE,

	/** Signal to send break symbol */
	LINBUS_EVENT_SEND_BREAK,

	/** Signal to send binary data */
	LINBUS_EVENT_SEND_DATA,

	/** Frame has been sent */
	LINBUS_EVENT_FRAME_SENT
};

/** linbus internal states */
enum linbus_state {
	/** Not doing anything */
	LINBUS_STATE_IDLE,

	/** Wait for event acknowledge */
	LINBUS_STATE_EACK,

	/** State that sends break event */
	LINBUS_STATE_SEND_BREAK,

	/** State that sends sync byte */
	LINBUS_STATE_SEND_SYNC,

	/** State that sends sync Parity bits + ID */
	LINBUS_STATE_SEND_PID,

	/** Send actual packet data */
	LINBUS_STATE_SEND_DATA,

	/** Send checksum */
	LINBUS_STATE_SEND_CHECKSUM,

	/** Frame has been sent */
	LINBUS_STATE_FRAME_SENT
};

/** linbus main instance data structure */
struct linbus {
	uint8_t _state;

	uint8_t _pid;

	uint8_t _data[8u];
	uint8_t _data_len;
	uint8_t _data_it; /**< Iterator */

	uint8_t _tx; /**< Current TX data byte */
	uint8_t _rx; /**< Current RX data byte */

	uint8_t _checksum;

	bool _legacy; /**< Legacy, pre 2.1 version mode */
	bool _nack;   /**< Event has not been acknowledged */
};

/** Initializes linbus main instance data structure */
void linbus_init(struct linbus *self);

/** Queues packet to be sent */
bool linbus_queue_frame(struct linbus *self, const uint8_t id,
			const uint8_t *data, const uint8_t len);

/** Returns tx data (single byte).
 * This is only to be called after LINBUS_EVENT_SEND_DATA occurs.
 * Calling this after any other event is undefined behaviour. */
uint8_t linbus_get_tx_data(struct linbus *self);

/** Acknowledge linbus event */
void linbus_ack_event(struct linbus *self) { self->_nack = false; }

/** Returns event, if any occurs.
 *  May be run in a while loop until returns LINBUS_EVENT_NONE */
uint8_t linbus_step(struct linbus *self);

#ifdef LINBUS_IMPLEMENTATION
/* 00111111b
 * ||\    /
 * || \  /
 * ||  ID (0-5)
 * ||
 * Parity (6-7) */
void _linbus_calc_pid(struct linbus *self, const uint8_t id)
{
	/* Calculate parity bits */
	uint8_t p0 = ((id >> 0u) ^ (id >> 1u) ^ (id >> 2u) ^ (id >> 4u)) & 1u;
	uint8_t p1 =
	    (((id >> 1u) ^ (id >> 3u) ^ (id >> 4u) ^ (id >> 5u)) & 1u) ^ 1u;

	/* Combine ID and Parity bits into the final PID */
	self->_pid = id | (p0 << 6u) | (p1 << 7u);
}

void _linbus_calc_checksum(struct linbus *self)
{
	uint8_t i;

	/* Classic uses PID, Legacy (1.x) skips it */
	uint16_t sum = (self->_legacy) ? 0u : self->_pid;

	assert(self->_data_len <= 8u);

	for (i = 0u; i < self->_data_len; i++) {
		sum += self->_data[i];

		/* If it overflows 255, add the carry back to the low byte */
		if (sum > 0xFFu) {
			sum -= 0xFFu;
		}
	}

	/* Bitwise NOT of the final 8-bit sum */
	self->_checksum = (uint8_t)~sum;
}

const char *_linbus_get_state_name(const uint8_t state)
{
	const char *name = "UNDEFINED";

	switch (state) {
	case LINBUS_STATE_IDLE:
		name = "IDLE";
		break;
	case LINBUS_STATE_EACK:
		name = "EACK";
		break;
	case LINBUS_STATE_SEND_BREAK:
		name = "SEND_BREAK";
		break;
	case LINBUS_STATE_SEND_SYNC:
		name = "SEND_SYNC";
		break;
	case LINBUS_STATE_SEND_PID:
		name = "SEND_PID";
		break;
	case LINBUS_STATE_SEND_DATA:
		name = "SEND_DATA";
		break;
	case LINBUS_STATE_SEND_CHECKSUM:
		name = "SEND_CHECKSUM";
		break;
	case LINBUS_STATE_FRAME_SENT:
		name = "FRAME_SENT";
		break;
	default:
		break;
	}

	return name;
}

void _linbus_enter_state(struct linbus *self, const uint8_t state)
{
	const char *from = _linbus_get_state_name(self->_state);
	const char *to	 = _linbus_get_state_name(state);

	(void)from;
	(void)to;

	LINBUS_LOG(("state: %s -> %s\n", from, to));

	self->_state = state;
}

void linbus_init(struct linbus *self)
{
	assert(self);

	self->_state = 0u;

	self->_pid = 0u;

	(void)memset(self->_data, 0u, 8u);
	self->_data_len = 0u;
	self->_data_it	= 0u;

	self->_tx = 0u;
	self->_rx = 0u;

	self->_checksum = 0u;

	self->_legacy = false;
	self->_nack   = false;
}

bool linbus_queue_frame(struct linbus *self, const uint8_t id,
			const uint8_t *data, const uint8_t len)
{
	bool success = false;

	assert(self);

	if (len > 0u) {
		assert(data);
	}

	/* If id is valid and len is valid and state is idle */
	if ((id <= 0x3Fu) && (len <= 8u) &&
	    (self->_state == (uint8_t)LINBUS_STATE_IDLE)) {
		(void)memcpy(self->_data, data, len);
		self->_data_len = len;

		_linbus_calc_pid(self, id);
		_linbus_calc_checksum(self);

		_linbus_enter_state(self, LINBUS_STATE_SEND_BREAK);

		success = true;
	}

	return success;
}

uint8_t linbus_get_tx_data(struct linbus *self) { return self->_tx; }

uint8_t linbus_step(struct linbus *self)
{
	uint8_t ev    = LINBUS_EVENT_NONE;
	uint8_t state = self->_state;

	if (self->_nack == true) {
		state = LINBUS_STATE_EACK;
	}

	switch (state) {
	case LINBUS_STATE_IDLE:
		/* Wait for external events */
		break;

	case LINBUS_STATE_EACK:
		/* User must acknowledge event here */
		break;

	case LINBUS_STATE_SEND_BREAK:
		ev = LINBUS_EVENT_SEND_BREAK;
		_linbus_enter_state(self, LINBUS_STATE_SEND_SYNC);
		break;

	case LINBUS_STATE_SEND_SYNC:
		self->_tx = 0x55u;
		LINBUS_LOG(("sync: 0x%02X\n", self->_tx));
		ev = LINBUS_EVENT_SEND_DATA;
		_linbus_enter_state(self, LINBUS_STATE_SEND_PID);
		break;

	case LINBUS_STATE_SEND_PID:
		self->_tx = self->_pid;
		LINBUS_LOG(("pid: 0x%02X\n", self->_tx));
		ev = LINBUS_EVENT_SEND_DATA;
		_linbus_enter_state(self, LINBUS_STATE_SEND_DATA);
		break;

	case LINBUS_STATE_SEND_DATA:
		if (self->_data_it < self->_data_len) {
			self->_tx = self->_data[self->_data_it];
			LINBUS_LOG(("data[%u]: %c (0x%02X)\n", self->_data_it,
				    self->_tx, self->_tx));
			self->_data_it += 1u;
			ev		= LINBUS_EVENT_SEND_DATA;
		}

		if (self->_data_it >= self->_data_len) {
			_linbus_enter_state(self, LINBUS_STATE_SEND_CHECKSUM);
		}

		break;

	case LINBUS_STATE_SEND_CHECKSUM:
		self->_tx = self->_checksum;
		LINBUS_LOG(("sum: 0x%02X\n", self->_tx));
		ev = LINBUS_EVENT_SEND_DATA;
		_linbus_enter_state(self, LINBUS_STATE_FRAME_SENT);
		break;

	case LINBUS_STATE_FRAME_SENT:
		_linbus_enter_state(self, LINBUS_STATE_IDLE);
		ev = LINBUS_EVENT_FRAME_SENT;
		break;

	default:
		break;
	}

	if (ev > (uint8_t)LINBUS_EVENT_NONE) {
		self->_nack = true;
	}

	return ev;
}

#endif /* LINBUS_IMPLEMENTATION */

#endif /* LINBUS_HEADER_GUARD */
