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

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/** linbus fsm emited event */
enum linbus_event { LINBUS_EVENT_NONE, LINBUS_EVENT_SEND_BREAK };

/** linbus internal states */
enum linbus_state { LINBUS_STATE_IDLE, LINBUS_STATE_SEND_BREAK };

/** linbus main instance data structure */
struct linbus {
	uint8_t _state;

	uint8_t _pid;

	uint8_t _data[8u];
	uint8_t _data_len;

	uint8_t _checksum;

	bool _legacy; /**< Legacy, pre 2.1 version mode */
};

/** Initializes linbus main instance data structure */
void linbus_init(struct linbus *self);

/** Queues packet to be sent */
bool linbus_queue_packet(struct linbus *self, const uint8_t pid,
			 const uint8_t *data, const uint8_t len);

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
	uint8_t p0 = ((id >> 0) ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 0x01u;
	uint8_t p1 =
	    ~(((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5)) & 0x01u);

	/* Combine ID and Parity bits into the final PID */
	self->_pid = id | (p0 << 6u) | (p1 << 7u);
}

void _linbus_calc_checksum(struct linbus *self)
{
	uint8_t i;

	/* Classic uses PID, Legacy (1.x) skips it */
	uint16_t sum = (self->_legacy) ? self->_pid : 0u;

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

void linbus_init(struct linbus *self)
{
	assert(self);

	self->_state = 0u;

	self->_pid = 0u;

	(void)memset(self->_data, 0u, 8u);
	self->_data_len = 0u;

	self->_checksum = 0u;

	self->_legacy = false;
}

bool linbus_queue_packet(struct linbus *self, const uint8_t id,
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

		self->_state = LINBUS_STATE_SEND_BREAK;

		success = true;
	}

	return success;
}

uint8_t linbus_step(struct linbus *self)
{
	uint8_t ev = LINBUS_EVENT_NONE;

	switch (self->_state) {
	case LINBUS_STATE_IDLE:
		/* Wait for external events */
		break;

	case LINBUS_STATE_SEND_BREAK:
		ev = LINBUS_EVENT_SEND_BREAK;
		break;

	default:
		break;
	}

	return ev;
}

#endif /* LINBUS_IMPLEMENTATION */

#endif /* LINBUS_HEADER_GUARD */
