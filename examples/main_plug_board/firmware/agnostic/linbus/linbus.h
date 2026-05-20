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

#define LINBUS_BUF_LEN 11u /**< SYN[1] + PID[1] + DAT[LEN<=8] + SUM[1] */
#define LINBUS_SYN self->_buf[0u]		    /**< SYN[1] */
#define LINBUS_PID self->_buf[1u]		    /**< PID[1] */
#define LINBUS_DAT ((uint8_t *)(&self->_buf[2u]))   /**< DAT[LEN<=8] */
#define LINBUS_SUM self->_buf[2u + self->_data_len] /**< SUM[1] */

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
	LINBUS_EVENT_SEND_COMPLETE,

	/** Signal to send break symbol */
	LINBUS_EVENT_RECV_BREAK,

	/** Signal to send binary data */
	LINBUS_EVENT_RECV_DATA,

	/** Signal to wait for IDLE condition */
	LINBUS_EVENT_RECV_IDLE,

	/** Frame has been sent */
	LINBUS_EVENT_RECV_COMPLETE,

	/** Frame has been received with fault */
	LINBUS_EVENT_RECV_FAULT
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

	/** State that sends Parity bits + ID */
	LINBUS_STATE_SEND_PID,

	/** Send actual packet data */
	LINBUS_STATE_SEND_DATA,

	/** Send checksum */
	LINBUS_STATE_SEND_CHECKSUM,

	/** Frame has been sent */
	LINBUS_STATE_SEND_COMPLETE,

	/** Receive break condition */
	LINBUS_STATE_RECV_BREAK,

	/** State that receives sync byte */
	LINBUS_STATE_RECV_SYNC,

	/** State that receives Parity bits + ID */
	LINBUS_STATE_RECV_PID,

	/** Receive actual packet data */
	LINBUS_STATE_RECV_DATA,

	/** Receive checksum */
	LINBUS_STATE_RECV_CHECKSUM,

	/** Frame has been received */
	LINBUS_STATE_RECV_COMPLETE,

	/** Wait for idle condition */
	LINBUS_STATE_RECV_IDLE,

	/** Frame has been received with fault */
	LINBUS_STATE_RECV_FAULT
};

/** Linbus can either wait, transmit, or receive. */
enum linbus_mode {
	LINBUS_MODE_IDLE, /**< The bus passively listens for RX */
	LINBUS_MODE_TX,	  /**< The bus entered into transmission mode */
	LINBUS_MODE_RX /**< The bus received data and entered into RX mode */
};

/** linbus main instance data structure */
struct linbus {
	/* Bus characteristics. */
	uint32_t baud;
	uint32_t err_line;

	uint8_t _event;
	uint8_t _state;
	uint8_t _mode;

	/** We pack SYN, PID, DAT, SUM, into a single buffer.
	 * This makes things little easier if hardware DMA is used */
	uint8_t _buf[LINBUS_BUF_LEN];

	uint8_t _data_len; /**< Payload length */
	uint8_t _data_it;  /**< Iterator */

	uint8_t _tx; /**< Current TX data byte (helper) */
	uint8_t _rx; /**< Current RX data byte (helper) */

	bool _legacy; /**< Legacy, pre 2.1 version mode */
	bool _nack;   /**< Event has not been acknowledged */
	bool _idle;   /**< bus IDLE condition */
	bool debug;
};

/** Initializes linbus main instance data structure */
void linbus_init(struct linbus *self);

/** Queues packet to be sent */
bool linbus_send_frame(struct linbus *self, const uint8_t id,
		       const uint8_t *data, const uint8_t len);

/** Returns tx data (single byte).
 * This is only to be called after LINBUS_EVENT_SEND_DATA occurs.
 * Calling this after any other event is undefined behaviour. */
uint8_t linbus_get_tx_data(const struct linbus *self);

/** Set rx data (single byte).
 * This can be called at any time if there's a RX byte available */
void linbus_set_rx_data(struct linbus *self, const uint8_t rx);

/** Acknowledge linbus event */
void linbus_ack_event(struct linbus *self);

/** Acknowledge carrier on the bus (carrier sensing) */
void linbus_ack_carrier(struct linbus *self);

/** Acknowledge bus idle condition (carrier sensing) */
void linbus_ack_idle(struct linbus *self);

/** Returns event, if any occurs.
 *  May be run in a while loop until returns LINBUS_EVENT_NONE */
uint8_t linbus_step(struct linbus *self);

/** Calculates EXACT frame time on the physical line in microseconds.
 *  Assumed DATA BITS == 8, PARITY BITS = 0, STOP BITS = 1 */
uint32_t linbus_calc_frame_us(const struct linbus *self,
			      const uint8_t	   payload_len);

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
	LINBUS_PID = id | (p0 << 6u) | (p1 << 7u);
}

uint8_t _linbus_calc_checksum(struct linbus *self)
{
	uint8_t i;

	/* Classic uses PID, Legacy (1.x) skips it */
	uint16_t sum = (self->_legacy) ? 0u : LINBUS_PID;

	assert(self->_data_len <= 8u);

	for (i = 0u; i < self->_data_len; i++) {
		sum += LINBUS_DAT[i];

		/* If it overflows 255, add the carry back to the low byte */
		if (sum > 0xFFu) {
			sum -= 0xFFu;
		}
	}

	/* Bitwise NOT of the final 8-bit sum */
	return (uint8_t)~sum;
}

const char *_linbus_get_state_name(const uint8_t state)
{
	const char *name = "UNDEFINED";

	switch (state) {
	case LINBUS_STATE_IDLE:
		name = "IDLE";
		break;
	/* Pseudo state.
	case LINBUS_STATE_EACK:
		name = "EACK";
		break;
	*/
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
	case LINBUS_STATE_SEND_COMPLETE:
		name = "SEND_COMPLETE";
		break;
	case LINBUS_STATE_RECV_BREAK:
		name = "RECV_BREAK";
		break;
	case LINBUS_STATE_RECV_SYNC:
		name = "RECV_SYNC";
		break;
	case LINBUS_STATE_RECV_PID:
		name = "RECV_PID";
		break;
	case LINBUS_STATE_RECV_DATA:
		name = "RECV_DATA";
		break;
	case LINBUS_STATE_RECV_CHECKSUM:
		name = "RECV_CHECKSUM";
		break;
	case LINBUS_STATE_RECV_COMPLETE:
		name = "RECV_COMPLETE";
		break;
	case LINBUS_STATE_RECV_IDLE:
		name = "RECV_IDLE";
		break;
	case LINBUS_STATE_RECV_FAULT:
		name = "RECV_FAULT";
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

void _linbus_fault(struct linbus *self, const uint8_t state, const uint32_t line)
{
	self->err_line = line;
	_linbus_enter_state(self, state);
}

void linbus_init(struct linbus *self)
{
	assert(self);

	self->baud     = 0u;
	self->err_line = 0u;

	self->_event = 0u;
	self->_state = 0u;
	self->_mode  = 0u;

	(void)memset(self->_buf, 0u, LINBUS_BUF_LEN);

	self->_data_len = 0u;
	self->_data_it	= 0u;

	self->_tx = 0u;
	self->_rx = 0u;

	self->_legacy = false;
	self->_idle   = false;
	/* self->_nack   = false; */
	self->debug = false;
}

bool linbus_send_frame(struct linbus *self, const uint8_t id,
		       const uint8_t *data, const uint8_t len)
{
	bool success = false;

	assert(self);

	if (len > 0u) {
		assert(data);
	}

	/* If id is valid and len is valid and state is idle */
	if (self->_state != (uint8_t)LINBUS_STATE_IDLE) {
		LINBUS_LOG(("Invalid state: %s\n",
			    _linbus_get_state_name(self->_state)));
	} else if (id > 0x3Fu) {
		LINBUS_LOG(("Invalid ID: %u\n", id));
	} else if (len > 8u) {
		LINBUS_LOG(("Invalid Len (>8): %u\n", len));
	} else {
		/* SYNC[1] */
		LINBUS_SYN = 0x55u;

		/* PID[1] */
		_linbus_calc_pid(self, id); /* PID */

		/* DATA[L<=8] */
		(void)memcpy(LINBUS_DAT, data, len);
		self->_data_len = len;
		self->_data_it	= 0u;

		/* SUM[1] */
		LINBUS_SUM = _linbus_calc_checksum(self);

		_linbus_enter_state(self, LINBUS_STATE_SEND_BREAK);
		self->_mode = LINBUS_MODE_TX;
		success	    = true;
	}

	return success;
}

uint8_t linbus_get_tx_data(const struct linbus *self)
{
	assert(self);

	return self->_tx;
}

void linbus_set_rx_data(struct linbus *self, const uint8_t rx)
{
	assert(self);

	self->_rx = rx;

	/* TODO wait for main FSM confirmation */
	switch (self->_state) {
	case LINBUS_STATE_RECV_SYNC:
		LINBUS_SYN = self->_rx;
		LINBUS_LOG(("rxsyn: 0x%02X\n", self->_rx));
		_linbus_enter_state(self, LINBUS_STATE_RECV_PID);
		break;

	case LINBUS_STATE_RECV_PID:
		LINBUS_PID = self->_rx;
		LINBUS_LOG(("rxpid: 0x%02X\n", self->_rx));
		_linbus_enter_state(self, LINBUS_STATE_RECV_DATA);
		break;

	case LINBUS_STATE_RECV_DATA:
		LINBUS_DAT[self->_data_it] = self->_rx;
		LINBUS_LOG(("rx[%u]: %c (0x%02X)\n", self->_data_it,
			    self->_rx, self->_rx));

		self->_data_it += 1u;

		if (self->_data_it >= 8u) {
			/* TODO: DATA OVERFLOW */
			self->_data_len = 8u;
			_linbus_enter_state(self, LINBUS_STATE_RECV_CHECKSUM);
		}

		break;

	case LINBUS_STATE_RECV_CHECKSUM: {
		uint8_t sum = _linbus_calc_checksum(self);

		LINBUS_SUM = self->_rx;
		LINBUS_LOG(("rxsum: 0x%02X\n", self->_rx));
		LINBUS_LOG(("expected: 0x%02X\n", sum));

		if (LINBUS_SUM == sum) {
			_linbus_enter_state(self, LINBUS_STATE_RECV_COMPLETE);
		} else {
			/* TODO better fault management */
			_linbus_fault(self, LINBUS_STATE_RECV_FAULT, __LINE__);
		}

		break;
	}

	default:
		break;
	}

	/*if (self->_mode == (uint8_t)LINBUS_MODE_IDLE) {
		_linbus_enter_state(self, LINBUS_STATE_RECV_BREAK);
	}*/

	/* TODO
	 * 1. If we're in IDLE mode, go into RX mode
	 * 2. (OPTIONAL) If we're in TX mode, readback TX bytes and detect
	 *    collisions. Readback must be done before linbus_ack_event.
	 */
}

void linbus_ack_event(struct linbus *self)
{
	assert(self);

	/*self->_nack = false;*/
	self->_event = LINBUS_EVENT_NONE;
}

void linbus_ack_carrier(struct linbus *self)
{
	assert(self);

	/* Go into RX mode if carrier detected while on idle */
	if (self->_mode == (uint8_t)LINBUS_MODE_IDLE) {
		self->_data_len = 0u;
		self->_data_it	= 0u;

		self->_mode = LINBUS_MODE_RX;
		_linbus_enter_state(self, LINBUS_STATE_RECV_BREAK);

		self->_idle = false;
	}
}

void linbus_ack_idle(struct linbus *self)
{
	assert(self);

	/* We need to stop any reception at this point and either:
	 * 1. Make last received byte a checksum
	 * 2. Exit gracefully if already got last byte
	 * 3. Throw a fault if reception is incomplete
	 *
	 * TODO if in rx and fault state - it will always fail
	 * Ensure other states are safely left... */
	if (self->_mode == (uint8_t)LINBUS_MODE_RX) {
		/* If IDLE detected while receiving data, see: 1. && 2. */
		if ((self->_state == (uint8_t)LINBUS_STATE_RECV_DATA) ||
		    (self->_state == (uint8_t)LINBUS_STATE_RECV_CHECKSUM)) {
			self->_data_len = self->_data_it - 1u;

			LINBUS_LOG(("idle detected! data len: %u\n",
				    self->_data_len));

			/* Enter checksum calculation mode manually */
			_linbus_enter_state(self, LINBUS_STATE_RECV_CHECKSUM);
			linbus_set_rx_data(self, self->_rx);
		} else if (self->_state == (uint8_t)LINBUS_STATE_RECV_IDLE) {
			/* We have already received the frame.
			 * Go to normal IDLE state */
			_linbus_enter_state(self, LINBUS_STATE_IDLE);
		} else {
			/* TODO better fault management */
			_linbus_fault(self, LINBUS_EVENT_RECV_FAULT, __LINE__);
		}
	}

	self->_idle = true;
}

uint8_t linbus_step(struct linbus *self)
{
	uint8_t state = self->_state;

	if (self->_event > 0u) {
		state = LINBUS_STATE_EACK;
	}

	switch (state) {
	case LINBUS_STATE_IDLE:
		/* In this state we either waiting for frame transmission
		 * start, or reception from other node */
		self->_mode = LINBUS_MODE_IDLE;
		break;

	case LINBUS_STATE_EACK:
		/* User must acknowledge event here */
		break;

	case LINBUS_STATE_SEND_BREAK:
		self->_event = LINBUS_EVENT_SEND_BREAK;
		_linbus_enter_state(self, LINBUS_STATE_SEND_SYNC);
		break;

	case LINBUS_STATE_SEND_SYNC:
		self->_tx = LINBUS_SYN;
		LINBUS_LOG(("txsyn: 0x%02X\n", self->_tx));
		self->_event = LINBUS_EVENT_SEND_DATA;
		_linbus_enter_state(self, LINBUS_STATE_SEND_PID);
		break;

	case LINBUS_STATE_SEND_PID:
		self->_tx = LINBUS_PID;
		LINBUS_LOG(("txpid: 0x%02X\n", self->_tx));
		self->_event = LINBUS_EVENT_SEND_DATA;
		_linbus_enter_state(self, LINBUS_STATE_SEND_DATA);
		break;

	case LINBUS_STATE_SEND_DATA:
		if (self->_data_it < self->_data_len) {
			self->_tx = LINBUS_DAT[self->_data_it];
			LINBUS_LOG(("tx[%u]: %c (0x%02X)\n", self->_data_it,
				    self->_tx, self->_tx));
			self->_data_it += 1u;
			self->_event	= LINBUS_EVENT_SEND_DATA;
		}

		if (self->_data_it >= self->_data_len) {
			_linbus_enter_state(self, LINBUS_STATE_SEND_CHECKSUM);
		}
		break;

	case LINBUS_STATE_SEND_CHECKSUM:
		self->_tx = LINBUS_SUM;
		LINBUS_LOG(("txsum: 0x%02X\n", self->_tx));
		self->_event = LINBUS_EVENT_SEND_DATA;
		_linbus_enter_state(self, LINBUS_STATE_SEND_COMPLETE);
		break;

	case LINBUS_STATE_SEND_COMPLETE:
		self->_event = LINBUS_EVENT_SEND_COMPLETE;
		_linbus_enter_state(self, LINBUS_STATE_IDLE);
		break;

	case LINBUS_STATE_RECV_BREAK:
		self->_event = LINBUS_EVENT_RECV_BREAK;
		_linbus_enter_state(self, LINBUS_STATE_RECV_SYNC);
		break;

	/* We use linbus_set_rx_data method to parse theese */
	case LINBUS_STATE_RECV_SYNC:
	case LINBUS_STATE_RECV_PID:
	case LINBUS_STATE_RECV_DATA:
	case LINBUS_STATE_RECV_CHECKSUM:
		self->_event = LINBUS_EVENT_RECV_DATA;
		break;

	case LINBUS_STATE_RECV_COMPLETE:
		self->_event = LINBUS_EVENT_RECV_COMPLETE;

		if (!self->_idle) {
			/* If line is not on IDLE */
			_linbus_enter_state(self, LINBUS_STATE_RECV_IDLE);
		} else {
			_linbus_enter_state(self, LINBUS_STATE_IDLE);
		}
		break;

	case LINBUS_STATE_RECV_IDLE:
		self->_event = LINBUS_EVENT_RECV_IDLE;
		break;

	case LINBUS_STATE_RECV_FAULT:
		self->_event = LINBUS_EVENT_RECV_FAULT;
		_linbus_enter_state(self, LINBUS_STATE_IDLE);
		break;

	default:
		break;
	}

	return (state == (uint8_t)LINBUS_STATE_EACK)
		   ? (uint8_t)LINBUS_EVENT_NONE
		   : self->_event;
}

uint32_t linbus_calc_frame_us(const struct linbus *self,
			      const uint8_t	   payload_len)
{
	uint64_t bit_t_us	 = (uint64_t)-1 / self->baud;
	uint64_t full_frame_time = 0u;
	uint8_t	 i;

	/* BREAK condition (LIN) */
	full_frame_time += bit_t_us * 13u;

	/* Stop bit after break (LIN) */
	full_frame_time += bit_t_us * 1u;

	/* SYNC + PID + PAYLOAD + CHECKSUMSUM */
	for (i = 0u; i < (2u + payload_len + 1u); i++) {
		/* Start bit (UART) */
		full_frame_time += bit_t_us * 1u;

		/* BYTE (UART) */
		full_frame_time += bit_t_us * 8u;

		/* stop bit */
		full_frame_time += bit_t_us * 1u;
	}

	/* start bit */
	/*full_frame_time += bit_t_us * 1u;*/

	return full_frame_time / (((uint64_t)-1) / 1000000u);
}

#endif /* LINBUS_IMPLEMENTATION */

#endif /* LINBUS_HEADER_GUARD */
