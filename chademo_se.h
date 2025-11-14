/**
 * @file chademo_se.h
 * @brief CHAdeMO power supply equipment (SE) Logic (Hardware-Agnostic)
 *
 * This file contains the software implementation of the CHAdeMO EVSE logic.
 * The design is hardware-agnostic, requiring an external adaptation layer 
 * for hardware interaction.
 *
 * **Reference Specification:**
 * Implemented based on the technical specifications outlined in
 * IEEE Std 2030.1.1™-2015. The specification is not included in this
 * repository due to legal reasons.
 *
 * **Important IP Notice:**
 * This implementation utilizes the CHAdeMO protocol, which is subject to
 * Patents (SEPs) and Trade Secrets held by the CHAdeMO Association.
 * * **Users intending to manufacture or commercially distribute hardware
 * derived from this software must secure the necessary licenses and
 * permissions from the CHAdeMO Association.**
 * * The author claims no ownership over these third-party intellectual
 * property rights and provides this code without warranty against
 * third-party infringement.
 *
 * ```LICENSE
 * Copyright (c) 2025 furdog <https://github.com/furdog>
 *
 * SPDX-License-Identifier: 0BSD
 * ```
 */

#ifndef _CHADEMO_SE_HEADER
#define _CHADEMO_SE_HEADER

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************
 * _CHADEMO_EV DEFINITIONS (EXACTLY SPECIFICATION BASED)
 *****************************************************************************/
/*TODO MAKE IT CORRECT*/
enum _chademo_ev_can_frame_id {
	_CHADEMO_EV_CAN_FRAME_ID_H100 = 0x100u,
	_CHADEMO_EV_CAN_FRAME_ID_H101 = 0x101u
	_CHADEMO_EV_CAN_FRAME_ID_H102 = 0x102u
};

enum _chademo_ev_field_h100_protocol_number {
	/** Before V0.9 of standard specifications */
	_CHADEMO_SE_FIELD_H109_PROTOCOL_NUMBER_0 = 0u,

	/** V0.9 - V0.9.1 of standard specifications */
	_CHADEMO_SE_FIELD_H109_PROTOCOL_NUMBER_1 = 1u,

	/** V1.0.0 - V1.0.1 of standard specifications */
	_CHADEMO_SE_FIELD_H109_PROTOCOL_NUMBER_2 = 2u
};

enum chademo_vehicle_can_error_flags {
	CHADEMO_VEHICLE_ERROR_FLAG_BATT_VOLTAGE_TOO_HIGH = 1,
	CHADEMO_VEHICLE_ERROR_FLAG_BATT_VOLTAGE_TOO_LOW	 = 2,
	CHADEMO_VEHICLE_ERROR_FLAG_CURRENT_DEVIATION	 = 4,
	CHADEMO_VEHICLE_ERROR_FLAG_BATT_TEMP_TOO_HIGH	 = 8,
	CHADEMO_VEHICLE_ERROR_FLAG_VOLTAGE_DEVIATION	 = 16
};

enum chademo_vehicle_status_flags {
	CHADEMO_VEHICLE_STATUS_FLAG_CHARGING_ENABLED		 = 1,
	CHADEMO_VEHICLE_STATUS_FLAG_NOT_PARKED			 = 2,
	CHADEMO_VEHICLE_STATUS_FLAG_SYSTEM_FAULT		 = 4,
	CHADEMO_VEHICLE_STATUS_FLAG_CONTACTOR_OPEN		 = 8,
	CHADEMO_VEHICLE_STATUS_FLAG_REQUEST_STOP_BEFORE_CHARGING = 16
};

/** H100 data structure defined by standard */
struct _chademo_ev_h100
{
	uint16_t max_charge_voltage;
	uint8_t	 soc_100_percent_const;
};

struct _chademo_ev_h101
{
	uint8_t max_charge_time_by_10_s;
	uint8_t max_charge_time_by_60_s;
	uint8_t est_charge_time;
};

struct _chademo_ev_h102
{
	uint8_t	 version;
	uint16_t target_voltage;
	uint8_t	 asking_ampers;
	uint8_t	 errors;
	uint8_t	 status;
	uint8_t	 soc;
};

/******************************************************************************
 * _CHADEMO_SE DEFINITIONS (EXACTLY SPECIFICATION BASED)
 *****************************************************************************/
/** Frame identifiers used to be sent via chademo SE */
enum _chademo_se_can_frame_id {
	_CHADEMO_SE_CAN_FRAME_ID_H108 = 0x108u,
	_CHADEMO_SE_CAN_FRAME_ID_H109 = 0x109u
};

/** Protocol number within h109 message */
enum _chademo_se_field_h109_protocol_number {
	/** Before V0.9 of standard specifications */
	_CHADEMO_SE_FIELD_H109_PROTOCOL_NUMBER_0 = 0u,

	/** V0.9 - V0.9.1 of standard specifications */
	_CHADEMO_SE_FIELD_H109_PROTOCOL_NUMBER_1 = 1u,

	/** V1.0.0 - V1.0.1 of standard specifications */
	_CHADEMO_SE_FIELD_H109_PROTOCOL_NUMBER_2 = 2u
};

/** Status flags within h109 message */
enum _chademo_se_field_h109_status {
	/** (0) Charger status (0: standby, 1: charging) */
	_CHADEMO_SE_FIELD_H109_STATUS_CHARGING = 1u,

	/** (1) Charger malfunction (0: normal, 1: fault) */
	_CHADEMO_SE_FIELD_H109_STATUS_CHARGER_MALFUNCTION = 2u,

	/** (2) Charging connector lock (0: open, 1: locked) */
	_CHADEMO_SE_FIELD_H109_STATUS_CONNECTOR_LOCKED = 4u,

	/** (3) Battery incompatibility (0: compatible, 1: incompatible) */
	_CHADEMO_SE_FIELD_H109_STATUS_INCOMPATIBLE = 8u,

	/** (4) Charging system malfunction (0: normal, 1: malfunction) */
	_CHADEMO_SE_FIELD_H109_STATUS_SYS_MALFUNCTION = 16u,

	/** (5) Charging stop control
	 *  (0: operating, 1: stopped or stop charging) */
	_CHADEMO_SE_FIELD_H109_STATUS_STOP_CONTROL = 32u
};

/** H108 data structure defined by standard */
struct _chademo_se_h108
{
	bool     welding_detection_support;
	uint16_t avail_output_voltage_V;
	uint8_t  avail_output_current_A;
	uint16_t threshold_voltage_V;
	/* uint8_t  reserved; */
	/* uint8_t  reserved; */
};

/** H109 data structure defined by standard */
struct _chademo_se_h109
{
	/* Transported via h109 */
	uint8_t  protocol_number;
	uint16_t present_output_voltage_V;
	uint8_t  present_output_current_A;
	/* uint8_t  _reserved; */
	uint8_t  status;
	uint8_t  remaining_charge_time_x10_s;
	uint8_t  remaining_charge_time_min;
};

/** Charging control flow states defined by standard */
enum _chademo_se_state_cf {
	_CHADEMO_SE_STATE_CF_AWAIT_CHARGE_START_BUTTON,
	_CHADEMO_SE_STATE_CF_TRANSMIT_CHARGE_START_SIGNAL,
	_CHADEMO_SE_STATE_CF_AWAIT_CAN_RX_AND_START_TX_AFTER
};

/******************************************************************************
 * CHADEMO_SE VIRTUAL GPIO (EXACTLY SPECIFICATION BASED)
 *****************************************************************************/
/** Virtual GPIO inputs.
 *  Everything from outside hardware should be mapped here. */
struct chademo_se_vgpio_in {
	bool oc_j; /**< Opto-coupler: vehicle charge permission */

	/* Not a part of main CHAdeMO circuit
	 * ( Colors: O - obligatory, R - recomended ): */
	bool bt_emergency; /**< Button (O: red): emergency stop signal */
	bool bt_start;     /**< Button (R: blue): charge start signal */
	bool bt_stop;      /**< Button (R: green): charge stop signal */
};

/** Virtual GPIO outputs.
 *  All the signals here must be mapped to hardware output GPIO's */
struct chademo_se_vgpio_out {
	bool sw_d1; /**< Switch: charge sequence signal 1 */
	bool sw_d2; /**< Switch: charge sequence signal 2 */
};

/** Virtual GPIO consist of inputs and outputs */
struct chademo_se_vgpio
{
	struct chademo_se_vgpio_in  in;  /**< Input  virtual GPIO */
	struct chademo_se_vgpio_out out; /**< Output virtual GPIO */
};

void _chademo_se_vgpio_init(struct chademo_se_vgpio *self)
{
	self->in.oc_j = false;

	self->in.bt_emergency = false;
	self->in.bt_start     = false;
	self->in.bt_stop      = false;

	self->out.sw_d1 = false;
	self->out.sw_d2 = false;
}

/******************************************************************************
 * CHADEMO_SE CAN2.0 FRAME (IMPLEMENTATION SPECIFIC)
 *****************************************************************************/
/** CAN2.0 simplified frame structure, specifically for chademo SE. */
struct chademo_se_can_frame {
	uint32_t id;	  /**< Frame identifier. */
	uint8_t  len;	  /**< Data length code (0-8). */
	uint8_t  data[8]; /**< Frame data payload. */
};

/******************************************************************************
 * CHADEMO_SE CAN2.0 TX ROUTINE (IMPLEMENTATION SPECIFIC)
 *****************************************************************************/
/** Structure that represents CAN2.0 TX */
struct _chademo_se_can_tx
{
	/* TX-side messages */
	struct _chademo_se_h108 h108;
	struct _chademo_se_h109 h109;

	/** Array to hold frames to be sent. TODO TX-side messages is enough */
	struct chademo_se_can_frame frames[2];

	/** The number of valid frames currently in the array. */
	uint8_t count;

	/** Timer used for frame transmission scheduling. */
	uint32_t timer_ms;
};

/**
 * @brief Initializes the CAN2.0 tx structure.
 * @param self Pointer to the tx instance.
 */
void _chademo_se_can_tx_init(struct _chademo_se_can_tx *self)
{
	self->h108.welding_detection_support = false;
	self->h108.avail_output_voltage_V    = 0u;
	self->h108.avail_output_current_A    = 0u;
	self->h108.threshold_voltage_V       = 0u;

	self->h109.protocol_number             = 0u;
	self->h109.present_output_voltage_V    = 0u;
	self->h109.present_output_current_A    = 0u;
	self->h109.status                      = 0u;
	self->h109.remaining_charge_time_x10_s = 0xFFu;
	self->h109.remaining_charge_time_min   = 0xFFu;

	/* struct chademo_se_can_frame frames[2]; */

	self->count    = 0u;
	self->timer_ms = 0u;
}

/******************************************************************************
 * CHADEMO_SE CAN2.0 RX ROUTINE (IMPLEMENTATION SPECIFIC)
 *****************************************************************************/
/*TODO MAKE IT CORRECT*/
/** Structure that represents CAN2.0 RX */
struct _chademo_se_can_rx
{
	struct _chademo_ev_h100 h100;
	struct _chademo_ev_h102 h101;
	struct _chademo_ev_h102 h102;

	/** Timer used for monitoring frame reception timeouts. */
	uint32_t timer_ms;

	/** Received frames (bits flagged) */
	uint8_t recv_flags;

	/** Flag indicating if new frames have been received in the step. */
	bool has_frames;
};

/**
 * @brief Initializes the CAN2.0 rx structure.
 * @param self Pointer to the rx instance.
 */
void _chademo_se_can_rx_init(struct _chademo_se_can_rx *self)
{
	self->h100.max_charge_voltage
	self->h100.soc_100_percent_const

	self->h101.max_charge_time_by_10_s
	self->h101.max_charge_time_by_60_s
	self->h101.est_charge_time

	self->h102.version
	self->h102.target_voltage
	self->h102.asking_ampers
	self->h102.errors
	self->h102.status
	self->h102.soc

	/* TODO implement properly */
	self->timer_ms   = 0u;
	self->recv_flags = 0u;
	self->has_frames = false;
}

void _chademo_se_can_rx_put_frame(struct _chademo_se_can_rx *self,
				  struct chademo_se_can_frame *frame)
{
	switch (frame->id) {
	case _CHADEMO_EV_CAN_FRAME_ID_H100:
		frame->data[4] =
		    (vehicle_vars->max_charge_voltage & 0x00FF) >> 0;
		frame->data[5] =
		    (vehicle_vars->max_charge_voltage & 0xFF00) >> 8;
		frame->data[6] = vehicle_vars->soc_100_percent_const;
		break;

	case _CHADEMO_EV_CAN_FRAME_ID_H101:
		frame->data[1] = vehicle_vars->max_charge_time_by_10_s;
		frame->data[2] = vehicle_vars->max_charge_time_by_60_s;
		frame->data[3] = vehicle_vars->est_charge_time;
		break;

	case _CHADEMO_EV_CAN_FRAME_ID_H102:
		frame->data[0] = vehicle_vars->version;
		frame->data[1] = (vehicle_vars->target_voltage & 0x00FF) >> 0;
		frame->data[2] = (vehicle_vars->target_voltage & 0xFF00) >> 8;
		frame->data[3] = vehicle_vars->asking_ampers;
		frame->data[4] = vehicle_vars->errors;
		frame->data[5] = vehicle_vars->status;
		frame->data[6] = vehicle_vars->soc;
		break;

	default:
		break;
	}
}

/******************************************************************************
 * CHADEMO_SE CAN2.0 LOGICAL DEVICE (IMPLEMENTATION SPECIFIC)
 *****************************************************************************/
/** Communication device (CAN2.0) logical representation */
struct _chademo_se_can_dev
{
	struct _chademo_se_can_tx tx;
	struct _chademo_se_can_rx rx;
};

void _chademo_se_can_dev_init(struct _chademo_se_can_dev *self)
{
	_chademo_se_can_tx_init(&self->tx);
	_chademo_se_can_rx_init(&self->rx);
}

/******************************************************************************
 * CHADEMO_SE MAIN INSTANCE (IMPLEMENTATION SPECIFIC)
 *****************************************************************************/
/** Events emited by main instance FSM */
enum chademo_se_event {
	CHADEMO_SE_EVENT_NONE,
	CHADEMO_SE_EVENT_CHARGE_START_BUTTON_PRESSED
};

/** Main instance. */
struct chademo_se {
	enum _chademo_se_state_cf _state_cf;

	struct  chademo_se_vgpio   _vgpio;
	struct _chademo_se_can_dev _can;
};

/******************************************************************************
 * CHADEMO_SE MAIN INSTANCE PRIVATE METHODS
 *****************************************************************************/
/* TODO */

/******************************************************************************
 * CHADEMO_SE MAIN INSTANCE PUBLIC METHODS
 *****************************************************************************/
/** Initializes main instance (must be done before anything else) */
void chademo_se_init(struct chademo_se *self)
{
	self->_state_cf = _CHADEMO_SE_STATE_CF_AWAIT_CHARGE_START_BUTTON;

	_chademo_se_vgpio_init(&self->_vgpio);
	_chademo_se_can_dev_init(&self->_can);
}

/** Sets VGPIO (only inputs are overriden) */
void chademo_se_set_vgpio(struct chademo_se *self,
			  struct chademo_se_vgpio *vgpio)
{
	self->_vgpio.in = vgpio->in;
}

/** Gets VGPIO (both inputs and outputs are overriden) */
void chademo_se_get_vgpio(struct chademo_se *self,
			  struct chademo_se_vgpio *vgpio)
{
	*vgpio = self->_vgpio;
}

bool chademo_se_put_rx_frame(struct chademo_se *self,
			     struct chademo_se_can_frame *f)
{
	/* TODO */
	(void)self;
	(void)f;

	return false;
}

bool chademo_se_get_tx_frame(struct chademo_se *self,
			     struct chademo_se_can_frame *f)
{
	struct _chademo_se_can_dev *can = &self->_can;

	bool frame_available = false;

	if (can->tx.count > 0u) {
		can->tx.count--;

		*f = can->tx.frames[can->tx.count];

		frame_available = true;
	}

	return frame_available;
}

/** Main instance FSM */
enum chademo_se_event chademo_se_step(struct chademo_se *self,
				      uint32_t delta_time_ms)
{
	enum chademo_se_event event = CHADEMO_SE_EVENT_NONE;

	(void)delta_time_ms; /* TODO remove */

	switch (self->_state_cf) {
	case _CHADEMO_SE_STATE_CF_AWAIT_CHARGE_START_BUTTON:
		if (self->_vgpio.in.bt_start != true) {
			break;
		}

		event = CHADEMO_SE_EVENT_CHARGE_START_BUTTON_PRESSED;

		/* _CHADEMO_SE_STATE_CF_TRANSMIT_CHARGE_START_SIGNAL INIT */
		self->_state_cf =
			_CHADEMO_SE_STATE_CF_TRANSMIT_CHARGE_START_SIGNAL;

		break;

	case _CHADEMO_SE_STATE_CF_TRANSMIT_CHARGE_START_SIGNAL:
		self->_vgpio.out.sw_d1 = true;

		/* _CHADEMO_SE_STATE_CF_AWAIT_CAN_RX_AND_START_TX_AFTER INIT */
		self->_state_cf =
			_CHADEMO_SE_STATE_CF_AWAIT_CAN_RX_AND_START_TX_AFTER;

		break;

	case _CHADEMO_SE_STATE_CF_AWAIT_CAN_RX_AND_START_TX_AFTER:
		/* TODO */
		break;

	default:
		break;
	}

	return event;
}

#endif  _CHADEMO_SE_HEADER
