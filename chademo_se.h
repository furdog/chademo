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
 */

#ifndef   CHADEMO_SE_DEFINITION_HEADER_GUARD
#define   CHADEMO_SE_DEFINITION_HEADER_GUARD

#include <stdbool.h>
#include <stdint.h>

/* Skip certain definitions if EV header is already included */
#if       !defined(CHADEMO_EV_DEFINITION_HEADER_GUARD)
/******************************************************************************
 * _CHADEMO_XX DEFINITIONS (COMMON FOR EV AND SE)
 *****************************************************************************/
/** CAN2.0 simplified frame structure. */
struct chademo_xx_can_frame {
	uint32_t id;	  /**< Frame identifier. */
	uint8_t  len;	  /**< Data length code (0-8). */
	uint8_t  data[8]; /**< Frame data payload. */
};

/** Label for chademo_xx_can_frame */
#define chademo_se_can_frame chademo_xx_can_frame

/** Label for chademo_xx_can_frame */
#define chademo_ev_can_frame chademo_xx_can_frame

/******************************************************************************
 * _CHADEMO_EV DEFINITIONS
 *****************************************************************************/
/** Frame ID's used by vehicle (EV) */
enum _chademo_ev_can_frame_id {
	_CHADEMO_EV_CAN_FRAME_ID_H100 = 0x100u,
	_CHADEMO_EV_CAN_FRAME_ID_H101 = 0x101u,
	_CHADEMO_EV_CAN_FRAME_ID_H102 = 0x102u
};

enum _chademo_ev_field_h100_protocol_number {
	/** Before V0.9 of standard specifications */
	_CHADEMO_EV_FIELD_H109_CONTROL_PROTOCOL_NUMBER_0 = 0u,

	/** V0.9 - V0.9.1 of standard specifications */
	_CHADEMO_EV_FIELD_H109_CONTROL_PROTOCOL_NUMBER_1 = 1u,

	/** V1.0.0 - V1.0.1 of standard specifications */
	_CHADEMO_EV_FIELD_H109_CONTROL_PROTOCOL_NUMBER_2 = 2u
};

enum _chademo_ev_field_h102_fault {
	/** (0: normal, 1: fault) */
	_CHADEMO_EV_FIELD_H102_FAULT_BATTERY_OVERVOLTAGE = 1u,

	/** (0: normal, 1: fault) */
	_CHADEMO_EV_FIELD_H102_FAULT_BATTERY_UNDERVOLTAGE = 2u,

	/** (0: normal, 1: fault) */
	_CHADEMO_EV_FIELD_H102_FAULT_BATTERY_CURRENT_DEVIATION = 4u,

	/** (0: normal, 1: fault) */
	_CHADEMO_EV_FIELD_H102_FAULT_BATTERY_HIGH_TEMPERATURE = 8u,

	/** (0: normal, 1: fault) */
	_CHADEMO_EV_FIELD_H102_FAULT_BATTERY_VOLTAGE_DEVIATION = 16u
};

enum _chademo_ev_field_h102_status {
	/** (0: disabled, 1: enabled) */
	_CHADEMO_EV_FIELD_H102_STATUS_CHARGING_ENABLED = 1u,

	/** (0: “Parking” position, 1: other position) */
	_CHADEMO_EV_FIELD_H102_STATUS_SHIFT_POSITION_NOT_PARKED = 2u,

	/** (0: normal, 1: fault) */
	_CHADEMO_EV_FIELD_H102_STATUS_CHARGING_SYS_FAULT = 4u,

	/** (0: EV contactor close or during welding detection,
	 *   1: EV contactor open or termination of welding detection) */
	_CHADEMO_EV_FIELD_H102_STATUS_CONTACTOR_OPEN = 8u,

	/** (0: No request, 1: Stop request) */
	_CHADEMO_EV_FIELD_H102_STATUS_NORMAL_STOP_REQUEST_BEFORE_CHARGING = 16u
};

/** H100 data structure defined by standard */
struct _chademo_ev_h100
{
	/* uint8_t  reserved; */
	/* uint8_t  reserved; */
	/* uint8_t  reserved; */
	/* uint8_t  reserved; */
	uint16_t max_battery_voltage_V;
	uint8_t	 charged_rate_ref_const; /* 1% / bit, 100% (fixed) = 0x64 */
	/* uint8_t  reserved; */
};

struct _chademo_ev_h101
{
	/* uint8_t  reserved; */
	uint8_t max_charge_time_10s;
	uint8_t max_charge_time_60s;
	uint8_t est_charge_time_60s;
	/* uint8_t  reserved; */
	uint16_t total_cap_of_battery_100wh; /* 0.1 kWh/bit, 0–6553.5 kWh */
};

struct _chademo_ev_h102
{
	uint8_t	 control_protocol_number;
	uint16_t target_battery_voltage;
	uint8_t	 charging_current_request;
	uint8_t	 fault;
	uint8_t	 status;
	uint8_t	 charged_rate;
	/* uint8_t  reserved; */
};

/******************************************************************************
 * _CHADEMO_SE DEFINITIONS
 *****************************************************************************/
/** Frame identifiers used to be sent via chademo SE */
enum _chademo_se_can_frame_id {
	_CHADEMO_SE_CAN_FRAME_ID_H108 = 0x108u,
	_CHADEMO_SE_CAN_FRAME_ID_H109 = 0x109u
};

/** Protocol number within h109 message */
enum _chademo_se_field_h109_protocol_number {
	/** Before V0.9 of standard specifications */
	_CHADEMO_SE_FIELD_H109_CONTROL_PROTOCOL_NUMBER_0 = 0u,

	/** V0.9 - V0.9.1 of standard specifications */
	_CHADEMO_SE_FIELD_H109_CONTROL_PROTOCOL_NUMBER_1 = 1u,

	/** V1.0.0 - V1.0.1 of standard specifications */
	_CHADEMO_SE_FIELD_H109_CONTROL_PROTOCOL_NUMBER_2 = 2u
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
	uint8_t  control_protocol_number;
	uint16_t present_output_voltage_V;
	uint8_t  present_output_current_A;
	/* uint8_t  _reserved; */
	uint8_t  status;
	uint8_t  remaining_charge_time_10s; /* 10s/bit */
	uint8_t  remaining_charge_time_60s; /* 60s/bit */
};
#endif /* !defined(CHADEMO_EV_HEADER__) */

/******************************************************************************
 * CHADEMO_SE VIRTUAL CAN2.0 TX ROUTINE
 *****************************************************************************/
enum _chademo_se_vcan_tx_state {
	_CHADEMO_SE_VCAN_TX_STATE_IDLE,
	_CHADEMO_SE_VCAN_TX_STATE_TRANSMISSION,
	_CHADEMO_SE_VCAN_TX_STATE_DELAY
};

/** Structure that represents CAN2.0 TX */
struct _chademo_se_vcan_tx
{
	uint8_t state;

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
void _chademo_se_vcan_tx_init(struct _chademo_se_vcan_tx *self)
{
	self->state = _CHADEMO_SE_VCAN_TX_STATE_IDLE;

	self->h108.welding_detection_support = false;
	self->h108.avail_output_voltage_V    = 0u;
	self->h108.avail_output_current_A    = 0u;
	self->h108.threshold_voltage_V       = 0u;

	self->h109.control_protocol_number   = 0u;
	self->h109.present_output_voltage_V  = 0u;
	self->h109.present_output_current_A  = 0u;
	self->h109.status                    = 0u;
	self->h109.remaining_charge_time_10s = 0xFFu;
	self->h109.remaining_charge_time_60s = 0xFFu;

	/* struct chademo_se_can_frame frames[2]; */

	self->count    = 0u;
	self->timer_ms = 0u;
}

void _chademo_se_vcan_tx_pack_frames(struct _chademo_se_vcan_tx *self)
{
	struct chademo_se_can_frame *f;

	/* We transmit exactly two frames. */
	self->count = 2u;

	f = &self->frames[0];
	f->id = _CHADEMO_SE_CAN_FRAME_ID_H108;
	f->len = 8u;
	f->data[0] =  self->h108.welding_detection_support;
	f->data[1] = (self->h108.avail_output_voltage_V & 0x00FFu) >> 0u;
	f->data[2] = (self->h108.avail_output_voltage_V & 0xFF00u) >> 8u;
	f->data[3] =  self->h108.avail_output_current_A;
	f->data[4] = (self->h108.threshold_voltage_V & 0x00FFu) >> 0u;
	f->data[5] = (self->h108.threshold_voltage_V & 0xFF00u) >> 8u;
	f->data[6] = 0x00u;
	f->data[7] = 0x00u;

	f = &self->frames[1];
	f->id = _CHADEMO_SE_CAN_FRAME_ID_H109;
	f->len = 8u;
	f->data[0] = self->h109.control_protocol_number;
	f->data[1] = (self->h109.present_output_voltage_V & 0x00FFu) >> 0u;
	f->data[2] = (self->h109.present_output_voltage_V & 0xFF00u) >> 8u;
	f->data[3] = self->h109.present_output_current_A;
	f->data[4] = 0x00u;
	f->data[5] = self->h109.status;
	f->data[6] = self->h109.remaining_charge_time_10s;
	f->data[7] = self->h109.remaining_charge_time_60s;
}

void _chademo_se_vcan_tx_start(struct _chademo_se_vcan_tx *self)
{
	_chademo_se_vcan_tx_init(self);

	self->state = _CHADEMO_SE_VCAN_TX_STATE_TRANSMISSION;
}

void _chademo_se_vcan_tx_stop(struct _chademo_se_vcan_tx *self)
{
	_chademo_se_vcan_tx_init(self);

	self->state = _CHADEMO_SE_VCAN_TX_STATE_IDLE;
}

void _chademo_se_vcan_tx_step(struct _chademo_se_vcan_tx *self,
				     uint32_t delta_time_ms)
{
	/* TODO check for untransmitted frames */

	switch (self->state) {
	case _CHADEMO_SE_VCAN_TX_STATE_TRANSMISSION:
		_chademo_se_vcan_tx_pack_frames(self);

		/* _CHADEMO_SE_VCAN_TX_STATE_DELAY INIT */
		self->state = _CHADEMO_SE_VCAN_TX_STATE_DELAY;
		self->timer_ms = 0u;
		break;

	case _CHADEMO_SE_VCAN_TX_STATE_DELAY:
		self->timer_ms += delta_time_ms;

		/* 100ms delay between messages */
		if (self->timer_ms < 100u) {
			break;
		}

		/* _CHADEMO_SE_VCAN_TX_STATE_TRANSMISSION INIT */
		self->state = _CHADEMO_SE_VCAN_TX_STATE_TRANSMISSION;
		break;

	case _CHADEMO_SE_VCAN_TX_STATE_IDLE:
		/* Do nothing here */
		break;

	default:
		/* Something really gone wrong here
		 * (probably memory corruption or worse)
		 * We must immediately go into termination and never
		 * start again until fault is diagnosed properly.
		 *
		 * TODO */
		 break;
	}
}

/******************************************************************************
 * CHADEMO_SE VIRTUAL CAN2.0 RX ROUTINE
 *****************************************************************************/
enum _chademo_se_vcan_rx_state {
	_CHADEMO_SE_VCAN_RX_STATE_IDLE,
	_CHADEMO_SE_VCAN_RX_STATE_LISTEN
};

/** Structure that represents CAN2.0 RX */
struct _chademo_se_vcan_rx
{
	uint8_t state;

	struct _chademo_ev_h100 h100;
	struct _chademo_ev_h101 h101;
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
void _chademo_se_vcan_rx_init(struct _chademo_se_vcan_rx *self)
{
	self->state = _CHADEMO_SE_VCAN_RX_STATE_IDLE;

	self->h100.max_battery_voltage_V  = 0x00u;
	self->h100.charged_rate_ref_const = 0x00u;

	self->h101.max_charge_time_10s        = 0x00u;
	self->h101.max_charge_time_60s        = 0x00u;
	self->h101.est_charge_time_60s        = 0x00u;
	self->h101.total_cap_of_battery_100wh = 0x00u;

	self->h102.control_protocol_number  = 0x00u;
	self->h102.target_battery_voltage   = 0x00u;
	self->h102.charging_current_request = 0x00u;
	self->h102.fault                    = 0x00u;
	self->h102.status                   = 0x00u;
	self->h102.charged_rate             = 0x00u;

	/* TODO implement properly */
	self->timer_ms   = 0u;
	self->recv_flags = 0u;
	self->has_frames = false;
}

void _chademo_se_vcan_rx_unpack_frame(
				       struct _chademo_se_vcan_rx *self,
				       struct chademo_se_can_frame *f)
{
	bool valid_frame = true;

	switch (f->id) {
	case _CHADEMO_EV_CAN_FRAME_ID_H100:
		self->h100.max_battery_voltage_V  = f->data[4] << 0u;
		self->h100.max_battery_voltage_V |= f->data[5] << 8u;

		self->h100.charged_rate_ref_const = f->data[6];

		self->recv_flags |= (1u << 0u);
		break;

	case _CHADEMO_EV_CAN_FRAME_ID_H101:
		self->h101.max_charge_time_10s = f->data[1];
		self->h101.max_charge_time_60s = f->data[2];
		self->h101.est_charge_time_60s = f->data[3];

		self->h101.total_cap_of_battery_100wh  = f->data[5] << 0u;
		self->h101.total_cap_of_battery_100wh |= f->data[6] << 8u;

		self->recv_flags |= (1u << 1u);
		break;

	case _CHADEMO_EV_CAN_FRAME_ID_H102:
		self->h102.control_protocol_number  = f->data[0];
		self->h102.target_battery_voltage   = f->data[1] << 0u;
		self->h102.target_battery_voltage  |= f->data[2] << 8u;
		self->h102.charging_current_request = f->data[3];
		self->h102.fault                    = f->data[4];
		self->h102.status                   = f->data[5];
		self->h102.charged_rate             = f->data[6];

		self->recv_flags |= (1u << 2u);
		break;

	default:
		valid_frame = false;
		break;
	}

	/* TODO reset timer only if all frames have been get */
	if (valid_frame && (self->recv_flags == ((1u << 3u) - 1u))) {
		self->has_frames = true;
		self->timer_ms   = 0u;
	}
}

void _chademo_se_vcan_rx_start(struct _chademo_se_vcan_rx *self)
{
	/* We won't assume a shit, just reset all the time. */
	_chademo_se_vcan_rx_init(self);

	self->state = _CHADEMO_SE_VCAN_RX_STATE_LISTEN;
}

void _chademo_se_vcan_rx_stop(struct _chademo_se_vcan_rx *self)
{
	/* We won't assume a shit, just reset all the time. */
	_chademo_se_vcan_rx_init(self);

	self->state = _CHADEMO_SE_VCAN_RX_STATE_IDLE;
}

void _chademo_se_vcan_rx_step(struct _chademo_se_vcan_rx *self,
			     uint32_t delta_time_ms)
{
	(void)self;
	(void)delta_time_ms;
	/* TODO check timeouts */
	/* TODO check errors */
}

/******************************************************************************
 * CHADEMO_SE VIRTUAL SENSORS (VOLTAGE, CURRENT, LEAKAGE, ETC)
 *****************************************************************************/
/** Virtual sensor inputs.
 *  Everything from outside hardware should be mapped here. */
struct chademo_se_vsensors {
	/** Measures voltage on EV terminals */
	uint16_t out_terminals_voltage_V;
};

void _chademo_se_vsensors_init(struct chademo_se_vsensors *self)
{
	/* Assume maximum voltage, until other value been providen */
	self->out_terminals_voltage_V = 0xFFFF;
}

/******************************************************************************
 * CHADEMO_SE VIRTUAL POWER SUPPLY UNIT
 *****************************************************************************/
/** Virtual PSU internal state */
enum _chademo_se_vpsu_state {
	_CHADEMO_SE_VPSU_STATE_IDLE,    /**< Do nothing */
	_CHADEMO_SE_VPSU_STATE_BOOT,    /**< Boot (softstart, etc) */
	_CHADEMO_SE_VPSU_STATE_RUNNING, /**< Active current output */
	_CHADEMO_SE_VPSU_STATE_SHUTDOWN /**< Must drop current */
};

/** Virtual PSU fault flags.
 *  User must map these faults FROM external PSU hardware. */
enum chademo_se_vpsu_flags_fault {
	CHADEMO_SE_VPSU_FLAGS_FAULT_OVERVOLTAGE = 1u,
	CHADEMO_SE_VPSU_FLAGS_FAULT_OVERCURRENT = 2u,
	CHADEMO_SE_VPSU_FLAGS_FAULT_OVERTEMP    = 4u,
	CHADEMO_SE_VPSU_FLAGS_FAULT_OTHER       = 8u,

	/** API fault, set by default (indicates user API usage error) */
	CHADEMO_SE_VPSU_FLAGS_FAULT_API         = 16u
};

/** Params that must be set by chademo_se (READ only).
 *  User must read these and map them TO PSU hardware. */
struct chademo_se_vpsu_config {
	uint16_t set_voltage_dc_V; /**< Voltage point set by chademo_se */
	uint8_t  set_current_dc_A; /**< Current point set by chademo_se */
};


/** Params that must be set by external PSU hardware.
 *  User must map theese FROM real PSU hardware. */
struct chademo_se_vpsu_outputs {
	uint16_t voltage_dc_V; /**< Voltage measurements on the PSU side */
	uint8_t  current_dc_A; /**< Current measurements on the PSU side */
};

/** Virtual PSU used internally by chademo_se. */
struct _chademo_se_vpsu {
	uint8_t state;

	struct chademo_se_vpsu_config  config;
	struct chademo_se_vpsu_outputs outputs;

	uint8_t flags_fault;
};

void _chademo_se_vpsu_init(struct _chademo_se_vpsu *self)
{
	self->state = _CHADEMO_SE_VPSU_STATE_IDLE;

	self->config.set_voltage_dc_V = 0u;
	self->config.set_current_dc_A = 0u;

	/** Set maximum by default, to prevent user from ignoring these */
	self->outputs.voltage_dc_V = 0xFFFF;
	self->outputs.current_dc_A = 0xFF;

	/** Set API fault by default. */
	self->flags_fault = CHADEMO_SE_VPSU_FLAGS_FAULT_API;
}

void _chademo_se_vpsu_step(struct _chademo_se_vpsu *self)
{
	/* TODO fsm */
	(void)self;
}

/******************************************************************************
 * CHADEMO_SE VIRTUAL GPIO
 *****************************************************************************/
/** Virtual GPIO inputs.
 *  Everything from outside hardware should be mapped here. */
struct _chademo_se_vgpio_in {
	bool oc_j; /**< Opto-coupler: vehicle charge permission */

	/* Not a part of main CHAdeMO circuit
	 * ( Colors: O - obligatory, R - recomended ): */
	bool bt_emergency; /**< Button (O: red): emergency stop signal */
	bool bt_start;     /**< Button (R: blue): charge start signal */
	bool bt_stop;      /**< Button (R: green): charge stop signal */
};

/** Virtual GPIO outputs.
 *  All the signals here must be mapped to hardware output GPIO's */
struct _chademo_se_vgpio_out {
	bool sw_d1; /**< Switch: charge sequence signal 1 */
	bool sw_d2; /**< Switch: charge sequence signal 2 */
};

/** Virtual GPIO consist of inputs and outputs */
struct chademo_se_vgpio
{
	struct _chademo_se_vgpio_in  in;  /**< Input  virtual GPIO */
	struct _chademo_se_vgpio_out out; /**< Output virtual GPIO */
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
 * CHADEMO_SE MAIN INSTANCE
 *****************************************************************************/
/** Charging control flow states defined by standard */
enum _chademo_se_state_cf {
	_CHADEMO_SE_STATE_CF_AWAIT_CHARGE_START_BUTTON,
	_CHADEMO_SE_STATE_CF_TRANSMIT_CHARGE_START_SIGNAL,
	_CHADEMO_SE_STATE_CF_AWAIT_CAN_RX_AND_START_TX_AFTER,
	_CHADEMO_SE_STATE_CF_PROCESS_INFO_BEFORE_CHARGING,
	_CHADEMO_SE_STATE_CF_LOCK_CHARHING_CONNECTOR,
	_CHADEMO_SE_STATE_CF_CHECK_EV_CONTACTORS_ARE_OPEN,
	_CHADEMO_SE_STATE_CF_INSULATION_TEST_ON_DC_CIRCUIT,
	_CHADEMO_SE_STATE_CF_CHECK_TERMINATION_OF_INSULATION_TEST,
	_CHADEMO_SE_STATE_CF_TRANSMIT_SIGNAL_OF_CHARGER_SETUP_COMPLETION,
	_CHADEMO_SE_STATE_CF_START_CHARGING_CURRENT_OUTPUT
};

/** Events emited by main instance FSM */
enum chademo_se_event {
	CHADEMO_SE_EVENT_NONE,
	CHADEMO_SE_EVENT_CHARGE_START_BUTTON_PRESSED,
	CHADEMO_SE_EVENT_GOT_EV_INITIAL_PARAMS,
	CHADEMO_SE_EVENT_INFO_BEFORE_CHARGING_IS_PROCESSED,
	CHADEMO_SE_EVENT_VEHICLE_CHARGE_PERMISSION,
	CHADEMO_SE_EVENT_EV_CONTACTORS_ARE_OPEN
};

/** Main instance. */
struct chademo_se {
	enum _chademo_se_state_cf _state_cf;

	struct  _chademo_se_vcan_tx  _can_tx;
	struct  _chademo_se_vcan_rx  _can_rx;
	struct   chademo_se_vsensors _sensors;
	struct   chademo_se_vgpio    _gpio;
	struct  _chademo_se_vpsu     _psu;
};

/******************************************************************************
 * CHADEMO_SE MAIN INSTANCE PUBLIC METHODS
 *****************************************************************************/
/** Initializes main instance (must be done before anything else) */
void chademo_se_init(struct chademo_se *self)
{
	self->_state_cf = _CHADEMO_SE_STATE_CF_AWAIT_CHARGE_START_BUTTON;

	_chademo_se_vcan_tx_init(&self->_can_tx);
	_chademo_se_vcan_rx_init(&self->_can_rx);
	_chademo_se_vsensors_init(&self->_sensors);
	_chademo_se_vgpio_init(&self->_gpio);
	_chademo_se_vpsu_init(&self->_psu);
}

/** Reads vgpio inputs and stores result into a destination `dst` */
void chademo_se_get_vgpio_inputs(struct chademo_se *self,
				 struct chademo_se_vgpio *dst)
{
	dst->in = self->_gpio.in;
}

/** Sets vgpio inputs from source `src` */
void chademo_se_set_vgpio_inputs(struct chademo_se *self,
				 struct chademo_se_vgpio *src)
{
	self->_gpio.in = src->in;
}

/** Gets vgpio outputs and stores result into a destination `dst` */
void chademo_se_get_vgpio_outputs(struct chademo_se *self,
				  struct chademo_se_vgpio *dst)
{
	dst->out = self->_gpio.out;
}

/** Sets vsensors inputs from source `src` */
void chademo_se_set_vsensors_inputs(struct chademo_se *self,
				    struct chademo_se_vsensors *src)
{
	self->_sensors = *src;
}

/** CAN2.0 frames from EV must go here.
 *  Returns true if frame has been consumed.
 *  Frame may not be consumed if charger is not in LISTEN mode. */
bool chademo_se_push_vcan_frame(struct chademo_se *self,
				struct chademo_se_can_frame *f)
{
	bool has_consumed_frame = false;

	if (self->_can_rx.state ==
	    (uint8_t)_CHADEMO_SE_VCAN_RX_STATE_LISTEN) {
		_chademo_se_vcan_rx_unpack_frame(&self->_can_rx, f);

		has_consumed_frame = true;
	}

	return has_consumed_frame;
}

/** CAN2.0 frames from charger emited here (popped from internal TX buffer).
 *  Returns true if frame has been returned.
 *  Frame may not be returned if internal buffer is empty. */
bool chademo_se_pop_vcan_frame(struct chademo_se *self,
			       struct chademo_se_can_frame *f)
{
	bool frame_available = false;

	if (self->_can_tx.count > 0u) {
		self->_can_tx.count--;

		*f = self->_can_tx.frames[self->_can_tx.count];

		frame_available = true;
	}

	return frame_available;
}

/** Main instance FSM. Must be called inside main loop. */
enum chademo_se_event chademo_se_step(struct chademo_se *self,
				      uint32_t delta_time_ms)
{
	struct  _chademo_se_vcan_tx *tx = &self->_can_tx;
	struct  _chademo_se_vcan_rx *rx = &self->_can_rx;

	struct  chademo_se_vsensors *sensors = &self->_sensors;
	struct  chademo_se_vgpio    *gpio    = &self->_gpio;

	enum chademo_se_event event = CHADEMO_SE_EVENT_NONE;

	_chademo_se_vcan_rx_step(rx, delta_time_ms);

	/* State machine reflecs chademo charger control flow precisely */
	switch (self->_state_cf) {
	/* "Charge start button" ON */
	case _CHADEMO_SE_STATE_CF_AWAIT_CHARGE_START_BUTTON:
		if (gpio->in.bt_start != true) {
			break;
		}

		event = CHADEMO_SE_EVENT_CHARGE_START_BUTTON_PRESSED;

		/* _CHADEMO_SE_STATE_CF_TRANSMIT_CHARGE_START_SIGNAL INIT */
		self->_state_cf =
			_CHADEMO_SE_STATE_CF_TRANSMIT_CHARGE_START_SIGNAL;

		break;

	/* Transmit charge start signal */
	case _CHADEMO_SE_STATE_CF_TRANSMIT_CHARGE_START_SIGNAL:
		gpio->out.sw_d1 = true; /* CONNECTOR PIN: 5 */

		/* _CHADEMO_SE_STATE_CF_AWAIT_CAN_RX_AND_START_TX_AFTER INIT */
		self->_state_cf =
			_CHADEMO_SE_STATE_CF_AWAIT_CAN_RX_AND_START_TX_AFTER;

		_chademo_se_vcan_rx_start(rx);
		break;

	/* Receive initial vehicle CAN data and
	   transmit charger CAN data in return */
	case _CHADEMO_SE_STATE_CF_AWAIT_CAN_RX_AND_START_TX_AFTER:
		/* TODO timeout */

		/* Await for all RX frames */
		if (!rx->has_frames) {
			break;
		}

		/* Charger params must be set by user after this event! */
		event = CHADEMO_SE_EVENT_GOT_EV_INITIAL_PARAMS;

		/* _CHADEMO_SE_STATE_CF_PROCESS_INFO_BEFORE_CHARGING INIT */
		self->_state_cf =
			_CHADEMO_SE_STATE_CF_PROCESS_INFO_BEFORE_CHARGING;

		_chademo_se_vcan_tx_start(tx);

		break;

	/* Process the information for charge control
	   before charging (Battery compatibility, charging
	   time calculation etc.) */
	case _CHADEMO_SE_STATE_CF_PROCESS_INFO_BEFORE_CHARGING:
		/* See A.7.2.7.2 */

		/* TODO more validation */

		if ((rx->h100.max_battery_voltage_V >
		     tx->h108.avail_output_voltage_V)) {
			    /* TODO terminate */
		}

		/* TODO calculate approximate estimated charge time
		 * based on current and voltage demand and pack kwh */

		event = CHADEMO_SE_EVENT_INFO_BEFORE_CHARGING_IS_PROCESSED;

		/* _CHADEMO_SE_STATE_CF_LOCK_CHARHING_CONNECTOR INIT */
		self->_state_cf = _CHADEMO_SE_STATE_CF_LOCK_CHARHING_CONNECTOR;

		break;

	/* Lock charging connector */
	case _CHADEMO_SE_STATE_CF_LOCK_CHARHING_CONNECTOR:
		/* Wait for vehicle readiness */
		if (gpio->in.oc_j != true) { /* CONNECTOR PIN: 4 */
			break;
		}

		event = CHADEMO_SE_EVENT_VEHICLE_CHARGE_PERMISSION;

		/* _CHADEMO_SE_STATE_CF_CHECK_EV_CONTACTORS_ARE_OPEN INIT */
		self->_state_cf =
			_CHADEMO_SE_STATE_CF_CHECK_EV_CONTACTORS_ARE_OPEN;
		break;

	/* Check that EV contactors are surely opened
	   (Voltage on output terminals is less than 10V.) */
	case _CHADEMO_SE_STATE_CF_CHECK_EV_CONTACTORS_ARE_OPEN:
		if (sensors->out_terminals_voltage_V >= 10u) {
			/* TODO terminate or wait ??? */
			break;
		}

		event = CHADEMO_SE_EVENT_EV_CONTACTORS_ARE_OPEN;

		/* _CHADEMO_SE_STATE_CF_INSULATION_TEST_ON_DC_CIRCUIT INIT */
		self->_state_cf =
			_CHADEMO_SE_STATE_CF_INSULATION_TEST_ON_DC_CIRCUIT;

		break;

	/* Insulation test on output DC circuit */
	case _CHADEMO_SE_STATE_CF_INSULATION_TEST_ON_DC_CIRCUIT:
		/* Insulation test is mostly transparent for the vehicle.
		 * We might skip this step for the future. */
		/* TODO implement */

	   /* _CHADEMO_SE_STATE_CF_CHECK_TERMINATION_OF_INSULATION_TEST INIT */
		self->_state_cf =
		     _CHADEMO_SE_STATE_CF_CHECK_TERMINATION_OF_INSULATION_TEST;
		break;

	/* Check the termination of insulation test
	   (Voltage on output terminals is less than 20V.) */
	case _CHADEMO_SE_STATE_CF_CHECK_TERMINATION_OF_INSULATION_TEST:
		if (sensors->out_terminals_voltage_V >= 20u) {
			/* TODO terminate or wait ??? */
			break;
		}

    /* _CHADEMO_SE_STATE_CF_TRANSMIT_SIGNAL_OF_CHARGER_SETUP_COMPLETION INIT */
		self->_state_cf =
	      _CHADEMO_SE_STATE_CF_TRANSMIT_SIGNAL_OF_CHARGER_SETUP_COMPLETION;

		break;

	/* Transmit the signal of charger setup completion */
	case _CHADEMO_SE_STATE_CF_TRANSMIT_SIGNAL_OF_CHARGER_SETUP_COMPLETION:
		gpio->out.sw_d2 = true; /* CONNECTOR PIN: 10 */

		/* _CHADEMO_SE_STATE_CF_START_CHARGING_CURRENT_OUTPUT INIT */
		self->_state_cf =
		     _CHADEMO_SE_STATE_CF_START_CHARGING_CURRENT_OUTPUT;
		break;

	/* Start charging current output */
	case _CHADEMO_SE_STATE_CF_START_CHARGING_CURRENT_OUTPUT:
		/* TODO implement */
		break;

	default:
		/* Something really gone wrong here
		 * (probably memory corruption or worse)
		 * We must immediately go into termination and never
		 * start again until fault is diagnosed properly.
		 *
		 * TODO */
		break;
	}

	_chademo_se_vcan_tx_step(tx, delta_time_ms);

	return event;
}

#endif /* CHADEMO_SE_DEFINITION_HEADER_GUARD */
