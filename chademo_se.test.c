/**
 * @file chademo_se.test.c
 * @brief CHAdeMO power supply equipment (SE) Software tests
 *
 * This file provides unit testing of the chademo_se.h
 *
 * The tests are designed to be as simple as possible and
 * designed with Test Driven Development (TDD) aproach.
 *
 * ```LICENSE
 * Copyright (c) 2025 furdog <https://github.com/furdog>
 *
 * SPDX-License-Identifier: 0BSD
 * ```
 */

#include "chademo_se.h"

#include <assert.h>

/** Main chademo SE instance we will perform test on */
struct chademo_se chse_main;

/** _CHADEMO_SE_STATE_CF_AWAIT_CHARGE_START_BUTTON */
void chademo_se_test_charge_start_button_pressed(struct chademo_se *self)
{
	struct chademo_se_vgpio vgpio;

	chademo_se_get_vgpio_inputs(self, &vgpio);

	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);
	vgpio.in.bt_start = true;

	chademo_se_set_vgpio_inputs(self, &vgpio);
	assert(chademo_se_step(self, 0) ==
		CHADEMO_SE_EVENT_CHARGE_START_BUTTON_PRESSED);
}

/** _CHADEMO_SE_STATE_CF_TRANSMIT_CHARGE_START_SIGNAL */
void chademo_se_test_charge_start_signal(struct chademo_se *self)
{
	struct chademo_se_vgpio vgpio;

	chademo_se_get_vgpio_outputs(self, &vgpio);

	assert(vgpio.out.sw_d1 == false);
	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);

	chademo_se_get_vgpio_outputs(self, &vgpio);

	assert(vgpio.out.sw_d1 == true);
}

/** _CHADEMO_SE_STATE_CF_AWAIT_CAN_RX_AND_START_TX_AFTER */
void chademo_se_test_rx_from_ev_and_tx_after(struct chademo_se *self)
{
	/* TX frame sent from charger */
	struct chademo_se_can_frame tx;

	/* We receive 3 frames from vehicle */
	struct chademo_se_can_frame rx[3] = {
		{0x100u, 8u, {0x00u, 0x00u, 0x00u, 0x00u,
			      0x00u, 0x00u, 0x00u, 0x00u}},
		{0x101u, 8u, {0x00u, 0x00u, 0x00u, 0x00u,
			      0x00u, 0x00u, 0x00u, 0x00u}},
		{0x102u, 8u, {0x00u, 0x00u, 0x00u, 0x00u,
			      0x00u, 0x00u, 0x00u, 0x00u}}
	};

	chademo_se_push_vcan_frame(self, &rx[0]);
	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);

	chademo_se_push_vcan_frame(self, &rx[1]);
	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);

	assert(chademo_se_pop_vcan_frame(self, &tx) == false);

	chademo_se_push_vcan_frame(self, &rx[2]);
	assert(chademo_se_step(self, 0) ==
	       CHADEMO_SE_EVENT_GOT_EV_INITIAL_PARAMS);

	/* Two frames must be emited by charger */
	assert(chademo_se_pop_vcan_frame(self, &tx) == true);
	assert(chademo_se_pop_vcan_frame(self, &tx) == true);
	assert(chademo_se_pop_vcan_frame(self, &tx) == false);
}

/** _CHADEMO_SE_STATE_CF_PROCESS_INFO_BEFORE_CHARGING */
void chademo_se_test_process_info_before_charging(struct chademo_se *self)
{
	assert(chademo_se_step(self, 0) ==
		CHADEMO_SE_EVENT_INFO_BEFORE_CHARGING_IS_PROCESSED);
}

/** _CHADEMO_SE_STATE_CF_LOCK_CHARHING_CONNECTOR */
void chademo_se_test_ev_charge_allowed(struct chademo_se *self)
{
	struct chademo_se_vgpio vgpio;

	chademo_se_get_vgpio_inputs(self, &vgpio);

	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);

	vgpio.in.oc_j = true;

	chademo_se_set_vgpio_inputs(self, &vgpio);

	assert(chademo_se_step(self, 0) ==
		CHADEMO_SE_EVENT_VEHICLE_CHARGE_PERMISSION);
}

/** _CHADEMO_SE_STATE_CF_CHECK_EV_CONTACTORS_ARE_OPEN */
void chademo_se_test_check_ev_contactors_are_open(struct chademo_se *self)
{
	struct chademo_se_vsensors sensors;

	sensors.out_terminals_voltage_V = 0u;

	chademo_se_set_vsensors_inputs(self, &sensors);

	assert(chademo_se_step(self, 0) ==
		CHADEMO_SE_EVENT_EV_CONTACTORS_ARE_OPEN);	
}

/** _CHADEMO_SE_STATE_CF_INSULATION_TEST_ON_DC_CIRCUIT */
void chademo_se_test_insulation_on_dc_circuit(struct chademo_se *self)
{
	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);
}

/** _CHADEMO_SE_STATE_CF_CHECK_TERMINATION_OF_INSULATION_TEST */
void chademo_se_test_termination_of_insulation_test(struct chademo_se *self)
{
	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);
}

/** _CHADEMO_SE_STATE_CF_TRANSMIT_SIGNAL_OF_CHARGER_SETUP_COMPLETION */
void chademo_se_test_check_charger_setup_completion(struct chademo_se *self)
{
	struct chademo_se_vgpio vgpio;

	chademo_se_get_vgpio_outputs(self, &vgpio);
	assert(vgpio.out.sw_d2 == false);

	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);

	chademo_se_get_vgpio_outputs(self, &vgpio);
	assert(vgpio.out.sw_d2 == true);

}

/** Tests all conditions sequentially */
void chademo_se_test_normal_run(struct chademo_se *self)
{
	chademo_se_init(self);	

	chademo_se_test_charge_start_button_pressed(self);
	chademo_se_test_charge_start_signal(self);
	chademo_se_test_rx_from_ev_and_tx_after(self);
	chademo_se_test_process_info_before_charging(self);
	chademo_se_test_ev_charge_allowed(self);
	chademo_se_test_check_ev_contactors_are_open(self);
	chademo_se_test_insulation_on_dc_circuit(self);
	chademo_se_test_termination_of_insulation_test(self);
	chademo_se_test_check_charger_setup_completion(self);
}

/** Runs all tests */
int main()
{
	chademo_se_test_normal_run(&chse_main);

	return 0;
}
