#include <stdio.h>

/* TODO test interactive mode */

#define DBG_SELF_TEST_LOG_IMPL
#define DBG_SELF_TEST_LOG(x)                                                  \
	printf x;                                                             \
	fflush(0)
#include "self_test.h"

struct dbg_self_test st;

struct dbg_self_test_din_desc  din[5u];
struct dbg_self_test_dout_desc dout[5u];

/*
#define onboard_led_Pin GPIO_PIN_13
#define onboard_led_GPIO_Port GPIOC
#define in_bt_start_Pin GPIO_PIN_15
#define in_bt_start_GPIO_Port GPIOB
#define in_bt_stop_Pin GPIO_PIN_8
#define in_bt_stop_GPIO_Port GPIOA
#define in_bt_emergency_Pin GPIO_PIN_10
#define in_bt_emergency_GPIO_Port GPIOA
#define in_oc_j_Pin GPIO_PIN_15
#define in_oc_j_GPIO_Port GPIOA
#define out_sw_d2_Pin GPIO_PIN_3
#define out_sw_d2_GPIO_Port GPIOB
#define out_sw_d1_Pin GPIO_PIN_4
#define out_sw_d1_GPIO_Port GPIOB
#define in_oc_conchk_Pin GPIO_PIN_5
#define in_oc_conchk_GPIO_Port GPIOB

CAN_HandleTypeDef hcan; main can
I2C_HandleTypeDef hi2c1; ina225
UART_HandleTypeDef huart1; modbus
UART_HandleTypeDef huart2; other/diagnostics
*/

void dbg_self_test_init_descriptors(struct dbg_self_test *self)
{
	uint8_t c = 0u;

	/* Outputs */
	dout[c].label	  = "onboard_led_Pin";
	dout[c].phy_label = "C13";
	dout[c].state	  = false;
	c++;

	dout[c].label	  = "out_sw_d2";
	dout[c].phy_label = "B3";
	dout[c].state	  = false;
	c++;

	dout[c].label	  = "out_sw_d1";
	dout[c].phy_label = "B4";
	dout[c].state	  = false;
	c++;

	self->dout_array = dout;
	self->dout_count = c;
	c		 = 0u;

	/* Inputs */
	/*
	din[c].label	  = "in_bt_start";
	din[c].phy_label = "B15";
	din[c].expected_state = false;
	din[c].state	  = false;
	c++;
	*/

	/*din[c].label	  = "in_bt_stop";
	din[c].phy_label = "A8";
	din[c].expected_state = true;
	din[c].state	  = false;
	c++;*/

	/*
	din[c].label	  = "in_bt_emergency";
	din[c].phy_label = "A10";
	din[c].expected_state = false;
	din[c].state	  = false;
	c++;
	*/

	din[c].label	      = "in_oc_j";
	din[c].phy_label      = "A15";
	din[c].expected_state = true;
	din[c].state	      = false;
	c++;

	din[c].label	      = "in_oc_conchk";
	din[c].phy_label      = "B5";
	din[c].expected_state = true;
	din[c].state	      = false;
	c++;

	self->din_array = din;
	self->din_count = c;
}

void dbg_self_test_test_dout(struct dbg_self_test	    *self,
			     struct dbg_self_test_dout_desc *target)
{
	uint8_t i = 0u;

	/* Print out test message
	 * TEST MESSAGE -> DELAY ON */
	dbg_self_test_step(self, 0u);
	assert(target->state == false);

	dbg_self_test_step(self, 1999u);
	assert(target->state == false);

	for (i = 0; i < 5u; i++) {
		/* DELAY ON -> DELAY OFF */
		dbg_self_test_step(self, 1u);
		assert(target->state == true);

		dbg_self_test_step(self, 499u);
		assert(target->state == true);

		/* DELAY OFF -> DELAY ON */
		dbg_self_test_step(self, 1u);
		assert(target->state == false);

		dbg_self_test_step(self, 499u);
		assert(target->state == false);
	}

	/* DELAY ON -> NEXT TEST */
	dbg_self_test_step(self, 1u);
	assert(target->state == false);
}

int main()
{
	dbg_self_test_init(&st);
	dbg_self_test_init_descriptors(&st);

	dbg_self_test_test_dout(&st, &dout[0]);
	dbg_self_test_test_dout(&st, &dout[1]);
	dbg_self_test_test_dout(&st, &dout[2]);

	dbg_self_test_step(&st, 0);

	return 0;
}
