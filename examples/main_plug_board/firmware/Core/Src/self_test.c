/** This is a stm32 wrapper for chademo_se self-test functionality
 *  This wrapper links self-test functionality with hardware */

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

/* Insert prototypes into main code */
extern void self_test_stm32_init();
extern void self_test_stm32_run();

#include <stdbool.h>
#include <stdio.h>

#include "SEGGER_RTT.h"
#include "main.h"

#define DBG_SELF_TEST_LOG(x)                                                  \
	printf x;                                                             \
	fflush(0)
#define DBG_SELF_TEST_LOG_IMPL
#include "self_test.h"

volatile bool	     dbg_self_test_enabled = false;
struct dbg_self_test dbg_self_test;

struct dbg_self_test_din_desc  din[5u];
struct dbg_self_test_dout_desc dout[5u];

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
	c = 0u;

	/* Inputs */
	/*
	din[c].label	  = "in_bt_start";
	din[c].phy_label = "B15";
	din[c].expected_state = false;
	din[c].state	  = false;
	c++;
	*/

	/* din[c].label	  = "in_bt_stop";
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

	din[c].label	  = "in_oc_j";
	din[c].phy_label = "A15";
	din[c].expected_state = true;
	din[c].state	  = false;
	c++;

	din[c].label	  = "in_oc_conchk";
	din[c].phy_label = "B5";
	din[c].expected_state = true;
	din[c].state	  = false;
	c++;

	self->din_array = din;
	self->din_count = c;
}

void self_test_stm32_init() {
	dbg_self_test_init(&dbg_self_test);
	dbg_self_test_init_descriptors(&dbg_self_test);
}

void self_test_stm32_run(uint32_t delta_time_ms)
{
	/* Self test */
	while (dbg_self_test_enabled) {
		/*--- Time stuff begin ---*/
		static uint32_t prev_ms = 0u;
		uint32_t	cur_ms	= 0u;
		uint32_t	dt_ms	= 0u; /* Delta time */

		cur_ms	= HAL_GetTick();
		dt_ms	= cur_ms - prev_ms;
		prev_ms = cur_ms;
		/*--- Time stuff end ---*/

		if (SEGGER_RTT_HasKey()) {
			int c = SEGGER_RTT_GetKey();
			dbg_self_test_feed_shell_char(&dbg_self_test,
						      (const char)c);
		}

		dbg_self_test_step(&dbg_self_test, dt_ms);

		/* --- Physical Pin Updates --- */

		// Onboard LED (Active Low on many STM32 boards, but logic
		// follows the struct)
		HAL_GPIO_WritePin(onboard_led_GPIO_Port, onboard_led_Pin,
				  (GPIO_PinState)dout[0].state);

		// Switch D2
		HAL_GPIO_WritePin(out_sw_d2_GPIO_Port, out_sw_d2_Pin,
				  (GPIO_PinState)dout[1].state);

		// Switch D1
		HAL_GPIO_WritePin(out_sw_d1_GPIO_Port, out_sw_d1_Pin,
				  (GPIO_PinState)dout[2].state);

		/* Inverted inputs. Because pressed state is defined as
		 * grounded by the hardware designer */

		/* Not really used */
		/*din[0].state =
		    !HAL_GPIO_ReadPin(in_bt_start_GPIO_Port, in_bt_start_Pin);*/

		/* Not really used */
		/*din[0].state =
		    HAL_GPIO_ReadPin(in_bt_stop_GPIO_Port, in_bt_stop_Pin);*/

		/* Not really used */
		/*din[2].state = HAL_GPIO_ReadPin(
		    !in_bt_emergency_GPIO_Port, in_bt_emergency_Pin);*/

		din[0].state =
		    HAL_GPIO_ReadPin(in_oc_j_GPIO_Port, in_oc_j_Pin);

		din[1].state =
		    HAL_GPIO_ReadPin(in_oc_conchk_GPIO_Port, in_oc_conchk_Pin);
	}
}
