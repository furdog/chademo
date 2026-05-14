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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "SEGGER_RTT.h"
#include "F7_INA226.h"

#include "ssd1306.h"
#include "fonts.h"

#define DBG_SELF_TEST_LOG(x)                                                  \
	printf x;                                                             \
	fflush(0)
#define DBG_SELF_TEST_LOG_IMPL
#include "self_test.h"

#define LINBUS_LOG(e)                                                         \
	printf("%s %i: ", __FILE__, __LINE__);                                \
	printf e
#define LINBUS_DEBUG_STATES
#define LINBUS_IMPLEMENTATION
#include "linbus.h"

/* Defines */
#define DBG_I2C_SCAN_INTERVAL_MS 5000u

/* Prototypes */
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart1;

/* Variables */
volatile bool	     dbg_self_test_enabled = true;
struct dbg_self_test dbg_self_test;

struct dbg_self_test_din_desc  din[5u];
struct dbg_self_test_dout_desc dout[5u];

volatile float dbg_ina226_self_test_bus_V       = 0u;
volatile uint32_t dbg_ina226_self_test_timer_ms = 0u;
volatile uint32_t dbg_uart_self_test_timer_ms   = 0u;
volatile char dbg_uart_self_test_str[255u]  = "Пyтін xyйлo!!!\r\n\0";

volatile uint32_t dbg_i2c_scan_timer_ms = DBG_I2C_SCAN_INTERVAL_MS;

struct linbus lb;

/* Functions */
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

void scan_i2c()
{
	uint8_t found = 0u;

	printf("Scanning i2c devices...\n");

	for (int i = 0; i < 128; ++i) {
		uint16_t address = i << 1;

		if (HAL_I2C_IsDeviceReady(&hi2c1, address, 1, HAL_MAX_DELAY) == HAL_OK) {
			printf("Address: 0x%X\r\n", address >> 1);
			found++;
		}
	}

	if (found == 0u) {
		printf("No i2c devices has been found :(\n");
	}
}

void self_test_stm32_init() {
	dbg_self_test_init(&dbg_self_test);
	dbg_self_test_init_descriptors(&dbg_self_test);
	linbus_init(&lb);
}

void test_ina(uint32_t delta_time_ms) {
	dbg_ina226_self_test_bus_V = INA226_getBusV(&hi2c1, INA226_ADDRESS);

	dbg_ina226_self_test_timer_ms += delta_time_ms;
	if (dbg_ina226_self_test_timer_ms > 2500u) {
		static uint32_t timer_ms;
		timer_ms += delta_time_ms;

		char str[255u];
		sprintf(str, "Hello t: %u!", timer_ms);

		dbg_ina226_self_test_timer_ms = 0u;
		printf("INA226_bus_V: %f\n", dbg_ina226_self_test_bus_V);

		// Write data to local screenbuffer
		ssd1306_SetCursor(0, 36);
			
		ssd1306_WriteString(str, Font_11x18, White);

		// Copy all data from local screenbuffer to the screen
		ssd1306_UpdateScreen(&hi2c1);
	}
}

void test_i2c(uint32_t delta_time_ms) {
	dbg_i2c_scan_timer_ms += delta_time_ms;
	if (dbg_i2c_scan_timer_ms >= DBG_I2C_SCAN_INTERVAL_MS) {
		dbg_i2c_scan_timer_ms = 0u;

		scan_i2c();
	}
}

void test_serial(uint32_t delta_time_ms) {
	/* Send uart signal */
	dbg_uart_self_test_timer_ms += delta_time_ms;
	if (dbg_uart_self_test_timer_ms >= 100u) {
		dbg_uart_self_test_timer_ms = 0u;

		/*if(!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
			ATOMIC_CLEAR_BIT(huart1.Instance->CR1, USART_CR1_RE); // Disable Receiver
			HAL_UART_Transmit(&huart1, dbg_uart_self_test_str, strlen(dbg_uart_self_test_str), 1000);
			while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET); // Wait for physical end
			ATOMIC_SET_BIT(huart1.Instance->CR1, USART_CR1_RE); // Re-enable Receiver
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		}*/
		
		linbus_send_frame(&lb, 37, (uint8_t *)"HeartBT!", 8u);
	}

	/* Limited loop (upper bound) */
	for (uint8_t i = 0u; i < 8u; i++) {
		linbus_step(&lb);

		if (lb._event == LINBUS_EVENT_SEND_BREAK) {
			//HAL_LIN_SendBreak(&huart1);
			linbus_ack_event(&lb);
		/*} else if (lb._event == LINBUS_EVENT_SEND_DATA) {
			if (HAL_UART_Transmit_IT(&huart1, &lb._tx, 1u) != HAL_OK) {
				break;
			}
			linbus_ack_event(&lb);*/
		} else if (lb._event == LINBUS_EVENT_FRAME_SENT) {
			HAL_LIN_SendBreak(&huart1);
			if (HAL_UART_Transmit_IT(&huart1, &lb._buf, 11u) != HAL_OK) {
				break;
			}
			linbus_ack_event(&lb);
		} else {
			linbus_ack_event(&lb);
		}
	}

	/* Print back response */
	uint8_t c;

	if (HAL_UART_Receive(&huart1, &c, 1u, 0u) == HAL_OK) {
		putchar(c);
	}
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

		//test_i2c(dt_ms);
		//test_ina(dt_ms);
		test_serial(dt_ms);
	}
}
