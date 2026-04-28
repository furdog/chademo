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

#ifndef DBG_SELF_TEST_LOG_HEADER_GUARD
#define DBG_SELF_TEST_LOG_HEADER_GUARD

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef DBG_SELF_TEST_LOG
#define DBG_SELF_TEST_LOG(x) (void)(x)
#endif

#define DBG_SELF_TEST_SHELL_MAX_CHARACTERS 32u

enum dbg_self_test_state {
	DBG_SELF_TEST_STATE_TEST_DOUT,
	DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_ON,
	DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_OFF,

	DBG_SELF_TEST_STATE_VALIDATE_DIN,

	DBG_SELF_TEST_STATE_TERMINAL
};

struct dbg_self_test_shell {
	char	cmd[DBG_SELF_TEST_SHELL_MAX_CHARACTERS];
	uint8_t len;
	bool	ready;
};

struct dbg_self_test_din_desc {
	/** Label, for example "in_oc_j" */
	const char *label;

	/** physical pin label "A15" */
	const char *phy_label;

	/** Initial (expected) state */
	bool expected_state;

	/** Current state */
	bool state;

	/** Previous (delta) state */
	bool _state_d;
};

struct dbg_self_test_dout_desc {
	/** Label, for example "out_sw_d1" */
	const char *label;

	/** physical pin label, for examble "B4" */
	const char *phy_label;

	/** Current state */
	bool state;
};

struct dbg_self_test {
	uint32_t _wait_ms;
	uint32_t _timer_ms;
	uint32_t _counter;
	uint32_t _dio_check_timer_ms;

	uint8_t _state;

	/** If interactive mode is selected, the user will be prompted
	 *  to proceed */
	bool			   interactive_mode;
	struct dbg_self_test_shell _shell;

	/* Output specific */
	struct dbg_self_test_dout_desc *dout_array;
	uint8_t				dout_count;
	uint8_t				_dout_iter;

	/* Input specific */
	struct dbg_self_test_din_desc *din_array;
	uint8_t			       din_count;
	uint8_t			       _din_iter;

	/* first run */
	bool init;
};

#ifdef DBG_SELF_TEST_LOG_IMPL
static void dbg_self_test_init(struct dbg_self_test *self)
{
	self->_wait_ms		  = 0u;
	self->_timer_ms		  = 0u;
	self->_counter		  = 0u;
	self->_dio_check_timer_ms = 0u;

	self->_state = 0u;

	self->interactive_mode = false;
	self->_shell.len       = 0u;
	self->_shell.ready     = false;

	/* Output specific */
	self->dout_array = NULL;
	self->dout_count = 0u;
	self->_dout_iter = 0u;

	/* Input specific */
	self->din_array = NULL;
	self->din_count = 0u;
	self->_din_iter = 0u;

	self->init = true;
}

/** Feed single input character into interactive shell */
static void dbg_self_test_feed_shell_char(struct dbg_self_test *self,
					  const char		c)
{
	if (self->_shell.len < (DBG_SELF_TEST_SHELL_MAX_CHARACTERS - 1u)) {
		self->_shell.cmd[self->_shell.len] = c;
		self->_shell.len++;
	}

	if (c == '\n') {
		self->_shell.cmd[self->_shell.len] = '\0';
		self->_shell.ready		   = true;
	}
}

static void _dbg_self_test_log_validate_shell_op(struct dbg_self_test *self)
{
	/* Interactive shell confirmation query */
	const char *confirmation_query = "To confirm, enter 'y' \n>> ";

	if (self->interactive_mode) {
		DBG_SELF_TEST_LOG(
		    ("Please, validate operation. %s", confirmation_query));
	}
}

/** Check answer from interactive shell (yes or no) */
static bool _dbg_self_test_shell_confirm_action(struct dbg_self_test *self)
{
	bool result = false;

	if (self->interactive_mode && self->_shell.ready) {
		if (self->_shell.cmd[0] == 'y') {
			result = true;
		} else {
			DBG_SELF_TEST_LOG(
			    ("Invalid input: %s", self->_shell.cmd));
			_dbg_self_test_log_validate_shell_op(self);
		}

		self->_shell.len   = 0u;
		self->_shell.ready = false;
	}

	return result;
}

static void _dbg_self_test_dout_log_name(struct dbg_self_test *self)
{
	struct dbg_self_test_dout_desc *dout =
	    &self->dout_array[self->_dout_iter];

	const char *trig =
	    self->interactive_mode ? "" : "(triggering 5 times)";

	DBG_SELF_TEST_LOG(
	    ("Testing %s (%s) %s\n", dout->label, dout->phy_label, trig));
}

static void _dbg_self_test_track_dio_changes(struct dbg_self_test *self)
{
	uint8_t i;

	for (i = 0u; i < self->din_count; i++) {
		struct dbg_self_test_din_desc *din = &self->din_array[i];

		if (din->state == din->_state_d) {
			continue;
		}

		DBG_SELF_TEST_LOG(("%s (%s) changed state to %s\n", din->label,
				   din->phy_label,
				   din->state ? "true" : "false"));

		din->_state_d = din->state;
	}
}

static void dbg_self_test_step(struct dbg_self_test *self,
			       uint32_t		     delta_time_ms)
{
	if (self->init) {
		uint8_t i;

		self->init = false;

		for (i = 0u; i < self->din_count; i++) {
			self->din_array[i]._state_d =
			    !self->din_array[i].state;
		}
	}

	self->_dio_check_timer_ms += delta_time_ms;
	if (self->_dio_check_timer_ms >= 100u) {
		self->_dio_check_timer_ms = 0u;
		_dbg_self_test_track_dio_changes(self);
	}

	switch (self->_state) {
	case DBG_SELF_TEST_STATE_TEST_DOUT:
		if (self->_dout_iter >= self->dout_count) {
			self->_state = DBG_SELF_TEST_STATE_VALIDATE_DIN;
			break;
		}

		_dbg_self_test_dout_log_name(self);
		_dbg_self_test_log_validate_shell_op(self);
		self->_state	= DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_ON;
		self->_wait_ms	= 2000u;
		self->_timer_ms = 0u;
		self->_counter	= 0u;
		break;

	case DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_OFF:
		self->_timer_ms += delta_time_ms;

		if (self->_timer_ms < self->_wait_ms) {
			break;
		}

		self->dout_array[self->_dout_iter].state = false;

		self->_state	= DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_ON;
		self->_timer_ms = 0u;
		self->_wait_ms	= 500u;

		/* At this point device made full ON/OFF cycle */
		self->_counter += 1u;

		break;

	case DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_ON:
		self->_timer_ms += delta_time_ms;

		if (self->_timer_ms < self->_wait_ms) {
			break;
		}

		/* Confirm shell action (interactive mode) */
		if (_dbg_self_test_shell_confirm_action(self)) {
			self->_state = DBG_SELF_TEST_STATE_TEST_DOUT;
			self->_dout_iter++;
			break;
		}

		/* Skip the last ON cycle */
		if (!self->interactive_mode && (self->_counter >= 5u)) {
			self->_state = DBG_SELF_TEST_STATE_TEST_DOUT;
			self->_dout_iter++;
			break;
		}

		self->dout_array[self->_dout_iter].state = true;

		self->_state	= DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_OFF;
		self->_timer_ms = 0u;
		self->_wait_ms	= 500u;

		break;

	case DBG_SELF_TEST_STATE_VALIDATE_DIN: {
		bool	invalid = false;
		uint8_t i;

		DBG_SELF_TEST_LOG(("Validating digital inputs state:\n"));
		self->_state = DBG_SELF_TEST_STATE_TERMINAL;

		for (i = 0u; i < self->din_count; i++) {
			struct dbg_self_test_din_desc *din =
			    &self->din_array[i];

			if (din->state == din->expected_state) {
				continue;
			}

			DBG_SELF_TEST_LOG(
			    ("%s (%s) state is invalid. Expected: %s\n",
			     din->label, din->phy_label,
			     din->state ? "false, got: true"
					: "true, got: false"));

			invalid = true;
		}

		if (invalid) {
			DBG_SELF_TEST_LOG(("Some checks has failed\n"));
		} else {
			DBG_SELF_TEST_LOG(
			    ("All digital input states are valid!\n"));
		}

		_dbg_self_test_log_validate_shell_op(self);

		break;
	}

	default:
		/* Confirm shell action (interactive mode) */
		if (!self->interactive_mode ||
		    _dbg_self_test_shell_confirm_action(self)) {
			/* Repeat all again */
			self->_state	 = DBG_SELF_TEST_STATE_TEST_DOUT;
			self->_dout_iter = 0u;
			break;
		}
		break;
	}
}
#endif /* DBG_SELF_TEST_LOG_IMPL */

#endif /* DBG_SELF_TEST_LOG_HEADER_GUARD */
