/*
#define in_bt_start_Pin GPIO_PIN_15
#define in_bt_start_GPIO_Port GPIOB
#define in_bt_stop_Pin GPIO_PIN_8
#define in_bt_stop_GPIO_Port GPIOA
#define in_bt_emergency_Pin GPIO_PIN_10
#define in_bt_emergency_GPIO_Port GPIOA
#define in_oc_j_Pin GPIO_PIN_15
#define in_oc_j_GPIO_Port GPIOA
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
	DBG_SELF_TEST_STATE_TEST_LED,
	DBG_SELF_TEST_STATE_TEST_SW1,
	DBG_SELF_TEST_STATE_TEST_SW2,

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

struct dbg_self_test {
	uint32_t _wait_ms;
	uint32_t _timer_ms;
	uint32_t _counter;
	uint32_t _dio_check_timer_ms;

	/** Target value for triggering output multiple times */
	bool *_dout_test_target;

	uint8_t _state;
	uint8_t _state_after_dout_test;

	/** If interactive mode is selected, the user will be prompted
	 *  to proceed */
	bool			   interactive_mode;
	struct dbg_self_test_shell _shell;

	/* Outputs */
	bool onboard_led;
	bool sw1;
	bool sw2;

	/* Inputs */
	bool bt_start;
	bool bt_stop;
	bool bt_emergency;
	bool oc_j;
	bool oc_conchk;

	/* Deltas (to track changes) */
	bool d_oc_j;
	bool d_oc_conchk;
};

#ifdef DBG_SELF_TEST_LOG_IMPL
static void dbg_self_test_init(struct dbg_self_test *self)
{
	self->_wait_ms	= 0u;
	self->_timer_ms = 0u;
	self->_counter	= 0u;
	self->_dio_check_timer_ms = 0u;

	self->_dout_test_target = NULL;

	self->_state		     = 0u;
	self->_state_after_dout_test = 0u;

	self->interactive_mode = false;
	self->_shell.len       = 0u;
	self->_shell.ready     = false;

	/* Outputs */
	self->onboard_led = false;
	self->sw1	  = false;
	self->sw2	  = false;

	/* Inputs */
	self->bt_start	   = false;
	self->bt_stop	   = false;
	self->bt_emergency = false;
	self->oc_j	   = false;
	self->oc_conchk	   = false;

	/* Deltas (to track changes) */
	self->d_oc_j	  = false;
	self->d_oc_conchk = false;
}

static void _dbg_self_test_dout(struct dbg_self_test *self, bool *target)
{
	self->_state		 = DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_ON;
	self->_wait_ms		 = 2000u;
	self->_timer_ms		 = 0u;
	self->_counter		 = 0u;
	self->_dout_test_target	 = target;
	*self->_dout_test_target = false;
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

static void _dbg_self_test_dout_log_name(struct dbg_self_test *self,
					 const char	      *name)
{
	const char *trig =
	    self->interactive_mode ? "" : "(triggering 5 times)";

	DBG_SELF_TEST_LOG(("Testing main board %s %s\n", name, trig));
}

static void _dbg_self_test_track_dio_changes(struct dbg_self_test *self)
{
	if (self->oc_j != self->d_oc_j) {
		self->d_oc_j = self->oc_j;

		DBG_SELF_TEST_LOG(("oc_j changed to %s\n", self->oc_j ? "true" : "false"));
	}

	if (self->oc_conchk != self->d_oc_conchk) {
		self->d_oc_conchk = self->oc_conchk;
		DBG_SELF_TEST_LOG(("oc_conchk changed to %s\n", self->oc_conchk ? "true" : "false"));
	}
}

static void dbg_self_test_step(struct dbg_self_test *self,
			       uint32_t		     delta_time_ms)
{
	self->_dio_check_timer_ms += delta_time_ms;
	if (self->_dio_check_timer_ms >= 100u) {
		self->_dio_check_timer_ms = 0u;
		_dbg_self_test_track_dio_changes(self);
	}

	switch (self->_state) {
	case DBG_SELF_TEST_STATE_TEST_LED:
		_dbg_self_test_dout_log_name(self, "LED");
		_dbg_self_test_dout(self, &self->onboard_led);
		self->_state_after_dout_test = DBG_SELF_TEST_STATE_TEST_SW1;
		_dbg_self_test_log_validate_shell_op(self);
		break;

	case DBG_SELF_TEST_STATE_TEST_SW1:
		_dbg_self_test_dout_log_name(self, "SW1 (B4)");
		_dbg_self_test_dout(self, &self->sw1);
		self->_state_after_dout_test = DBG_SELF_TEST_STATE_TEST_SW2;
		_dbg_self_test_log_validate_shell_op(self);
		break;

	case DBG_SELF_TEST_STATE_TEST_SW2:
		_dbg_self_test_dout_log_name(self, "SW2 (B3)");
		_dbg_self_test_dout(self, &self->sw2);
		self->_state_after_dout_test =
		    DBG_SELF_TEST_STATE_VALIDATE_DIN;
		_dbg_self_test_log_validate_shell_op(self);
		break;

	case DBG_SELF_TEST_STATE_VALIDATE_DIN: {
		bool	    invalid  = false;
		bool	    expected = false;
		const char *name     = "unknown";

		DBG_SELF_TEST_LOG(("Validating digital inputs state:\n"));
		self->_state = DBG_SELF_TEST_STATE_TERMINAL;

		/* Inputs */
		/* Not used by board (inside connector) */
		/*if (self->bt_start == false) {
			name	 = "bt_start";
			expected = true;
			invalid	 = true;
		}*/

		if (self->bt_stop == false) {
			name	 = "bt_stop";
			expected = false;
			invalid	 = true;
		}

		/* Not used by board (inside connector) */
		/*if (self->bt_emergency == false) {
			name	 = "bt_emergency";
			expected = true;
			invalid	 = true;
		}*/

		if (self->oc_j == false) {
			name	 = "oc_j (A15)";
			expected = true;
			invalid	 = true;
		}

		if (self->oc_conchk == false) {
			name	 = "oc_conchk (B5)";
			expected = true;
			invalid	 = true;
		}

		if (invalid) {
			DBG_SELF_TEST_LOG(
			    ("%s state is invalid! (expected: %s)\n", name,
			     expected ? "true" : "false"));
		} else {
			DBG_SELF_TEST_LOG(("All digital input states are valid!\n"));
		}

		_dbg_self_test_log_validate_shell_op(self);

		break;
	}

	case DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_OFF:
		self->_timer_ms += delta_time_ms;

		if (self->_timer_ms < self->_wait_ms) {
			break;
		}

		*self->_dout_test_target = false;

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
			self->_state = self->_state_after_dout_test;
			break;
		}

		/* Skip the last ON cycle */
		if (!self->interactive_mode && (self->_counter >= 5u)) {
			self->_state = self->_state_after_dout_test;
			break;
		}

		*self->_dout_test_target = true;

		self->_state	= DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_OFF;
		self->_timer_ms = 0u;
		self->_wait_ms	= 500u;

		break;

	default:
		/* Confirm shell action (interactive mode) */
		if (!self->interactive_mode || _dbg_self_test_shell_confirm_action(self)) {
			/* Repeat all again */
			self->_state = DBG_SELF_TEST_STATE_TEST_LED;
			break;
		}
		break;
	}
}
#endif /* DBG_SELF_TEST_LOG_IMPL */

#endif /* DBG_SELF_TEST_LOG_HEADER_GUARD */
