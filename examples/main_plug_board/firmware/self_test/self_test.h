#ifndef DBG_SELF_TEST_LOG_HEADER_GUARD
#define DBG_SELF_TEST_LOG_HEADER_GUARD

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef DBG_SELF_TEST_LOG
#define DBG_SELF_TEST_LOG(x) (void)(x)
#endif

enum dbg_self_test_state {
	DBG_SELF_TEST_STATE_TEST_LED,
	DBG_SELF_TEST_STATE_TEST_SW1,
	DBG_SELF_TEST_STATE_TEST_SW2,

	DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_ON,
	DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_OFF,

	DBG_SELF_TEST_STATE_TERMINAL
};

struct dbg_self_test {
	uint32_t _wait_ms;
	uint32_t _timer_ms;
	uint32_t _counter;

	/* Target value for triggering output multiple times */
	bool *_dout_test_target;

	uint8_t _state;
	uint8_t _state_after_dout_test;

	bool onboard_led;
	bool sw1;
	bool sw2;
};

#ifdef   DBG_SELF_TEST_LOG_IMPL
static void dbg_self_test_init(struct dbg_self_test *self)
{
	self->_wait_ms	= 0u;
	self->_timer_ms = 0u;
	self->_counter	= 0u;

	self->_dout_test_target = NULL;

	self->_state		     = 0u;
	self->_state_after_dout_test = 0u;

	self->onboard_led = false;
	self->sw1	  = false;
	self->sw2	  = false;
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

static void dbg_self_test_step(struct dbg_self_test *self,
			       uint32_t		     delta_time_ms)
{
	switch (self->_state) {
	case DBG_SELF_TEST_STATE_TEST_LED:
		DBG_SELF_TEST_LOG(
		    ("Testing main board LED (blinking 5 times)\n"));
		_dbg_self_test_dout(self, &self->onboard_led);
		self->_state_after_dout_test = DBG_SELF_TEST_STATE_TEST_SW1;
		break;

	case DBG_SELF_TEST_STATE_TEST_SW1:
		DBG_SELF_TEST_LOG(
		    ("Testing main board SW1 (triggering 5 times)\n"));
		_dbg_self_test_dout(self, &self->sw1);
		self->_state_after_dout_test = DBG_SELF_TEST_STATE_TEST_SW2;
		break;
	case DBG_SELF_TEST_STATE_TEST_SW2:
		DBG_SELF_TEST_LOG(
		    ("Testing main board SW2 (triggering 5 times)\n"));
		_dbg_self_test_dout(self, &self->sw2);
		self->_state_after_dout_test = DBG_SELF_TEST_STATE_TERMINAL;
		break;

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

		/* Skip the last ON cycle */
		if (self->_counter >= 5u) {
			self->_state = self->_state_after_dout_test;
			break;
		}

		*self->_dout_test_target = true;

		self->_state	= DBG_SELF_TEST_STATE_TEST_DOUT_DELAY_OFF;
		self->_timer_ms = 0u;
		self->_wait_ms	= 500u;

		break;

	default:
		self->_state = DBG_SELF_TEST_STATE_TEST_LED;
		break;
	}
}
#endif /* DBG_SELF_TEST_LOG_IMPL */

#endif /* DBG_SELF_TEST_LOG_HEADER_GUARD */
