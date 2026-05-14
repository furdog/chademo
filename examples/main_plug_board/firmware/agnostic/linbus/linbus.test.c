#include <stdio.h>

#define LINBUS_LOG(e)                                                         \
	printf("%s %i: ", __FILE__, __LINE__);                                \
	printf e
#define LINBUS_DEBUG_STATES
#define LINBUS_IMPLEMENTATION
#include "linbus.h"

#ifdef NDEBUG
#undef assert
#define assert(e)                                                             \
	if (e) {                                                              \
	};
#endif /* NDEBUG */

#define DATA_LEN 8u

struct linbus lb;
struct linbus saved; /* Saved state, to repeat some actions */

int linbus_test_transmission(struct linbus *self, uint8_t *data)
{
	uint8_t i;

	assert(linbus_step(self) == LINBUS_EVENT_NONE);

	/* Can't send frame with invalid id */
	assert(linbus_send_frame(self, 0x40u, data, DATA_LEN) == false);

	/* Can't send frame with invalid len */
	assert(linbus_send_frame(self, 0u, data, 9u) == false);

	assert(linbus_send_frame(self, 0u, data, DATA_LEN) == true);

	/* Can't send more than one frame at a time */
	assert(linbus_send_frame(self, 0u, data, DATA_LEN) == false);

	assert(linbus_step(self) == LINBUS_EVENT_SEND_BREAK);
	assert(linbus_step(self) == LINBUS_EVENT_NONE); /* EACK */
	linbus_ack_event(self); /* Break event acknowledged */

	assert(linbus_step(self) == LINBUS_EVENT_SEND_DATA);
	linbus_ack_event(self);
	assert(linbus_get_tx_data(self) == 0x55u);

	assert(linbus_step(self) == LINBUS_EVENT_SEND_DATA);
	linbus_ack_event(self);
	assert(linbus_get_tx_data(self) == 0x80u);

	for (i = 0; i < DATA_LEN; i++) {
		assert(linbus_step(self) == LINBUS_EVENT_SEND_DATA);
		linbus_ack_event(self);
		assert(linbus_get_tx_data(self) == data[i]);
	}

	assert(linbus_step(self) == LINBUS_EVENT_SEND_DATA);
	linbus_ack_event(self);

	assert(linbus_get_tx_data(self) == (self->_legacy ? 0xE4u : 0x64u));

	assert(linbus_step(self) == LINBUS_EVENT_SEND_COMPLETE);
	linbus_ack_event(self);

	assert(linbus_step(self) == LINBUS_EVENT_NONE);

	return 0;
}

int linbus_test_reception_full(struct linbus *self, uint8_t *data)
{
	uint8_t i;

	assert(linbus_step(self) == LINBUS_EVENT_NONE);

	/* Acknowledge carrier (somebody is transmitting something) */
	linbus_ack_carrier(self);

	/* We must accept break condition */
	assert(linbus_step(self) == LINBUS_EVENT_RECV_BREAK);
	assert(linbus_step(self) == LINBUS_EVENT_NONE); /* Paranoid test */
	linbus_ack_event(self);

	/* Recv sync */
	assert(linbus_step(self) == LINBUS_EVENT_RECV_DATA);
	linbus_set_rx_data(self, 0x55);
	linbus_ack_event(self);

	/* Recv pid */
	assert(linbus_step(self) == LINBUS_EVENT_RECV_DATA);
	linbus_set_rx_data(self, 0x80);
	linbus_ack_event(self);

	saved = *self;

	/* We must accept data now */
	for (i = 0u; i < 8u; i++) {
		assert(linbus_step(self) == LINBUS_EVENT_RECV_DATA);
		linbus_set_rx_data(self, data[i]);
		linbus_ack_event(self);
	}

	/* Recv checksum */
	assert(linbus_step(self) == LINBUS_EVENT_RECV_DATA);
	linbus_set_rx_data(self, 0x64);
	linbus_ack_event(self);

	/* Wait for idle condition */
	assert(linbus_step(self) == LINBUS_EVENT_RECV_COMPLETE);
	linbus_ack_event(self);

	/* Recv complete */
	assert(linbus_step(self) == LINBUS_EVENT_RECV_IDLE);
	linbus_ack_idle(self);
	linbus_ack_event(self);

	return 0;
}

int linbus_test_reception_partial(struct linbus *self, uint8_t *data)
{
	uint8_t i;

	/* We must accept data now */
	for (i = 0u; i < 4u; i++) {
		assert(linbus_step(self) == LINBUS_EVENT_RECV_DATA);
		linbus_set_rx_data(self, data[i]);
		linbus_ack_event(self);
	}

	linbus_ack_idle(self);

	assert(linbus_step(self) == LINBUS_EVENT_RECV_COMPLETE);
	linbus_ack_event(self);
	assert(linbus_step(self) == LINBUS_EVENT_NONE);

	return 0;
}

int main()
{
	/* uint8_t *data = (uint8_t *)"Hlowrld!"; */
	uint8_t *data = (uint8_t *)"HiFurdog";

	linbus_init(&lb);
	lb.baud = 9600u;

	printf("linbus_test_transmission\n");
	linbus_test_transmission(&lb, data);

	lb._legacy = true;
	printf("linbus_test_transmission (legacy)\n");
	linbus_test_transmission(&lb, data);

	printf("frame_time_uS: %u\n", linbus_calc_frame_us(&lb, 8u));

	lb._legacy = false;
	printf("linbus_test_reception_full\n");
	linbus_test_reception_full(&lb, data);

	lb = saved; /* Reset saved state */
	printf("linbus_test_reception_partial\n");
	linbus_test_reception_partial(&lb, data);

	printf("linbus_test_reception_full\n");
	linbus_test_reception_full(&lb, data);

	return 0;
}
