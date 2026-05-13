#include <stdio.h>

#define LINBUS_IMPLEMENTATION
#include "linbus.h"

#ifdef    NDEBUG
#undef assert
#define assert(e) if (e) {};
#endif /* NDEBUG */

#define DATA_LEN 8u

struct linbus lb;

int main()
{
	uint8_t i;

	linbus_init(&lb);

	assert(linbus_step(&lb) == LINBUS_EVENT_NONE);

	assert(linbus_queue_frame(&lb, 0u, (uint8_t *)"Hlowrld!", DATA_LEN) ==
	       true);

	/* Can't queue more than one frame at a time */
	assert(linbus_queue_frame(&lb, 0u, (uint8_t *)"Hlowrld!", DATA_LEN) ==
	       false);

	assert(linbus_step(&lb) == LINBUS_EVENT_SEND_BREAK);
	linbus_ack_event(&lb);

	assert(linbus_step(&lb) == LINBUS_EVENT_TX_READY);
	linbus_ack_event(&lb);
	assert(linbus_get_tx_data(&lb) == 0x55u);

	assert(linbus_step(&lb) == LINBUS_EVENT_TX_READY);
	linbus_ack_event(&lb);
	printf("pid: 0x%02X\n", linbus_get_tx_data(&lb));

	for (i = 0; i < DATA_LEN; i++) {
		assert(linbus_step(&lb) == LINBUS_EVENT_TX_READY);
		linbus_ack_event(&lb);
		assert(linbus_get_tx_data(&lb) == "Hlowrld!"[i]);
		printf("data[%u]: %c (0x%02X)\n", i,
		       (char)linbus_get_tx_data(&lb),
		       (char)linbus_get_tx_data(&lb));
	}

	assert(linbus_step(&lb) == LINBUS_EVENT_TX_READY);
	linbus_ack_event(&lb);
	printf("sum: 0x%02X\n", linbus_get_tx_data(&lb));

	assert(linbus_step(&lb) == LINBUS_EVENT_FRAME_SENT);
	linbus_ack_event(&lb);

	assert(linbus_step(&lb) == LINBUS_EVENT_NONE);

	return 0;
}
