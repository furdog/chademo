#include <stdio.h>

#define DBG_SELF_TEST_LOG(x) printf(x)
#include "self_test.h"

struct dbg_self_test st;

void dbg_self_test_test_dout(struct dbg_self_test *self, bool *target)
{
	uint8_t i = 0u;

	/* Print out test message
	 * TEST MESSAGE -> DELAY ON */
	dbg_self_test_step(self, 0u);
	assert(*target == false);

	dbg_self_test_step(self, 1999u);
	assert(*target == false);

	for (i = 0; i < 5u; i++) {
		/* DELAY ON -> DELAY OFF */
		dbg_self_test_step(self, 1u);
		assert(*target == true);

		dbg_self_test_step(self, 499u);
		assert(*target == true);

		/* DELAY OFF -> DELAY ON */
		dbg_self_test_step(self, 1u);
		assert(*target == false);

		dbg_self_test_step(self, 499u);
		assert(*target == false);
	}

	/* DELAY ON -> NEXT TEST */
	dbg_self_test_step(self, 1u);
	assert(*target == false);
}

int main()
{
	dbg_self_test_init(&st);

	dbg_self_test_test_dout(&st, &st.onboard_led);
	dbg_self_test_test_dout(&st, &st.sw1);
	dbg_self_test_test_dout(&st, &st.sw2);

	dbg_self_test_step(&st, 0);

	return 0;
}
