#include "chademo_se.h"

#include <assert.h>

struct chademo_se chse_main;

void chademo_se_test_charge_start_button_pressed(struct chademo_se *self)
{
	struct chademo_se_vgpio vgpio;

	chademo_se_get_vgpio(self, &vgpio);	

	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);
	vgpio.in.bt_start = true;

	chademo_se_set_vgpio(self, &vgpio);
	assert(chademo_se_step(self, 0) ==
		CHADEMO_SE_EVENT_CHARGE_START_BUTTON_PRESSED);
}

void chademo_se_test_charge_start_signal(struct chademo_se *self)
{
	struct chademo_se_vgpio vgpio;

	chademo_se_get_vgpio(self, &vgpio);
	assert(vgpio.out.sw_d1 == false);
	assert(chademo_se_step(self, 0) == CHADEMO_SE_EVENT_NONE);

	chademo_se_get_vgpio(self, &vgpio);
	assert(vgpio.out.sw_d1 == true);
}

void chademo_se_test_normal_run(struct chademo_se *self)
{
	chademo_se_init(self);	

	chademo_se_test_charge_start_button_pressed(self);
	chademo_se_test_charge_start_signal(self);
}

int main()
{
	chademo_se_test_normal_run(&chse_main);

	return 0;
}
