#include <stdbool.h>
#include "native_gecko.h"
#include "log.h"
#include "main.h"
#include "letimer.h"
#include "gpio.h"

extern void gecko_main_init();
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

extern void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

int main(void)
{

  // Initialize stack
	gecko_main_init();
	buttonPress=0;
	buttonDefine=0;
	clock_init();
	letimer_init();
	gpioInit();
	logInit();
	displayInit();
	gpioEnableDisplay();
	GPIOINT_Init();
	GPIOINT_CallbackRegister(6, gpioCallback1);
	//gecko_cmd_hardware_set_soft_timer(1 * 32768,LcdHandler ,0); //Enabling one second timer to update LCD.
  /* Infinite loop */
  while (1) {
	//displayPrintf(DISPLAY_ROW_ACTION,"LCD check");
	struct gecko_cmd_packet *evt = gecko_wait_event();
	bool pass = mesh_bgapi_listener(evt);
	if (pass) {
		handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
	}
  };
}
