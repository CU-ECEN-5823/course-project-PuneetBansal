/* @FileName	: main.c
 * @description	: Sending the pb0 button state from publisher to subscriber via bluetooth mesh
 * @author 		: Puneet Bansal
 * @reference	: Silicon Labs SDK -https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 * 				: Bluetooth Mesh SDK- https://www.silabs.com/documents/login/reference-manuals/bluetooth-le-and-mesh-software-api-reference-manual.pdf
 *				  SOC Bluetooth Mesh Light, SOC Bluetooth Mesh Switch example code.
 */

#include <stdbool.h>
#include "native_gecko.h"
#include "log.h"
#include "display.h"
#include "main.h"
#include "gpiointerrupt.h"

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

	bt_event=0;


	/* Infinite loop */
	while (1) {
		struct gecko_cmd_packet *evt = gecko_wait_event();
		displayPrintf(DISPLAY_ROW_ACTION,"");
		bool pass = mesh_bgapi_listener(evt);
		if (pass) {
			handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		}
	};
}
