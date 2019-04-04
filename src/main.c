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

    /*
    gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();

	gecko_bgapi_class_mesh_proxy_init();

	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	gecko_bgapi_class_mesh_generic_client_init();
	//gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
	*/

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
