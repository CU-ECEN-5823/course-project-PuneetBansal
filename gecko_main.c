/* @FileName	: gecko_main.c
 * @description	: Code for handling events related to provisioning the publisher and subscriber and sending the
 * 				  button press status of PB0 using bluetooth mesh API. The code is built up on silicon labs
 * 				  bluetooth mesh empty example.
 * 				  The device type i.e. publisher/subscriber can be switched by changing the DEVICE_IS_ONOFF_PUBLISHER
 * 				  define in ble_mesh_device_type.h
 * @author 		: Puneet Bansal
 * @reference	: Silicon Labs SDK -https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 * 				: Bluetooth Mesh SDK- https://www.silabs.com/documents/login/reference-manuals/bluetooth-le-and-mesh-software-api-reference-manual.pdf
 *				  SOC Bluetooth Mesh Light, SOC Bluetooth Mesh Switch example code.

/***************************************************************************//**
 * @file
 * @brief Silicon Labs BT Mesh Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

#include "src/main.h"
#include "src/log.h"


/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#include "src/ble_mesh_device_type.h"
#include <stdlib.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"


#define buttonPort gpioPortF
#define buttonPin 6

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

// bluetooth stack heap
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config =
{
		.bluetooth.max_connections = MAX_CONNECTIONS,
		.bluetooth.max_advertisers = MAX_ADVERTISERS,
		.bluetooth.heap = bluetooth_stack_heap,
		.bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
		.bluetooth.sleep_clock_accuracy = 100,
		.bluetooth.linklayer_priorities = &linklayer_priorities,
		.gattdb = &bg_gattdb_data,
		.btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
		.pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
		.pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
		.pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
		.max_timers = 16,
};

//#include "src/log.h"

#include "src/display.h"

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

uint16_t element_index = 0;
uint8_t transaction_id = 0;

static void onoff_request(uint16_t model_id,
		uint16_t element_index,
		uint16_t client_addr,
		uint16_t server_addr,
		uint16_t appkey_index,
		const struct mesh_generic_request *request,
		uint32_t transition_ms,
		uint16_t delay_ms,
		uint8_t request_flags)
{
	LOG_INFO("Entered onff request");

	if(request->on_off == MESH_GENERIC_ON_OFF_STATE_ON)
	{
		displayPrintf(DISPLAY_ROW_TEMPVALUE,"%s","Button Pressed");
	}
	else
	{
		displayPrintf(DISPLAY_ROW_TEMPVALUE,"%s","Button Released");
	}

}
static void onoff_change(uint16_t model_id,
		uint16_t element_index,
		const struct mesh_generic_state *current,
		const struct mesh_generic_state *target,
		uint32_t remaining_ms)
{
	LOG_INFO("Entered on_off change");
}


/**
 * See light switch app.c file definition
 */

void gecko_bgapi_classes_init_server_friend(void)
{
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
	//gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
	//gecko_bgapi_class_mesh_friend_init();
}


/**
 * See main function list in soc-btmesh-switch project file
 */
void gecko_bgapi_classes_init_client_lpn(void)
{
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
	//gecko_bgapi_class_mesh_friend_init();

}
void gecko_main_init()
{
	// Initialize device
	initMcu();
	// Initialize board
	initBoard();
	// Initialize application
	initApp();

	// Minimize advertisement latency by allowing the advertiser to always
	// interrupt the scanner.
	linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

	gecko_stack_init(&config);

	if( DeviceUsesClientModel() ) {
		gecko_bgapi_classes_init_client_lpn();
	} else {
		gecko_bgapi_classes_init_server_friend();
	}

	// Initialize coexistence interface. Parameters are taken from HAL config.
	gecko_initCoexHAL();

}

struct gecko_msg_system_get_bt_address_rsp_t* bluetoothAdd;
void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	uint8_t * meshPublisher;
	char devName[20];
	uint16_t ret;
	uint16_t ret1;
	switch (evt_id)
	{
	case gecko_evt_system_boot_id:
		LOG_INFO("entered boot id");

#if DEVICE_USES_BLE_MESH_CLIENT_MODEL
		displayPrintf(DISPLAY_ROW_NAME,"Publisher");
#endif

#if DEVICE_USES_BLE_MESH_SERVER_MODEL
		displayPrintf(DISPLAY_ROW_NAME,"Subscriber");
#endif

		bluetoothAdd=gecko_cmd_system_get_bt_address();
		meshPublisher=(uint8_t *)&bluetoothAdd->address;


#if DEVICE_USES_BLE_MESH_CLIENT_MODEL
		sprintf(devName, "5823Pub %02x%02x", meshPublisher[1], meshPublisher[0]);
#endif

#if DEVICE_USES_BLE_MESH_SERVER_MODEL
		sprintf(devName, "5823Sub %02x%02x", meshPublisher[1], meshPublisher[0]);
#endif

		LOG_INFO("Device name is %s",devName);
		displayPrintf(DISPLAY_ROW_BTADDR,devName);

		/*Setting the name of the device*/
		ret=gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(devName), (uint8 *)devName)->result;
		if (ret)
		{
			LOG_INFO("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", ret);
		}

		struct gecko_msg_flash_ps_erase_all_rsp_t *rsp;

		if(GPIO_PinInGet(buttonPort,buttonPin)==0)
		{
			LOG_INFO("Initiating factory reset");
			displayPrintf(DISPLAY_ROW_ACTION,"Factory Reset");
			rsp=gecko_cmd_flash_ps_erase_all();
			LOG_INFO("Return value of erase all is %d",rsp->result);
			gecko_cmd_hardware_set_soft_timer(2*32768,timerHandle,1);
		}

		else
		{
			// Initialize Mesh stack
			int ret1=gecko_cmd_mesh_node_init()->result;
			if(ret1)
			{
				LOG_INFO("gecko_cmd_mesh_node_init() failed, code %x\r\n", ret1);
			}
		}
		break;


	case gecko_evt_hardware_soft_timer_id:
		if(evt->data.evt_hardware_soft_timer.handle==timerHandle)
		{
			// reset the device to finish factory reset
			LOG_INFO("Just Before reset");
			gecko_cmd_system_reset(0);
		}
		break;

	case gecko_evt_mesh_node_initialized_id:

		LOG_INFO("Mesh node initialized");
		LOG_INFO("Now calling generic client init");
		struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

		/*Check if the device is provisioned or not. If it is not provisioned, then start the beaconing process.
		 *Otherwise, do the mesh lib init.*/
		errorcode_t err,err1,err2;

		if(pData->provisioned)
		{
			LOG_INFO("Node is provisioned, calling mesh lib init");
			displayPrintf(DISPLAY_ROW_ACTION,"Provisioned");
			//gecko_cmd_mesh_generic_client_init();

#if DEVICE_USES_BLE_MESH_CLIENT_MODEL
			gecko_cmd_mesh_generic_client_init();
			GPIOINT_Init();
			GPIOINT_CallbackRegister(6, gpioCallback1);
			err2=mesh_lib_init(malloc,free,8); // Initializing the mesh library with 8 model support.
			LOG_INFO("Err in client mesh lib init is %d",err2);

#endif

#if DEVICE_USES_BLE_MESH_SERVER_MODEL
			LOG_INFO("Calling server init");
			gecko_cmd_mesh_generic_server_init();
			err2=mesh_lib_init(malloc,free,9); // Initializing the mesh library with 8 model support.
			LOG_INFO("Err in server mesh lib init is %d",err2);
			err1=mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,0,onoff_request,onoff_change);
			LOG_INFO("Err in server register is %d",err1);
			err=mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,0,mesh_generic_state_on_off);
			LOG_INFO("Err in server publish is %d",err);
#endif
		}
		else
		{
			LOG_INFO("Node is unprovisioned");
			struct gecko_msg_mesh_node_start_unprov_beaconing_rsp_t* beaconingResp;
			displayPrintf(DISPLAY_ROW_CONNECTION,"Unprovisioned");
			LOG_INFO("calling beaconing");
			// The Node is now initialized, start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
			beaconingResp=gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
			LOG_INFO("Beaconing response result is %d",beaconingResp->result);
		}
		break;

	case gecko_evt_mesh_node_provisioning_started_id:

		displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioning");
		LOG_INFO("Provisioning Started");
		break;

	case gecko_evt_mesh_node_provisioned_id:
		displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioned");
		LOG_INFO("Provisioning Completed");

		break;

	case gecko_evt_mesh_node_provisioning_failed_id:

		displayPrintf(DISPLAY_ROW_CONNECTION,"Provision Fail");
		struct gecko_msg_mesh_node_provisioning_failed_evt_t *res=(struct gecko_msg_mesh_node_provisioning_failed_evt_t *)&(evt->data);;
		LOG_INFO("Provision Fail %x",res->result);
		break;

	case gecko_evt_mesh_node_key_added_id:
		LOG_INFO("got new %s key with index %x\r\n", evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
				evt->data.evt_mesh_node_key_added.index);
		break;

	case gecko_evt_mesh_node_model_config_changed_id:
		LOG_INFO("model config changed\r\n");
		break;

	case gecko_evt_le_connection_opened_id:
		LOG_INFO("evt:gecko_evt_le_connection_opened_id\r\n");
		displayPrintf(DISPLAY_ROW_ACTION,"connected");
		break;

	case gecko_evt_mesh_generic_server_client_request_id:
		LOG_INFO("evt gecko_evt_mesh_generic_server_client_request_id\r\n");
		// pass the server client request event to mesh lib handler that will invoke
		// the callback functions registered by application
		mesh_lib_generic_server_event_handler(evt);
		break;

	case gecko_evt_mesh_generic_server_state_changed_id:

		LOG_INFO("Entered state changed id");
		// pass the server state changed event to mesh lib handler that will invoke
		// the callback functions registered by application
		mesh_lib_generic_server_event_handler(evt);
		break;

	case gecko_evt_le_connection_closed_id:
		if (boot_to_dfu)
		{
			/* Enter to DFU OTA mode */
			gecko_cmd_system_reset(2);
		}
		break;

	case gecko_evt_system_external_signal_id:
		LOG_INFO("Entered External Signal Id");
		struct mesh_generic_request req;
		uint16 resp;
		req.kind= mesh_generic_request_on_off;

		if (evt->data.evt_system_external_signal.extsignals & bt_risingEdgeInt)
		{
			bt_event &= ~bt_risingEdgeInt;
			LOG_INFO("Rising Edge Interrupt Received");
			req.on_off=MESH_GENERIC_ON_OFF_STATE_ON;
			resp = mesh_lib_generic_client_publish(
					MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
					element_index,
					transaction_id,
					&req,
					0,     // transition
					0,
					0     // flags
			);
			transaction_id++;
			if (resp) {
				printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
			} else {
				printf("request sent, trid = %u, delay = %d\r\n", transaction_id, 0);
			}


		}
		if (evt->data.evt_system_external_signal.extsignals & bt_fallingEdgeInt)
		{
			bt_event &= ~bt_fallingEdgeInt;
			LOG_INFO("Falling Edge Interrupt Received");
			req.on_off=MESH_GENERIC_ON_OFF_STATE_OFF;
			resp = mesh_lib_generic_client_publish(
					MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
					element_index,
					transaction_id,
					&req,
					0,     // transition
					0,
					0     // flags
			);
			transaction_id++;
			if (resp) {
				printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
			} else {
				printf("request sent, trid = %u, delay = %d\r\n", transaction_id, 0);
			}
		}
		break;
	default:
		break;
	}
}
