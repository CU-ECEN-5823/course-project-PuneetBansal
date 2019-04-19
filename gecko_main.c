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

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#include "src/ble_mesh_device_type.h"

#include "src/main.h"
#include "src/log.h"
#include "src/display.h"
#include "src/gpio.h"
#include "mesh_lib.h"

#include "mesh_lighting_model_capi_types.h"
/**********DEFINES FOR EXAMPLE CODE*************************************************************************/

#define TIMER_ID_RESTART    78
#define TIMER_ID_FACTORY_RESET  77
#define TIMER_ID_PROVISIONING   66
#define TIMER_ID_RETRANS    10
#define TIMER_ID_FRIEND_FIND 20

static uint8 conn_handle = 0xFF;
static uint16 _elem_index = 0xffff;
/// Address of the Primary Element of the Node
static uint16 _my_address = 0;
static uint8 num_connections = 0;
uint16_t element_index = 0;
uint8_t transaction_id = 0;

/***********************************************************************************************/

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

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

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
void lpn_init(void)
{
  uint16 res;
  // Initialize LPN functionality.
  res = gecko_cmd_mesh_lpn_init()->result;
  if (res) {
    LOG_INFO("LPN init failed (0x%x)\r\n", res);
    return;
  }

  // Configure the lpn with following parameters:
  // - Minimum friend queue length = 2
  // - Poll timeout = 5 seconds
  res = gecko_cmd_mesh_lpn_configure(2, 5 * 1000)->result;
  if (res) {
    LOG_INFO("LPN conf failed (0x%x)\r\n", res);
    return;
  }

  LOG_INFO("trying to find friend...\r\n");
  res = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

  if (res != 0) {
    LOG_INFO("ret.code %x\r\n", res);
  }
}

void switch_node_init(void)
{
  // Initialize mesh lib, up to 8 models
	LOG_INFO("Calling mesh lib init\n");
	mesh_lib_init(malloc, free, 8);

	// Initialize Low Power Node functionality
	LOG_INFO("Calling lpn_init\n");
	lpn_init();
}
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "5823Pub %02x%02x", pAddr->addr[1], pAddr->addr[0]);

  LOG_INFO("Device name: '%s'\r\n", name);


  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res) {
    LOG_INFO("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }

  // show device name on the LCD
  displayPrintf(DISPLAY_ROW_BTADDR,name);
}

void initiate_factory_reset(void)
{
  LOG_INFO("factory reset\r\n");
  displayPrintf(DISPLAY_ROW_BTADDR,"");
  displayPrintf(DISPLAY_ROW_BTADDR2,"***FACTORY RESET***");

  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_le_connection_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, timerHandle, 1);
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

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	uint16 result;

	if (NULL == evt) {
		return;
	}

	switch (evt_id)
	{
	case gecko_evt_system_boot_id:
		LOG_INFO("entered boot id\n");
		displayPrintf(DISPLAY_ROW_NAME,"Low Power Node");

		// check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
		if(GPIO_PinInGet(buttonPort,buttonPin)==0)
		{
			LOG_INFO("Entered If condition");
			initiate_factory_reset();
		}
		else
		{
			struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

			set_device_name(&pAddr->address);

			// Initialize Mesh stack in Node operation mode, it will generate initialized event
			result = gecko_cmd_mesh_node_init()->result;
			if (result) {
				LOG_ERROR("init failed (0x%x)", result);
			}
		}
		break;


	case gecko_evt_hardware_soft_timer_id:
		LOG_INFO("");
		int res5;
		if(evt->data.evt_hardware_soft_timer.handle==timerHandle)
		{
			// reset the device to finish factory reset
			LOG_INFO("Just Before reset");
			gecko_cmd_system_reset(0);
		}

		if(evt->data.evt_hardware_soft_timer.handle==frienshipFailedHandle)
		{
			res5 = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

			if (res5 != 0) {
				LOG_INFO("ret.code %x\r\n", res5);
			}
		}
		break;

	case gecko_evt_mesh_node_initialized_id:
		LOG_INFO("node initialized\r\n");

		// Initialize generic client models
		gecko_cmd_mesh_generic_client_init();

		struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

		if (pData->provisioned)
		{
			LOG_INFO("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

			_my_address = pData->address;
			_elem_index = 0;   // index of primary element is zero. This example has only one element.

			//LOG_INFO("enabling button interrupts\n");
			//enable_button_interrupts();
			switch_node_init();

			displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioned");
		}
		else
		{
			LOG_INFO("node is unprovisioned\r\n");
			displayPrintf(DISPLAY_ROW_CONNECTION,"unprovisioned");

			LOG_INFO("starting unprovisioned beaconing...\r\n");
			gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
		}
		break;

	case gecko_evt_mesh_node_provisioning_started_id:
		LOG_INFO("Started provisioning\r\n");
		displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioning...");
		break;

	case gecko_evt_mesh_node_provisioned_id:
		_elem_index = 0;   // index of primary element is zero. This example has only one element.
		switch_node_init();
		LOG_INFO("node provisioned, got address=%x\r\n", evt->data.evt_mesh_node_provisioned.address);
		displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioned");
		break;

	case gecko_evt_mesh_node_provisioning_failed_id:
		LOG_INFO("provisioning failed, code %x\r\n", evt->data.evt_mesh_node_provisioning_failed.result);
		displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioning failed");
		/* start a one-shot timer that will trigger soft reset after small delay */
		gecko_cmd_hardware_set_soft_timer(2 * 32768, timerHandle, 1);
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
		num_connections++;
		conn_handle = evt->data.evt_le_connection_opened.connection;
		gecko_cmd_mesh_lpn_deinit();
		displayPrintf(DISPLAY_ROW_CONNECTION,"LPN Off");
		break;

	case gecko_evt_le_connection_closed_id:
		/* Check if need to boot to dfu mode */
		if (boot_to_dfu) {
			/* Enter to DFU OTA mode */
			gecko_cmd_system_reset(2);
		}

		LOG_INFO("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
		conn_handle = 0xFF;
		if (num_connections > 0) {
			if (--num_connections == 0) {
				displayPrintf(DISPLAY_ROW_CONNECTION,"");
				lpn_init();
			}
		}
		break;

	case gecko_evt_mesh_node_reset_id:
		LOG_INFO("evt gecko_evt_mesh_node_reset_id\r\n");
		initiate_factory_reset();
		break;

	case gecko_evt_le_connection_parameters_id:
		LOG_INFO("connection params: interval %d, timeout %d\r\n", evt->data.evt_le_connection_parameters.interval,
				evt->data.evt_le_connection_parameters.timeout
		);
		break;

	case gecko_evt_le_gap_adv_timeout_id:
		LOG_INFO("entered adv_timeout\n");
		// these events silently discarded
		break;

	case gecko_evt_gatt_server_user_write_request_id:
		LOG_INFO("entered write_req id\n");
		if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
			/* Set flag to enter to OTA mode */
			boot_to_dfu = 1;
			/* Send response to Write Request */
			gecko_cmd_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control,
					bg_err_success);

			/* Close connection to enter to DFU OTA mode */
			gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
		}
		break;
	case gecko_evt_system_external_signal_id:
			LOG_INFO("Entered External Signal Id");
			//struct mesh_generic_request req;
			struct mesh_generic_state req;
			uint16 resp;

			//req.kind= mesh_generic_request_on_off;
			req.kind= mesh_generic_state_level;
			req.level.level=0x05;

			//req.kind = mesh_lighting_request_lightness_actual;
			//req.lightness = 0x44;
			//req.on_off=0x01;
			if (evt->data.evt_system_external_signal.extsignals & bt_risingEdgeInt)
			{
				bt_event &= ~bt_risingEdgeInt;
				LOG_INFO("Rising Edge Interrupt Received");
				//req.on_off=MESH_GENERIC_ON_OFF_STATE_ON;
				resp = mesh_lib_generic_client_publish(
						MESH_GENERIC_LEVEL_CLIENT_MODEL_ID,
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
				//req.on_off=MESH_GENERIC_ON_OFF_STATE_ON;
				resp = mesh_lib_generic_client_publish(
						MESH_GENERIC_LEVEL_CLIENT_MODEL_ID,
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


	case gecko_evt_mesh_lpn_friendship_established_id:
		LOG_INFO("friendship established\r\n");
		displayPrintf(DISPLAY_ROW_CONNECTION,"LPN");
		break;

	case gecko_evt_mesh_lpn_friendship_failed_id:
		LOG_INFO("friendship failed\r\n");
		displayPrintf(DISPLAY_ROW_CONNECTION,"No Friend");
		LOG_INFO("%d",evt->data.evt_mesh_lpn_friendship_failed.reason);
		// try again in 2 seconds
		result  = gecko_cmd_hardware_set_soft_timer(2*32768, frienshipFailedHandle, 1)->result;
		if (result) {
			LOG_INFO("timer failure?!  %x\r\n", result);
		}
		break;

	case gecko_evt_mesh_lpn_friendship_terminated_id:
		LOG_INFO("friendship terminated\r\n");
		LOG_INFO("%d",evt->data.evt_mesh_lpn_friendship_terminated.reason);
		//DI_Print("friend lost", DI_ROW_LPN);
		displayPrintf(DISPLAY_ROW_CONNECTION,"Friend Lost");
		if (num_connections == 0) {
			// try again in 2 seconds
			result  = gecko_cmd_hardware_set_soft_timer(2*32768, frienshipFailedHandle, 1)->result;
			if (result) {
				LOG_INFO("timer failure?!  %x\r\n", result);
			}
		}
		break;


      default:
        //printf("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
        break;
    }

}
