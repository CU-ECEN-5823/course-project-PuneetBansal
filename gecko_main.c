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
#include <src/sensors.h>
#include "mesh_lib.h"

#include "mesh_lighting_model_capi_types.h"


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
		.sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
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

/********************************GLOBAL VARIABLES FOR gecko_main.c*****************************/
static uint8 conn_handle = 0xFF;
static uint16 _elem_index = 0xffff;
static uint16 _my_address = 0;
static uint8 num_connections = 0;

uint16_t element_index = 0;
uint8_t transaction_id = 0;
uint16_t element_index1 = 0;
uint8_t transaction_id1 = 0;


/*********************************************************************************************/

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

void gecko_bgapi_classes_init_server_friend(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	gecko_bgapi_class_mesh_node_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	gecko_bgapi_class_mesh_generic_server_init();
	gecko_bgapi_class_mesh_lpn_init();
}

void gecko_bgapi_classes_init_client_lpn(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	gecko_bgapi_class_mesh_node_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_lpn_init();
}

void lpn_init(void)
{
  uint16 res;
  // Initialize LPN functionality.
  BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_init());

  // Configure the lpn with following parameters:
  // - Minimum friend queue length = 2
  // - Poll timeout = 5 seconds

  BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_configure(2, 6 * 1000));
  LOG_INFO("trying to find friend...\r\n");
  BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_establish_friendship(0));

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

/**
 * @ brief setting device name based on input bluetooth address
 * @param pAddr
 */
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "5823Pub %02x%02x", pAddr->addr[1], pAddr->addr[0]);

  LOG_INFO("Device name: '%s'\r\n", name);

  // write device name to the GATT database
  BTSTACK_CHECK_RESPONSE(gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name));

  // show device name on the LCD
  displayPrintf(DISPLAY_ROW_BTADDR,name);
}

/**
 * @brief Clearing flash memory and calling a timer to reset after 2 seconds
 */
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
  BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer(2 * 32768, timerHandle, 1));
}

/**
 * @brief Sending data to friend using Level Model
 * @param data to send to friend
 */
void sendDataToFriend(uint16_t data, uint16_t delay)
{
	struct mesh_generic_state req;
	uint16 resp,resp1;
	req.kind= mesh_generic_state_level;
	req.level.level=data;

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

/**
 * @brief: Sendinf on/ off updates to friend
 * @param on_off
 */
void sendThreshToFriend(uint8_t on_off,uint16_t delay)
{
	struct mesh_generic_request req1;
	uint16 resp1;
	req1.kind= mesh_generic_request_on_off;
	req1.on_off=on_off;

	resp1 = mesh_lib_generic_client_publish(
			MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
			element_index1,
			transaction_id1,
			&req1,
			0,     // transition
			0,
			0     // flags
	);
	transaction_id1++;
	if (resp1) {
		printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp1);
	} else {
		printf("request sent, trid = %u, delay = %d\r\n", transaction_id1, 0);
	}
}

/**
 * @brief storing the maximum value of humidity and AQI in the flash memory
 * @param KEY to distinguish between humidity and AQI
 * @param maxVal Maximum value received from the sensors
 */
void storePersistentData(uint16_t KEY , uint16_t maxVal)
{
	int rsp;
	uint8_t * val_data;
	val_data =&maxVal;
	rsp=gecko_cmd_flash_ps_save(KEY, sizeof(maxVal),val_data)->result;
	LOG_INFO("%s in store persistent data (returned %d)",rsp==0 ? "Success" : "Error",rsp);
}

/**
 * @brief Loading the data from flash memory corresponding to the key passed
 * @param KEY is the flash memory address from which the data needs to be loaded.
 * @return
 */

uint16_t loadPersistentData(uint16_t KEY)
{
	uint16_t data;
	struct gecko_msg_flash_ps_load_rsp_t* resp;
	resp=(gecko_cmd_flash_ps_load(KEY));
	memcpy(&data,&resp->value.data,resp->value.len);
	LOG_INFO("Persistent data is %d",data);
	return data;
}


/**
 * @brief : displaying the persistent data on LCD
 */
void displayPersistentData()
{
	uint16_t persistentAQI, persistentHumid;
	persistentAQI=loadPersistentData(AQI_KEY);
	persistentHumid=loadPersistentData(HUMID_KEY);
	displayPrintf(9,"PERSISTENT DATA :");
	displayPrintf(11,"HUMID %d",persistentHumid);
	displayPrintf(10,"AQI %d",persistentAQI);
	LOG_INFO("HUMID %d",persistentHumid);
	LOG_INFO("AQI %d",persistentAQI);
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
			BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_node_init());/*gecko_cmd_mesh_node_init_oob(0X00,0X03,0X03,0x08,0x00,0x04,0x01)->result;*/
			/*I was trying the out of band authentication and it was not working, so I commented it out for now.*/

		}
		break;

	case gecko_evt_hardware_soft_timer_id:
		LOG_INFO("");
		int res5;
		uint32_t relHumid;
		uint16_t ppm;
		if(evt->data.evt_hardware_soft_timer.handle==timerHandle)
		{
			// reset the device to finish factory reset
			LOG_INFO("Just Before reset");
			gecko_cmd_system_reset(0);
		}

		if(evt->data.evt_hardware_soft_timer.handle==frienshipFailedHandle)
		{
			BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_establish_friendship(0));
		}
		if(evt->data.evt_hardware_soft_timer.handle==sensorReading)
		{
			LOG_INFO("Humidity Timeout");


#if NON_BLOCKING_IMPLEMENTATION
			//event = Comp0Underflow;//to remove
			aqi_event= dataReady;
			event=takeReading; //to add

#else if
			/************************This was the blocking working version of Humidity******************************/
			GPIO_PinOutSet(SENSOR_ENABLE_PORT,SENSOR_ENABLE_PIN);
			timerWaitUs(80000);
			relHumid=humid_get();

			if(relHumid>maxHumid)
			{
				maxHumid=relHumid;
				storePersistentData(HUMID_KEY,maxHumid);
			}

			sendDataToFriend(relHumid,0); //Sending the data to friend node via level model.
			/***********************Blocking version of AQI*************************************/
			//GPIO_PinOutClear(SENSOR_ENABLE_PORT,SENSOR_ENABLE_PIN);//Cannot do lpm for humidity since lcd also operates on same pin.

			/*Enabling Load Power Management for the CCS811 sensor. The sensor is active only when the wake pin is low*/
			//GPIO_PinOutClear(WAKE_PIN_PORT,WAKE_PIN); //Turning on the sensor
			ppm=ppmGet();
			sendDataToFriend(ppm,0);
			if(ppm>maxAqi)
			{
				LOG_INFO("Entered if of maxaqi\n");
				maxAqi=ppm;
				storePersistentData(AQI_KEY,maxAqi);
			}
			sendDataToFriend(ppm,0);
			//GPIO_PinOutSet(WAKE_PIN_PORT,WAKE_PIN);	//Turning off the sensor.
#endif
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

			switch_node_init();
			displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioned");
			displayPersistentData();

			BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer(5 * 32768, sensorReading, 0));
		}
		else
		{
			LOG_INFO("node is unprovisioned\r\n");
			displayPrintf(DISPLAY_ROW_CONNECTION,"unprovisioned");
			LOG_INFO("starting unprovisioned beaconing...\r\n");
			BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_node_start_unprov_beaconing(0x3));   // enable ADV and GATT provisioning bearer
			displayPersistentData();
		}
		break;

	case gecko_evt_mesh_node_provisioning_started_id:
		LOG_INFO("Started provisioning\r\n");
		displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioning...");
		break;

	/*Was trying to check Out of band authentication functionality. */
	/*case gecko_evt_mesh_node_display_output_oob_id:
		LOG_INFO("");
		break;*/


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
		BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer(2 * 32768, timerHandle, 1));
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
		BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_deinit());
		displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");
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

	/*case gecko_evt_mesh_generic_client_server_status_id:
	{
	To check the functionality of friend queue.
	break;
	}*/


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

	case gecko_evt_mesh_lpn_friendship_established_id:
		LOG_INFO("friendship established\r\n");
		displayPrintf(DISPLAY_ROW_CONNECTION,"LPN");
		//gecko_cmd_hardware_set_soft_timer(2 * 32768, SensorReading, 0); // Can be added further to take sensor reading only after
																		// friendship has been established.

		break;

	case gecko_evt_mesh_lpn_friendship_failed_id:
		LOG_INFO("friendship failed\r\n");
		displayPrintf(DISPLAY_ROW_CONNECTION,"No Friend");
		LOG_INFO("%d",evt->data.evt_mesh_lpn_friendship_failed.reason);
		// try again in 2 seconds
		BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer(2*32768, frienshipFailedHandle,1));
		break;

	case gecko_evt_mesh_lpn_friendship_terminated_id:
		LOG_INFO("friendship terminated\r\n");
		LOG_INFO("%d",evt->data.evt_mesh_lpn_friendship_terminated.reason);
		//DI_Print("friend lost", DI_ROW_LPN);
		displayPrintf(DISPLAY_ROW_CONNECTION,"Friend Lost");
		if (num_connections == 0) {

			// try again in 2 seconds
			BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer(2*32768, frienshipFailedHandle, 1));
		}
		break;

	case gecko_evt_system_external_signal_id:

		if((evt->data.evt_system_external_signal.extsignals) & TRANSFER_COMPLETE_HUMID)
		{
			event = transferComplete;
			bt_event &= ~(TRANSFER_COMPLETE_HUMID);
		}

		if((evt->data.evt_system_external_signal.extsignals) & TRANSFER_COMPLETE_AQI)
		{
			aqi_event = aqi_transferComplete;
			bt_event &= ~(TRANSFER_COMPLETE_AQI);
		}

		if((evt->data.evt_system_external_signal.extsignals) & TRANSFER_FAIL_HUMID)
		{
			event = transferError;
			bt_event &= ~(TRANSFER_FAIL_HUMID);
		}
		if((evt->data.evt_system_external_signal.extsignals) & TRANSFER_FAIL_AQI)
		{
			aqi_event = aqi_transferError;
			bt_event &= ~(TRANSFER_FAIL_AQI);
		}
		if((evt->data.evt_system_external_signal.extsignals) & DATA_READY)
		{
			aqi_event = dataReady;
			bt_event &= ~(DATA_READY);
		}
		if((evt->data.evt_system_external_signal.extsignals) & COMP1_INTERRUPT)
		{
			bt_event = ~ (COMP1_INTERRUPT);
			event=Comp1Interrupt;
		}

		break;


      default:
        break;
    }

}
