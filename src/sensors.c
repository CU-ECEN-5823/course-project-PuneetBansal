/*
 * @filename	: sensors.c
 * @description	: This file contains functions to obtain humidity value from si7021 and AQI from CCS811 sensor
 * @author 		: Puneet Bansal
 * @reference	: Silicon Labs SDK -https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 * 				  si7021 datasheet -https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
 * 				  CCS 811 Programming Reference Guide: https://cdn.sparkfun.com/datasheets/BreakoutBoards/CCS811_Programming_Guide.pdf
 */
#include <src/sensors.h>
#include "main.h"
#include "i2c_interrupt.h"
#include "letimer.h"
#include "mesh_generic_model_capi_types.h"


extern uint8_t data_buffer_humid[2];
extern uint8_t data_buffer_aqi[2];
extern uint8_t data_buffer_alg[2];
extern uint8_t DataToSend;
extern uint8_t DataToSend1;
extern uint8_t Data_Humid;



///**
// * @brief Sending data to friend using Level Model
// * @param data to send to friend
// */
extern void sendDataToFriend(uint16_t data, uint16_t delay);
//{
//	struct mesh_generic_state req;
//	uint16 resp,resp1;
//	req.kind= mesh_generic_state_level;
//	req.level.level=data;
//
//		resp = mesh_lib_generic_client_publish(
//				MESH_GENERIC_LEVEL_CLIENT_MODEL_ID,
//				element_index,
//				transaction_id,
//				&req,
//				0,     // transition
//				0,
//				0     // flags
//		);
//		transaction_id++;
//		if (resp) {
//			printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
//		} else {
//			printf("request sent, trid = %u, delay = %d\r\n", transaction_id, 0);
//		}
//
//}
//
///**
// * @brief: Sendinf on/ off updates to friend
// * @param on_off
// */
extern void sendThreshToFriend(uint8_t on_off, uint16_t delay);
//{
//	struct mesh_generic_request req1;
//	uint16 resp1;
//	req1.kind= mesh_generic_request_on_off;
//	req1.on_off=on_off;
//
//	resp1 = mesh_lib_generic_client_publish(
//			MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
//			element_index1,
//			transaction_id1,
//			&req1,
//			0,     // transition
//			0,
//			0     // flags
//	);
//	transaction_id1++;
//	if (resp1) {
//		printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp1);
//	} else {
//		printf("request sent, trid = %u, delay = %d\r\n", transaction_id1, 0);
//	}
//}
//
///**
// * @brief storing the maximum value of humidity and AQI in the flash memory
// * @param KEY to distinguish between humidity and AQI
// * @param maxVal Maximum value received from the sensors
// */
extern void storePersistentData(uint16_t KEY , uint16_t maxVal);
//{
//	int rsp;
//	uint8_t * val_data;
//	val_data =&maxVal;
//	rsp=gecko_cmd_flash_ps_save(KEY, sizeof(maxVal),val_data)->result;
//	LOG_INFO("%s in store persistent data (returned %d)",rsp==0 ? "Success" : "Error",rsp);
//}
//
///**
// * @brief Loading the data from flash memory corresponding to the key passed
// * @param KEY is the flash memory address from which the data needs to be loaded.
// * @return
// */
//
//extern uint16_t loadPersistentData(uint16_t KEY);
//{
//	uint16_t data;
//	struct gecko_msg_flash_ps_load_rsp_t* resp;
//	BTSTACK_CHECK_RESPONSE(gecko_cmd_flash_ps_load(KEY));
//	memcpy(&data,&resp->value.data,resp->value.len);
//	LOG_INFO("Persistent data is %d",data);
//	return data;
//}
//
//
///**
// * @brief : displaying the persistent data on LCD
// */
//extern void displayPersistentData();
//{
//	uint16_t persistentAQI, persistentHumid;
//	persistentAQI=loadPersistentData(AQI_KEY);
//	persistentHumid=loadPersistentData(HUMID_KEY);
//	displayPrintf(DISPLAY_ROW_TEMPVALUE,"HUMID %d",persistentHumid);
//	displayPrintf(6,"AQI %d",persistentAQI);
//	LOG_INFO("HUMID %d",persistentHumid);
//	LOG_INFO("AQI %d",persistentAQI);
//}


uint32_t humid_get()
{
	uint32_t RLhumid;
	uint16_t humidValue;
	I2C_TransferSeq_TypeDef init;

	init.addr= SLAVE_ADD_HUMID << 1;
	uint8_t command=NO_HOLD_MASTER_HUMID;
	init.buf[0].data=&command;

	/*Write operation to transfer slave address and type of data required*/
	i2c_write(init,1);
	timerWaitUs(10000);
	/*Read operation to read temperature from Si7021*/

	humidValue=i2c_read(init,2);					 /*Perform the read operation*/

	RLhumid = ((125*humidValue)/65536)-6;
	LOG_INFO("humid: %d",RLhumid);
	return RLhumid;
}

void sensorConfig()
{
	I2C_TransferSeq_TypeDef init;
	uint16_t getval;

	init.addr= SLAVE_ADD_AQI << 1;
	uint16_t command=STATUS_REG;
	init.buf[0].data=&command;

	i2c_write(init,1);
	timerWaitUs(10000);

	getval=i2c_read(init,1);
	LOG_INFO("Status  Register is %x\n",getval);

	if(getval & APP_VALID)
	{
		LOG_INFO("Valid firmware is present. Now writing to the APP_START register\n");
	}

	command=APP_START;
	init.buf[0].data=&command;

	i2c_write(init,1);
	timerWaitUs(10000);

	LOG_INFO("Now checking the status register again \n");

	command=STATUS_REG;
	init.buf[0].data=&command;

	i2c_write(init,1);
	timerWaitUs(10000);

	getval=i2c_read(init,1);
	LOG_INFO("Status  Register is %x\n",getval);

	if(getval & FW_MODE)
	{
		LOG_INFO("Mode changed to firmware mode\n");
	}
}

void changeMode()
{
	//Writing to MEAS_MODE register the mode in which the sensor will operate. Mode 1
	I2C_TransferSeq_TypeDef init;
	uint16_t getval;

	init.addr= SLAVE_ADD_AQI << 1;
	uint8_t command[2]={ 0x01, 0x18}; //0x1001;//MEAS_MODE;
	init.buf[0].data=command;
	i2c_write(init,2);
	timerWaitUs(10000);

	command[0]=MEAS_MODE;
	init.buf[0].data=command;
	i2c_write(init,1);
	timerWaitUs(10000);
	getval=i2c_read(init,1);
	LOG_INFO("Measure  Register is %x\n",getval);

}

uint16_t ppmGet()
{
	I2C_TransferSeq_TypeDef init;
	uint16_t getval;

	init.addr= SLAVE_ADD_AQI << 1;
	uint16_t command=STATUS_REG;
	init.buf[0].data=&command;

	i2c_write(init,1);
	timerWaitUs(10000);

	getval=i2c_read(init,1);
	//LOG_INFO("Status  Register is %x\n",getval);
	if(getval & DATA_READY)
	{
		init.addr= SLAVE_ADD_AQI << 1;
		uint16_t command=ALG_RES;
		init.buf[0].data=&command;

		i2c_write(init,1);
		timerWaitUs(10000);

		getval=i2c_read(init,2);
		LOG_INFO("ppm value is %d\n",getval);
	}
return getval;
}

void humidity_stateMachine()
{
	printf("Entered humidity state machine\n");
	if(aqiStateActive!=1)
	{
		humidStateActive=1;
		switch(prevState)
		{
		case powerOn_humid:					//power turned on
			if(event==takeReading)
			{
				event = None_humid;
				LOG_INFO("Starting I2C write");
				Data_Humid = NO_HOLD_MASTER_HUMID;
				i2c_IntBasedWrite(SLAVE_ADD_HUMID,NO_HOLD_MASTER_HUMID,1);
				presentState=writeState;
			}
			break;

		case writeState:				// write I2C_IRQ
			if(event==transferComplete)
			{
				event = None_humid;
				LOG_INFO("Write complete, Starting I2C Read");
				//humidStateActive=1;
				i2c_IntBasedRead(SLAVE_ADD_HUMID, 2);
				presentState=readState;
			}
			else if(event == transferError)
			{
				event = None_humid;
				LOG_ERROR("write error");
				NVIC_DisableIRQ(I2C0_IRQn);
				presentState=powerOn_humid;
			}
			break;

		case readState:

			if(event==transferComplete)
			{
				uint32_t RLhumid;
				event = None_humid;
				RLhumid=0;

				LOG_INFO("I2C Read complete");
				humid_value = data_buffer_humid[0];
				humid_value <<= 8;
				humid_value	|= (data_buffer_humid[1]);

				RLhumid = ((125*humid_value)/65536)-6;
				LOG_INFO("humid: %d",RLhumid);
				displayPrintf(7,"Humid : %d",RLhumid);

				if(RLhumid>maxHumid)
				{
					maxHumid=RLhumid;
					storePersistentData(HUMID_KEY,maxHumid);
				}
				sendThreshToFriend(1,1); 	 //Sending the value identifier via on/offmodel
				sendDataToFriend(RLhumid,1); //Sending the data to friend node via level model.
				//if(RLhumid > HUMID_THRESH)
				//{

				//}
				NVIC_DisableIRQ(I2C0_IRQn);
				presentState=powerOn_humid;
			}
			else if(event == transferError)
			{
				event = None_humid;
				LOG_ERROR("read error");
				presentState=powerOn_humid;
				NVIC_DisableIRQ(I2C0_IRQn);
			}
			humidStateActive=0;
			break;
		default:
			LOG_ERROR("Unknown State");
			break;

		}

		if(prevState != presentState)
		{
			LOG_INFO("Changing State from: %d to %d with Event:%d", prevState, presentState,event);
			prevState=presentState;
		}
	}

}


void aqi_stateMachine()
{
	printf("Entered aqi state machine\n");
	if(humidStateActive!=1)
	{
		aqiStateActive=1;
		switch(aqi_prevState)
		{

		case powerOff_aqi:
			DataToSend= STATUS_REG;
			i2c_IntBasedWriteRead(SLAVE_ADD_AQI,STATUS_REG,1,1);
			aqi_presentState= initialised;
			break;
		case initialised:
			LOG_INFO("Inside initialised state");
			if(aqi_event==aqi_transferComplete)
			{
				aqi_event=None_aqi;
				LOG_INFO("Status register in state 1 is %x\n",data_buffer_aqi[0]);
				if(data_buffer_aqi[0] & APP_VALID)
				{
					//data_buffer_alg[0]=0;
					LOG_INFO("Writing to the app start register");
					DataToSend=APP_START;
					//aqiStateActive=1;
					i2c_IntBasedWrite(SLAVE_ADD_AQI,APP_START,1);
					aqi_presentState= appStarted;
				}
			}
			else if(aqi_event == aqi_transferError)
			{
				aqi_event = None_aqi;
				LOG_ERROR("Error reading the data from status register in Event 1");
				aqi_presentState=powerOff_aqi;
				NVIC_DisableIRQ(I2C0_IRQn);
			}
			break;

		case appStarted:
			LOG_INFO("Inside app started state");
			if(aqi_event == aqi_transferComplete)
			{
				aqi_event=None_aqi;
				DataToSend=STATUS_REG;
				//aqiStateActive=1;
				i2c_IntBasedWriteRead(SLAVE_ADD_AQI,STATUS_REG,1,1);
				aqi_presentState=modeConfiguring;
			}
			else if(aqi_event == aqi_transferError)
			{
				aqi_event = None_aqi;
				LOG_ERROR("Error writing the data to app valid register");
				aqi_presentState=powerOff_aqi;
				NVIC_DisableIRQ(I2C0_IRQn);
			}
			break;

		case modeConfiguring:
			LOG_INFO("Inside mode configuring state");
			if(aqi_event == aqi_transferComplete)
			{
				LOG_INFO("Status register in state 3 is %x\n",data_buffer_aqi[0]);
				if(data_buffer_aqi[0] & FW_MODE)
				{
					aqi_event=None_aqi;
					DataToSend=MEAS_MODE;
					DataToSend1=0x18;
					//aqiStateActive=1;
					i2c_IntBasedWriteWrite(SLAVE_ADD_AQI,MEAS_MODE,0x18,1);
					//i2c_IntBasedWrite(SLAVE_ADD_AQI,0x1018,2);
					aqi_presentState=configured;
				}
			}
			else if(aqi_event == aqi_transferError)
			{
				aqi_event = None_aqi;
				LOG_ERROR("Error reading the data from status register");
				aqi_presentState=powerOff_aqi;
				NVIC_DisableIRQ(I2C0_IRQn);
			}
			break;

		case configured:
			if(aqi_event == aqi_transferComplete || passed==1 )/*dataReady )*/
			{
				//			/*************************Checking the meas mode register***********************************/
				//			LOG_INFO("Now checking the value of MEAS MODE register");
				//
				//			I2C_TransferSeq_TypeDef init;
				//				uint16_t getval;
				//
				//				init.addr= SLAVE_ADD_AQI << 1;
				//				uint16_t command=MEAS_MODE;
				//				init.buf[0].data=&command;
				//
				//				getval=i2c_read(init,1);
				//				LOG_INFO("Measure mode register is %x\n",getval);
				//				//timerWaitUs(10000);

				/******************************************************************************************/
				passed=1;
				if(data_ready_check==1)
				{
					data_ready_check=0;
					aqi_event=None_aqi;
					//i2c_IntBasedWriteRead(SLAVE_ADD_AQI,ALG_RES,1,1);
					DataToSend=ALG_RES;
					//aqiStateActive=1;
					i2c_IntBasedWrite(SLAVE_ADD_AQI,ALG_RES,1);
					aqi_presentState=getReading;
				}
				else
				{
					LOG_INFO("Data not ready in status register");
					NVIC_DisableIRQ(I2C0_IRQn);
					aqi_presentState=configured;

				}
			}
			else if(aqi_event== aqi_transferError)
			{
				aqi_event = None_aqi;
				aqi_presentState=powerOff_aqi;
				LOG_INFO("Write write transfer fail");
				NVIC_DisableIRQ(I2C0_IRQn);
			}
			break;

		case getReading:
			if(aqi_event == aqi_transferComplete)
			{
				aqi_event=None_aqi;
				//aqiStateActive=1;
				i2c_IntBasedRead(SLAVE_ADD_AQI,2);
				aqi_presentState = getReading1;

				/***********************OLD IMPLEMENTATION**************************/
				//			LOG_INFO("value in data buffer aqi[0] is %d",data_buffer_alg[0]);
				//			LOG_INFO("value in data buffer aqi[1] is %d",data_buffer_alg[1]);
				//			ppm=data_buffer_alg[0];
				//			ppm <<= 8;
				//			ppm |= data_buffer_alg[1];
				//			LOG_INFO("ppm is %d",ppm);
				//			aqi_presentState=configured;
				/*********************************************************************/
			}
			else if(aqi_event == aqi_transferError)
			{
				aqi_event = None_aqi;
				LOG_ERROR("Error writing to the alg_res register");
				aqi_presentState=configured;
				NVIC_DisableIRQ(I2C0_IRQn);
			}
			break;

		case getReading1:
			if(aqi_event == aqi_transferComplete)
			{
				uint16_t ppm=0;

				aqi_event = None_aqi;
				LOG_INFO("value in data buffer aqi[0] is %d",data_buffer_aqi[0]);
				LOG_INFO("value in data buffer aqi[1] is %d",data_buffer_aqi[1]);
				ppm=data_buffer_aqi[0];
				ppm <<= 8;
				ppm |= data_buffer_aqi[1];
				LOG_INFO("ppm is %d",ppm);

				/*********************Storing persistent data and sending data to friend*********/
				if(ppm>maxAqi)
				{
					storePersistentData(AQI_KEY,maxAqi);
					maxAqi=ppm;
				}
				sendDataToFriend(ppm,2); //Sending the data to friend node via level model.
				if(ppm> AQI_THRESH)
				{
				sendThreshToFriend(1,2);
				}

				/**********************************************************************************/

				aqi_presentState=configured;
				NVIC_DisableIRQ(I2C0_IRQn);

			}
			else if(aqi_event == aqi_transferError)
			{
				aqi_event = None_aqi;
				LOG_ERROR("Error reading the data from al_res register");
				aqi_presentState=configured;
				NVIC_DisableIRQ(I2C0_IRQn);
			}
			aqiStateActive=0;
			break;

		default:
			LOG_ERROR("Unknown State");
			break;
		}

		if(aqi_prevState != aqi_presentState)
		{
			LOG_INFO("AQI Changing State from: %d to %d with Event:%d", aqi_prevState, aqi_presentState,aqi_event);
			aqi_prevState=aqi_presentState;
		}
	}
}

void aqi_stateMachine1()
{
	if(humidStateActive!=1)
	{
		aqiStateActive=1;
		switch(aqi_prevState)
		{
		case configured:
			if(aqi_event == dataReady/*data_ready_check==1*/)
			{
				//data_ready_check=0;
				aqi_event=None_aqi;
				DataToSend=ALG_RES;
				#if LPM_AQI
				GPIO_PinOutClear(WAKE_PIN_PORT,WAKE_PIN); //Turning on the sensor
				#endif
				i2c_IntBasedWriteRead(SLAVE_ADD_AQI,ALG_RES,1,2);
				aqi_presentState=getReading;
			}
			else /*if(data_ready_check==0)*/
			{
				aqi_event=None_aqi;
				aqi_presentState=configured;
				NVIC_DisableIRQ(I2C0_IRQn);
			}
			break;

		case getReading:
			if(aqi_event == aqi_transferComplete)
			{
				uint16_t ppm=0;
				aqi_event = None_aqi;
				LOG_INFO("value in data buffer aqi[0] is %d",data_buffer_aqi[0]);
				LOG_INFO("value in data buffer aqi[1] is %d",data_buffer_aqi[1]);
				ppm=data_buffer_aqi[0];
				ppm <<= 8;
				ppm |= data_buffer_aqi[1];
				LOG_INFO("ppm is %d",ppm);
				displayPrintf(6,"AQI : %d",ppm);
				#if LPM_AQI
				GPIO_PinOutSet(WAKE_PIN_PORT,WAKE_PIN); //Turning off the sensor
				#endif

				sendThreshToFriend(0,0);
				sendDataToFriend(ppm,0);

				if(ppm>maxAqi)
				{
					storePersistentData(AQI_KEY,ppm);
					maxAqi=ppm;
				}
				aqi_presentState = configured;
			}
			else if(aqi_event == aqi_transferError)
			{
				aqi_event = None_aqi;
				aqi_presentState = configured;
				NVIC_DisableIRQ(I2C0_IRQn);

			}
			aqiStateActive=0;
			break;

		default:
			LOG_ERROR("Unknown State");
			break;
		}

		if(aqi_prevState != aqi_presentState)
		{
			LOG_INFO("AQI Changing State from: %d to %d with Event:%d", aqi_prevState, aqi_presentState,aqi_event);
			aqi_prevState=aqi_presentState;
		}

	}

}

void stateCheck()
{
	switch(aqi_prevState)
	{
	case powerOff_aqi:
		LOG_INFO("Writing address of MEAS MODE reg\n");
		aqi_event= None_aqi;
		DataToSend=MEAS_MODE;
		aqiStateActive=1;
		i2c_IntBasedWrite(SLAVE_ADD_AQI,MEAS_MODE,1);
		aqi_presentState = writingData;
		break;
	case writingData:
		if(aqi_event == aqi_transferComplete)
		{
			LOG_INFO("Performing write data to meas mode\n");
			aqi_event= None_aqi;
			DataToSend=0X18;
			aqiStateActive=1;
			i2c_IntBasedWrite(SLAVE_ADD_AQI,0x18,1);
			aqi_presentState = writingIntermediate;
		}
		else if (aqi_event == aqi_transferError)
		{
			LOG_INFO("Writing address of meas mode failed \n");
			aqi_presentState = powerOff_aqi;
			aqi_event= None_aqi;
			NVIC_DisableIRQ(I2C0_IRQn);
		}
		break;

	case writingIntermediate:
		if(aqi_event == aqi_transferComplete)
		{
			LOG_INFO("Initiating read\n");
			aqi_event= None_aqi;
			DataToSend=MEAS_MODE;
			aqiStateActive=1;
			i2c_IntBasedWrite(SLAVE_ADD_AQI,MEAS_MODE,1);
			aqi_presentState = initialised;
		}
		else if (aqi_event == aqi_transferError)
		{
			LOG_INFO("Writing data failed\n");
			aqi_presentState = powerOff_aqi;
			aqi_event= None_aqi;
			NVIC_DisableIRQ(I2C0_IRQn);
		}
		break;

	case initialised:

		if(aqi_event == aqi_transferComplete)
		{
			LOG_INFO("Performing read from meas mode\n");
			aqi_event= None_aqi;
			aqiStateActive=1;
			i2c_IntBasedRead(SLAVE_ADD_AQI,1);
			aqi_presentState = appStarted;

		}
		else if (aqi_event == aqi_transferError)
		{
			LOG_INFO("Write failed\n");
			aqi_presentState = powerOff_aqi;
			aqi_event= None_aqi;
			NVIC_DisableIRQ(I2C0_IRQn);
		}
		break;
	case appStarted:
		if(aqi_event == aqi_transferComplete)
		{
			LOG_INFO("Meas mode register is %x\n",data_buffer_aqi[0]);
			aqi_presentState = powerOff_aqi;
			aqi_event= None_aqi;
			NVIC_DisableIRQ(I2C0_IRQn);
		}
		else if (aqi_event == aqi_transferError)
		{
			LOG_INFO("Read failed\n");
			aqi_presentState = powerOff_aqi;
			aqi_event= None_aqi;
			NVIC_DisableIRQ(I2C0_IRQn);
		}
		break;

	default:
			LOG_ERROR("Unknown State");
			break;
	}
	if(aqi_prevState != aqi_presentState)
	{
		aqi_prevState=aqi_presentState;
	}
}

//void humidityStateMachineLPM()
//{
//	humidStateActive=1;
//	switch(prevState)
//	{
//
//	case idle:			// power off
//
//		if(event == Comp0Underflow)
//		{
//			event = None_humid;
//			GPIO_PinOutSet(SENSOR_ENABLE_PORT,SENSOR_ENABLE_PIN);
//			LOG_INFO("Got COMP0. Setting comp1");
//			timerSetEventinms(80); 			/*Wait for 80 ms*/
//			presentState=initState;
//		}
//
//		break;
//
//	case initState:					//power turned on
//		if(event==Comp1Interrupt)
//		{
//			event = None_humid;
//			LOG_INFO("Starting I2C write");
//			//SLEEP_SleepBlockBegin(sleepEM2);
//			Data_Humid= NO_HOLD_MASTER_HUMID;
//			i2c_IntBasedWrite(SLAVE_ADD_HUMID,NO_HOLD_MASTER_HUMID,1);
//			presentState=writeState;
//		}
//		break;
//
//	case writeState:				// write I2C_IRQ
//		if(event==transferComplete)
//		{
//			event = None_humid;
//			LOG_INFO("Write complete, Starting I2C Read");
//			i2c_IntBasedRead(SLAVE_ADD_HUMID,2);
//			presentState=readState;
//		}
//		else if(event == transferError)
//		{
//			event = None_humid;
//			LOG_ERROR("write error");
//			//SLEEP_SleepBlockEnd(sleepEM2);
//			NVIC_DisableIRQ(I2C0_IRQn);
//			presentState=idle;
//		}
//		break;
//
//	case readState:
//
//		if(event==transferComplete)
//		{
//			uint32_t RLhumid;
//			uint16_t temp_value;
//			event = None_humid;
//			RLhumid=0;
//			LOG_INFO("I2C Read complete");
//			temp_value = data_buffer_humid[0];
//			temp_value <<= 8;
//			temp_value	|= (data_buffer_humid[1]);
//
//			RLhumid = ((125*temp_value)/65536)-6;
//			LOG_INFO("humid: %d",RLhumid);
//			//float tempDeg = (((temp_value*175.72)/65536)-46.85); /*Convert the temperature value in degree*/
//			//LOG_INFO("Temp %.02f",tempDeg);
//			GPIO_PinOutClear(SENSOR_ENABLE_PORT,SENSOR_ENABLE_PIN);
//			//sendDataToFriend(RLhumid,0); //sending data to friend node
//			//SLEEP_SleepBlockEnd(sleepEM2);
//			humidStateActive=0;
//			NVIC_DisableIRQ(I2C0_IRQn);
//			presentState=idle;
//		}
//		else if(event == transferError)
//		{
//			event = None_humid;
//			LOG_ERROR("read error");
//			presentState=idle;
//			//SLEEP_SleepBlockEnd(sleepEM2);
//			NVIC_DisableIRQ(I2C0_IRQn);
//		}
//		break;
//	default:
//		LOG_ERROR("Unknown State");
//		break;
//
//	}
//
//	if(prevState != presentState){
//		LOG_INFO("Changing State from: %d to %d with Event:%d", prevState, presentState,event);
//		prevState=presentState;
//	}
//
//}





