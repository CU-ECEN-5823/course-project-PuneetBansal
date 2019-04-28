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
#include "i2c.h"
#include "mesh_generic_model_capi_types.h"


extern uint8_t data_buffer_humid[2];
extern uint8_t data_buffer_aqi[2];
extern uint8_t DataToSend;
extern uint8_t DataToSend1;
extern uint8_t Data_Humid;

extern void sendDataToFriend(uint16_t data, uint16_t delay);
extern void sendThreshToFriend(uint8_t on_off, uint16_t delay);
extern void storePersistentData(uint16_t KEY , uint16_t maxVal);

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
	uint8_t command[2]={ 0x01, 0x18}; // Setting the address to Measure mode register  (0x01) and writing 0x18 to it.
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
	CORE_irqState_t irqState;
	printf("Entered humidity state machine\n");
	if(aqiStateActive!=1)
	{
		humidStateActive=1;
		switch(prevState)
		{
		case powerOn_humid:					//Sensor turned on after 80ms delay.
			irqState=CORE_EnterCritical();
			if(event==takeReading)
			{
				event = None_humid;
				LOG_INFO("Starting I2C write");
				Data_Humid = NO_HOLD_MASTER_HUMID;
				i2c_IntBasedWrite(SLAVE_ADD_HUMID,1);
				presentState=writeState;
			}
			CORE_ExitCritical(irqState);
			break;

		case writeState:				// writing the command successful. Now performing read
			irqState=CORE_EnterCritical();
			if(event==transferComplete)
			{
				event = None_humid;
				LOG_INFO("Write complete, Starting I2C Read");
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
			CORE_ExitCritical(irqState);
			break;

		case readState:					//Read command successful. Now reading the value from buffer.
			irqState=CORE_EnterCritical();
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

				/* if the value is maximum value in a power cycle, store the value in persistent data*/
				if(RLhumid>maxHumid)
				{
					maxHumid=RLhumid;
					storePersistentData(HUMID_KEY,maxHumid);
				}

				sendThreshToFriend(1,1); 	 //Sending the value identifier via on/offmodel
				sendDataToFriend(RLhumid,1); //Sending the data to friend node via level model.
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
			CORE_ExitCritical(irqState);
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

void aqi_stateMachine1()
{
	CORE_irqState_t irqState;
	if(humidStateActive!=1)
	{
		aqiStateActive=1;
		switch(aqi_prevState)
		{
		irqState=CORE_EnterCritical();
		case configured:				// CCS 811 sensor turned on, configured the mode in which it is to be used
			if(aqi_event == dataReady)
			{
				aqi_event=None_aqi;
				DataToSend=ALG_RES;
				#if LPM_AQI
				GPIO_PinOutClear(WAKE_PIN_PORT,WAKE_PIN); //Turning on the sensor
				#endif
				i2c_IntBasedWriteRead(SLAVE_ADD_AQI,1,2);
				aqi_presentState=getReading;
			}
			else
			{
				aqi_event=None_aqi;
				aqi_presentState=configured;
				NVIC_DisableIRQ(I2C0_IRQn);
			}
			CORE_ExitCritical(irqState);
			break;

		case getReading:				// Writing the register add successful, now reading the data from buffer
			irqState=CORE_EnterCritical();
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

				sendThreshToFriend(0,0); //Sending sensor type to the friend using generic on/off model.
				sendDataToFriend(ppm,0); //Sending ppm data to friend using level model.

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
			CORE_ExitCritical(irqState);
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





