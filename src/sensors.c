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
#include "i2c.h"
#include "letimer.h"

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
	uint16_t command=0x1001;//MEAS_MODE;
	init.buf[0].data=&command;
	i2c_write(init,2);
	timerWaitUs(10000);

	command=MEAS_MODE;
	init.buf[0].data=&command;
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








