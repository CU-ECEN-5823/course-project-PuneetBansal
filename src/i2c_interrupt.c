/*
 * @filename	: i2c_interrupt.c
 * @description	: This file contains functions to configure I2C to work in non blocking mode.
 * @author 		: Puneet Bansal
 * @reference	: Silicon Labs SDK -https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 * 				: Lecture slides.
 *
 */

#define INCLUDE_LOG_DEBUG 1
#include "log.h"
#include "i2c_interrupt.h"
#include "main.h"
#include <em_core.h>

uint8_t data_buffer_humid[2] = {0};
uint8_t data_buffer_aqi[2] = {0};
uint8_t data_buffer_alg[2] = {0};
uint8_t DataToSend;
uint8_t DataToSend1;
uint8_t Data_Humid;

/*
 * @decription
 * sets the slave address, flag, command to write to the slave.
 * the function also enables the NVIC interrut for i2c transfers
 *
 */

void i2c_IntBasedWrite(uint8_t SlaveAdd,uint8_t len)
{
	initNonPollingI2c.addr= SlaveAdd << 1;
	initNonPollingI2c.flags= I2C_FLAG_WRITE;
	initNonPollingI2c.buf[0].len=len;
	if(SlaveAdd == SLAVE_ADD_HUMID)
	{
		initNonPollingI2c.buf[0].data=&Data_Humid;
	}
	else if(SlaveAdd == SLAVE_ADD_AQI)
	{
		initNonPollingI2c.buf[0].data=&DataToSend;
	}
	NVIC_EnableIRQ(I2C0_IRQn);
	I2C_TransferReturn_TypeDef ret = I2C_TransferInit(I2C0, &initNonPollingI2c); 		/*Initialising the non polling I2c*/
	if(ret != i2cTransferInProgress){
		LOG_ERROR("Failed");
	}
}

/*
 * @description
 * sets the slave address, flags to read flag and pointer to data buffer to initiate i2c read transfer.
 */


void i2c_IntBasedRead(uint8_t SlaveAdd, uint8_t len)
{
	initNonPollingI2c.addr= SlaveAdd << 1;
	initNonPollingI2c.flags= I2C_FLAG_READ;
	initNonPollingI2c.buf[0].len=len;
	if(SlaveAdd==SLAVE_ADD_HUMID)
	{
	initNonPollingI2c.buf[0].data=data_buffer_humid;
	}
	else if(SlaveAdd==SLAVE_ADD_AQI)
	{
	initNonPollingI2c.buf[0].data=data_buffer_aqi;
	}

//	NVIC_EnableIRQ(I2C0_IRQn);
	I2C_TransferReturn_TypeDef ret = I2C_TransferInit(I2C0, &initNonPollingI2c); 		/*Initialising the non polling I2c*/
	if(ret != i2cTransferInProgress){
		LOG_ERROR("Failed");
	}
}

void i2c_IntBasedWriteRead(uint8_t SlaveAdd,uint8_t writeLen, uint8_t readLen)
{
	initNonPollingI2c.addr= SlaveAdd << 1;
	initNonPollingI2c.flags= I2C_FLAG_WRITE_READ;
	initNonPollingI2c.buf[0].len=writeLen; // setting the length of write buffer
	initNonPollingI2c.buf[1].len=readLen; //setting the length of read buffer

	initNonPollingI2c.buf[0].data= &DataToSend;
	if(SlaveAdd==SLAVE_ADD_HUMID)
	{
		initNonPollingI2c.buf[1].data= data_buffer_humid;
	}
	else if(SlaveAdd==SLAVE_ADD_AQI)
	{
		initNonPollingI2c.buf[1].data= data_buffer_aqi;
	}

	NVIC_EnableIRQ(I2C0_IRQn);
	I2C_TransferReturn_TypeDef ret = I2C_TransferInit(I2C0, &initNonPollingI2c); 		/*Initialising the non polling I2c*/
	if(ret != i2cTransferInProgress){
		LOG_ERROR("Failed");
	}

}

void i2c_IntBasedWriteWrite(uint8_t SlaveAdd ,uint8_t len)
{
	initNonPollingI2c.addr= SlaveAdd << 1;
	initNonPollingI2c.flags= I2C_FLAG_WRITE_WRITE;
	initNonPollingI2c.buf[0].len=len; // setting the length of write buffer
	initNonPollingI2c.buf[1].len=len;
	initNonPollingI2c.buf[0].data= &DataToSend;
	initNonPollingI2c.buf[1].data= &DataToSend1;
	LOG_INFO("Buffer 0 data  is %x",initNonPollingI2c.buf[0].data[0]);
	LOG_INFO("Buffer 1 data  is %x",initNonPollingI2c.buf[1].data[0]);

	NVIC_EnableIRQ(I2C0_IRQn);
	I2C_TransferReturn_TypeDef ret = I2C_TransferInit(I2C0, &initNonPollingI2c); 		/*Initialising the non polling I2c*/
	if(ret != i2cTransferInProgress){
		LOG_ERROR("Failed");
	}

}

/*
 * @description
 * I2c interrupt handler. Returns transfer complete on sucessfull completion of transfer and
 * returns an error otherwise.
 */

void I2C0_IRQHandler(void)
{
	CORE_ATOMIC_IRQ_DISABLE();
	I2C_TransferReturn_TypeDef ret =I2C_Transfer(I2C0);

	if(ret == i2cTransferDone)
	{
		if(humidStateActive)
		{
			bt_event |= TRANSFER_COMPLETE_HUMID;
		}
		else if (aqiStateActive)
		{
			bt_event |= TRANSFER_COMPLETE_AQI;
		}
	}

	else if(ret != i2cTransferInProgress)
	{
		LOG_DEBUG("I2C Error:%d",ret);

		if(humidStateActive)
		{
			bt_event |= TRANSFER_FAIL_HUMID;
		}
		else if (aqiStateActive)
		{
			bt_event |= TRANSFER_FAIL_AQI;
		}

	}

	gecko_external_signal(bt_event);
	CORE_ATOMIC_IRQ_ENABLE();
}



