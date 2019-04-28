/*
 * @filename	: i2c.c
 * @description	: This file contains functions to configure I2C peripheral.
 * @author 		: Puneet Bansal
 * @reference	: Silicon Labs SDK -https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 *
 */
#include "i2c.h"
#include "main.h"

/*
 * @decription
 * initialise i2c module with default settings
 * Set the approriate SCL and SDA port ,pin and alternate function number
 *
 */
void i2c_init()
{
I2CSPM_Init_TypeDef init1 = I2CSPM_INIT_DEFAULT;

/*init1.sclPin=SCL_PIN;
init1.sclPort=SCL_PORT;
init1.sdaPin=SDA_PIN;
init1.sdaPort=SDA_PORT;
init1.portLocationScl=12;
init1.portLocationSda=14;*/


I2CSPM_Init(&init1);
}

/*
 * @description
 * function to perform write to slave.
 * writing the slave address, mode to hold master mode, length to 1 byte.
 * if the transfer is not complete then logged the error.
 *
 * @param: init
 * instance of TransferSeq_TypeDef passed from temp_get() in si7021.c
 * @param : len
 * length of data to write
 */

void i2c_write(I2C_TransferSeq_TypeDef init,uint16_t len)
{
	I2C_TransferReturn_TypeDef ret;

	init.buf[0].len=len;
	init.flags= I2C_FLAG_WRITE;


	ret=I2CSPM_Transfer(I2C0,&init);
	if(ret != i2cTransferDone)
	{
		LOG_ERROR("I2C Write error");
		return;
	}
}

/*
 * @description:
 * function to read temperature from slave. Set received_data to point to buf[0].data buffer.
 * calculated the 12bit temperature value from the received_data.
 * received_data[0] consists MSB, received_data[1] consists LSB
 *
 * @param: init
 * instance of TransferSeq_TypeDef passed from temp_get() in si7021.c
 *
 */
uint16_t i2c_read_alg(I2C_TransferSeq_TypeDef init)
{
	I2C_TransferReturn_TypeDef ret;
	uint8_t received_data[2] = {0};			/*Buffer to receive the data recieved from slave*/
	uint16_t tempvalue;						/*Storing the ADC value given by sensor*/
	init.flags= I2C_FLAG_READ;

	init.buf[0].data= received_data;
	init.buf[0].len=sizeof(received_data);//received_data=*(init.buf[0].data);
	ret=I2CSPM_Transfer(I2C0,&init);

	if(ret != i2cTransferDone)
			{
				LOG_ERROR("I2C Read error");
			}

	tempvalue = received_data[0];
	tempvalue <<= 8;
	tempvalue	|= (received_data[1]);
	return tempvalue;
}

uint16_t i2c_read(I2C_TransferSeq_TypeDef init, uint8_t len)
{
	I2C_TransferReturn_TypeDef ret;
	uint8_t received_data[2] = {0};			/*Buffer to receive the data recieved from slave*/
	uint16_t tempvalue;						/*Storing the ADC value given by sensor*/
	init.flags= I2C_FLAG_READ;

	init.buf[0].data= received_data;
	init.buf[0].len=sizeof(received_data);//received_data=*(init.buf[0].data);
	ret=I2CSPM_Transfer(I2C0,&init);

	if(ret != i2cTransferDone)
	{
		LOG_ERROR("I2C Read error");
	}
	if(len==1)
	{
		tempvalue = received_data[0];
	}
	else
	{
		tempvalue = received_data[0];
		tempvalue <<= 8;
		tempvalue	|=(received_data[1]);
	}

	return tempvalue;

}

