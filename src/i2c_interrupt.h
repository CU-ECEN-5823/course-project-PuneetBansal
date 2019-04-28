/*
 * @filename 	: i2c_interrupt.h
 * @description	: contains headers to support i2c_interrupt.c
 * @author		: Puneet Bansal
 */

#include <i2cspm.h>
#include "em_gpio.h"
#include "gpio.h"
#include "em_cmu.h"
#include <em_i2c.h>

I2C_TransferSeq_TypeDef initNonPollingI2c;
uint8_t command_to_write;
uint16_t humid_value;

void i2c_interrupt_init();
void i2c_IntBasedRead(uint8_t, uint8_t);
void i2c_IntBasedWrite(uint8_t, uint8_t, uint8_t);
void i2c_IntBasedWriteWrite(uint8_t , uint8_t, uint8_t,uint8_t);
void i2c_IntBasedWriteRead(uint8_t, uint8_t , uint8_t, uint8_t);

#define SCL_PORT gpioPortC
#define SCL_PIN 10

//#define SDA_PORT gpioPortC
//#define SDA_PIN 11

#define SDA_PORT gpioPortC
#define SDA_PIN 11

#define SLAVE_ADD_HUMID 0x40
#define SLAVE_ADD_AQI 0X5A

#define NO_HOLD_MASTER_TEMP 0XE3
#define NO_HOLD_MASTER_HUMID 0XE5

#define HW_ID 0X20
#define ALG_RES 0X02
#define STATUS_REG 0X00
#define MEAS_MODE 0X01
#define APP_START 0xF4
#define ERROR_ID 0XE0

/*Defines for bits of status register*/
#define APP_VALID 1<<4
#define DATA_READY 1<<3
#define FW_MODE 1<<7

uint16_t data,len;
