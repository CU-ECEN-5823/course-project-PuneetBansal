/*
 * @filename 	: i2c.h
 * @description	: contains headers to support i2c.c
 * @author		: Puneet Bansal
 */

#include <i2cspm.h>
#include "em_gpio.h"
#include "gpio.h"
#include "em_cmu.h"
#include <em_i2c.h>


void i2c_init();
void i2c_transfer();
void i2c_write(I2C_TransferSeq_TypeDef,uint16_t);
//uint16_t i2c_read_alg(I2C_TransferSeq_TypeDef);
uint16_t i2c_read(I2C_TransferSeq_TypeDef,uint8_t);

//#define SCL_PORT gpioPortC
//#define SCL_PIN 10

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

/*Defines for bits of status register*/
#define APP_VALID 1<<4
#define DATA_READY 1<<3
#define FW_MODE 1<<7

uint16_t data,len;

