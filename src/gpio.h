/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>

void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void gpioEnableDisplay();
void gpioSetDisplayExtcomin(bool high);
void gpioCallback1(uint8_t);

#define buttonPort gpioPortF
#define buttonPin 6
#define GPIO_SET_DISPLAY_EXT_COMIN_IMPLEMENTED 	1
#define GPIO_DISPLAY_SUPPORT_IMPLEMENTED		1

#endif /* SRC_GPIO_H_ */
