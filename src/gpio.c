/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "native_gecko.h"
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>
#include "main.h"


#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5
#define	LED0_port gpioPortF
#define LED0_pin 4
#define LED1_port gpioPortF
#define LED1_pin 5

#define buttonPort gpioPortF
#define buttonPin 6

//#define WAKE_PIN_PORT gpioPortF
//#define WAKE_PIN 7


void gpioInit()
{
	/*GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);*/

	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
		//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
		GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
		GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
		//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateStrong);
		//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
		GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
		GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

		GPIO_PinModeSet(buttonPort, buttonPin, gpioModeInputPull, 1);
		GPIO_ExtIntConfig(buttonPort,buttonPin,buttonPin,true,true,true);

		GPIO_PinModeSet(WAKE_PIN_PORT, WAKE_PIN, gpioModePushPull, false);
//		GPIO_PinModeSet(WAKE_PIN_PORT, WAKE_PIN, gpioModeInputPull, 1);
//		GPIO_ExtIntConfig(WAKE_PIN_PORT,WAKE_PIN,WAKE_PIN,false,true,true);

//		NVIC_EnableIRQ(GPIO_EVEN_IRQn);


}

void gpioLed0Toggle()
{
	GPIO_PinOutToggle(LED0_port,LED0_pin);
}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}
void gpioEnableDisplay()
{
	GPIO_PinOutSet(LCD_ENABLE_PORT,LCD_ENABLE_PIN);
}
void gpioSetDisplayExtcomin(bool high)
{
	GPIO_PinOutToggle(LCD_PORT,LCD_PIN);
}

void gpioCallback1(uint8_t pin)
{
	LOG_INFO("Entered handler");
	//aqi_event=dataReady;
	bt_event= DATA_READY;
	//gecko_external_signal(bt_event);
	//data_ready_check=1;
}

