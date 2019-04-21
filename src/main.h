/* @FileName	: main.h
 * @description	: contains header files and #defines for the project.
 * @author 		: Puneet Bansal
 * @reference	: Silicon Labs SDK -https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 * 				: Bluetooth Mesh SDK- https://www.silabs.com/documents/login/reference-manuals/bluetooth-le-and-mesh-software-api-reference-manual.pdf
 *				: SOC Bluetooth Mesh Light, SOC Bluetooth Mesh Switch example code.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "log.h"
#include "display.h"
#include "ble_device_type.h"
#include "gecko_ble_errors.h"

/************************ DEFINES TO SET CLOCK RATE ****************************************/
#define LED_PERIOD (1)
#define sleepEM (1)
#define LED_ONTIME (.175)	/*LED on time in seconds*/
#define LFX0_FREQ (32768)	/*Low frequency crystal oscillator frequency*/
#define ULFRCO_FREQ (1000)	/*Ultra low frequency crystal oscillator frequency*/
#define MAX_TICKS (65535)	/*Maximum ticks possible for a 16bit timer*/

/*******************************************************************************************/

#define bt_Comp0Underflow 1<<0
#define bt_risingEdgeInt 1<<4
#define bt_fallingEdgeInt 1<<5
#define bt_Comp1Interrupt 1<<1

/************************ GPIO PINS AND PORTS DEFINE****************************************/
#define SENSOR_ENABLE_PIN 15
#define SENSOR_ENABLE_PORT gpioPortD

#define LCD_PORT gpioPortD
#define LCD_PIN 13

#define LCD_ENABLE_PIN 15
#define LCD_ENABLE_PORT gpioPortD

#define WAKE_PIN_PORT gpioPortF
#define WAKE_PIN 7

/************************** HARDWARE SOFTWARE TIMER DEFINES ******************************/
#define timerHandle 12
#define LcdHandler 8
#define sensorReading 6
#define frienshipFailedHandle 10


/*********************PERSISTENT MEMORY DEFINES & GLOBAL VARIABLES************************/

#define HUMID_KEY 0x4005
#define AQI_KEY 0X4010
uint16_t maxHumid;
uint16_t maxAqi;
/****************************************************************************************/


#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1
#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1
uint32_t buttonPress;
uint32_t buttonDefine;
uint32_t bt_event;
