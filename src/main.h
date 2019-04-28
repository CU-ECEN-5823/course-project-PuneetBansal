/* @FileName	: main.h
 * @description	: contains header files and #defines for the project.
 * 				  In order to implement non-blocking version of I2C the #define NON_BLOCKING_IMPLEMENTATION should be set to
 * 				  1 otherwise it should be set to 0.
 * 				  Similarly to implement, LPM the LPM_AQI define should be set to 1.
 *
 * @author 		: Puneet Bansal
 * @reference	: Silicon Labs SDK -https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 * 				: Bluetooth Mesh SDK- https://www.silabs.com/documents/login/reference-manuals/bluetooth-le-and-mesh-software-api-reference-manual.pdf
 *				: SOC Bluetooth Mesh Light, SOC Bluetooth Mesh Switch example code.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "log.h"
#include "display.h"
#include "ble_device_type.h"
#include "gecko_ble_errors.h"

#define NON_BLOCKING_IMPLEMENTATION 1
#define LPM_AQI 1
#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1
#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

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

#define HUMID_KEY 0x4001
#define AQI_KEY 0X4016
uint16_t maxHumid;
uint16_t maxAqi;
volatile uint8_t data_ready_check;
uint8_t passed;

/***************************** STATE MACHINE RELATED DEFINES******************************/
typedef enum
{
	powerOff_humid,
	powerOn_humid,
	initState,
	writeState,
	readState,
}states;

typedef enum
{
	None_humid,
	Comp1Interrupt,
	takeReading,
	transferComplete,
	transferError
}events;

typedef enum
{
	powerOff_aqi,
	writingData,
	writingIntermediate,
	initialised,

	appStarted,
	modeConfiguring,
	configured,
	getReading,
	getReading1,
	final,
}AqiStates;

typedef enum
{
	None_aqi,
	dataReady,
	aqi_transferComplete,
	aqi_transferError,
}AqiEvents;

volatile states presentState,prevState;
volatile events event;

volatile AqiStates aqi_presentState,aqi_prevState;
volatile AqiEvents aqi_event;

/*****************************************************************************************/
uint32_t buttonPress;
uint32_t buttonDefine;
uint32_t bt_event;

uint8_t humidStateActive;
uint8_t aqiStateActive;


#define TRANSFER_COMPLETE_HUMID 1
#define TRANSFER_COMPLETE_AQI 1<<2
#define TRANSFER_FAIL_HUMID 1<<3
#define TRANSFER_FAIL_AQI 1<<4
#define DATA_READY 1<<5
#define COMP1_INTERRUPT 1<<6
