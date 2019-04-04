/* @FileName	: main.h
 * @description	: contains header files and #defines for the project.
 * @author 		: Puneet Bansal
 * @reference	: Silicon Labs SDK -https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */

#include <stdint.h>
#include "cmu.h"
#include "letimer.h"
//#include "i2c.h"
#include <stdlib.h>
#include <stdbool.h>
#include "log.h"
//#include "si7021.h"
//#include "sleep.h"
//#include "i2c_interrupt.h"
#include "display.h"
#include "ble_device_type.h"

//#define sleepEM (sleepEM3) 		/*Energy mode you want to sleep in. i.e. if EM = 2 then, EM3,EM4 will be blocked.*/
#define sleepEM (1)
#define LED_PERIOD (1)		/*Total LED period in seconds*/
#define LED_ONTIME (.175)	/*LED on time in seconds*/
#define LFX0_FREQ (32768)	/*Low frequency crystal oscillator frequency*/
#define ULFRCO_FREQ (1000)	/*Ultra low frequency crystal oscillator frequency*/
#define MAX_TICKS (65535)	/*Maximum ticks possible for a 16bit timer*/
#define MEAS_TEMP (1<<0)
#define SET_STATE (1<<1);

#define SENSOR_ENABLE_PIN 15
#define SENSOR_ENABLE_PORT gpioPortD

#define connection_min_interval	(60) 	//time=value*1.25 so this is for 75ms
#define connection_max_interval (60)
#define connection_latency	(3) 		//(300/75)
#define connection_timeout (600) 		//value should be greater than (1+latency)*max_interval*2

#define bt_Comp0Underflow 1<<0
#define bt_Comp1Interrupt 1<<1
#define bt_transferComplete 1<<2
#define bt_transferError 1<<3
#define bt_risingEdgeInt 1<<4
#define bt_fallingEdgeInt 1<<5



#define LCD_PORT gpioPortD
#define LCD_PIN 13

#define LCD_ENABLE_PIN 15
#define LCD_ENABLE_PORT gpioPortD

#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1
#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

volatile uint16_t scheduler; 			/*Global variable to keep a track of events*/

typedef enum
{
	idle,
	initState,
	writeState,
	readState,
}states;

typedef enum
{
	None=0,
	Comp0Underflow,
	Comp1Interrupt,
	transferComplete,
	transferError
}events;
typedef enum
{
	discoverPrimaryServices,
	discoverCharacteristics,
	readCharacteristicsValue,
	indication,
	tempIndication,
	running
}client;

client clientState;

volatile states presentState,prevState;
volatile events event;
int flag;
void state_machine();


void send_temp_via_bluetooth(float);
void disable_fxn();

int connection_open;

uint8_t bt_connection;
uint32_t bt_event;
int32_t transmitPower;
int32_t rssiValue;

/*************Client related defines***************/
volatile uint8_t client_connection;
volatile uint32_t client_tempService;
volatile uint32_t client_buttonService;
volatile uint16_t client_buttonCharacteristics;
volatile uint16_t client_tempCharacteristics;
volatile uint32_t client_temp;
uint8_t chNotifCounterTemp;
uint8_t chNotifCounterButton;
uint8_t indicationCounter;

uint8_t* charTemp;
uint8_t* receivedButtonState;
#define UINT32_TO_FLT(data) (((float)((int32_t)(data) & 0x00FFFFFFU)) * (float)pow(10, ((int32_t)(data) >> 24)))
uint8_t client_state;
#define GETTEMP 0x01
/*************************************************/

///*Bonding related declarations*/
//uint8 bondingConnection;
//int8 bondingHandle;
//uint16_t boundingVar;
//
///*Bonding +Client*/
//uint8_t bondingCon;

uint32_t buttonPress;
#define timerHandle 12
uint32_t buttonDefine;


