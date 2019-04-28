#include <src/sensors.h>
#include <stdbool.h>
#include "native_gecko.h"
#include "log.h"
#include "main.h"
#include "letimer.h"
#include "gpio.h"
#include "i2c.h"
#include "i2c_interrupt.h"

#include "cmu.h"
#include "gpiointerrupt.h"

extern void gecko_main_init();
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

extern void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
//#define WAKE_PIN_PORT gpioPortF
//#define WAKE_PIN 7

int main(void)
{
  // Initialize stack
	gecko_main_init();
	clock_init();
	letimer_init();
	gpioInit();
	logInit();
	displayInit();
	gpioEnableDisplay();
	GPIOINT_Init();
	//GPIOINT_CallbackRegister(7, gpioCallback1);
	passed=0;

	humidStateActive = 0;
	aqiStateActive =0;

	/*Initialisations for using I2C*/
	i2c_init();

	presentState=powerOff_humid;// to add
	prevState=powerOff_humid;// to add
//	presentState = idle;
//	prevState = idle;
	event = None_humid;

	aqi_presentState=powerOff_aqi;
	aqi_prevState=powerOff_aqi;
	aqi_event= None_aqi;

#if NON_BLOCKING_IMPLEMENTATION
	GPIO_PinOutSet(SENSOR_ENABLE_PORT,SENSOR_ENABLE_PIN); //Enabling the si7021 sensor pin and waiting for 80ms
	timerSetEventinms(80);
#endif

	/************Setting configurations for CCS811 Sensor****/
	GPIO_PinOutClear(WAKE_PIN_PORT,WAKE_PIN); //initially making the CCS811 sensor on.
	sensorConfig();
	changeMode();

#if LPM_AQI
	GPIO_PinOutSet(WAKE_PIN_PORT,WAKE_PIN); //Making the CCS811 sensor off for LPM.
#endif

#if NON_BLOCKING_IMPLEMENTATION
	aqi_prevState = configured;
#endif

	/************Initializing Global Variables***************/
	humid_value=0;
	maxHumid=0;
	maxAqi=0;
	buttonPress=0;
	buttonDefine=0;
	data_ready_check=1;
	/********************************************************/

  /* Infinite loop */
	while (1) {
		//LOG_INFO("Pin value is %d",GPIO_PinInGet(WAKE_PIN_PORT,WAKE_PIN));
		struct gecko_cmd_packet *evt = gecko_wait_event();
		bool pass = mesh_bgapi_listener(evt);
		if (pass) {
			handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		}

#if NON_BLOCKING_IMPLEMENTATION

		humidity_stateMachine();
//		humidityStateMachineLPM();
//		if(event) continue;

//		aqi_stateMachine();
		aqi_stateMachine1();
		//stateCheck();

		//if(aqi_event)continue;
#endif
	};
}
