#include <src/sensors.h>
#include <stdbool.h>
#include "native_gecko.h"
#include "log.h"
#include "main.h"
#include "letimer.h"
#include "gpio.h"
#include "i2c.h"
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
	GPIOINT_CallbackRegister(6, gpioCallback1);

	/*Initialisations for using I2C*/
	i2c_init();

	/************Setting configurations for CCS811 Sensor****/
	GPIO_PinOutClear(WAKE_PIN_PORT,WAKE_PIN);
	sensorConfig();
	changeMode();

	/************Initialising Global Variables***************/
	maxHumid=0;
	maxAqi=0;
	buttonPress=0;
	buttonDefine=0;
	/********************************************************/

  /* Infinite loop */
	while (1) {
		struct gecko_cmd_packet *evt = gecko_wait_event();
		bool pass = mesh_bgapi_listener(evt);
		if (pass) {
			handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		}
	};
}
