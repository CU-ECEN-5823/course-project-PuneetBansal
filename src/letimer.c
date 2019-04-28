/*
 * @filename	: letimer.c
 * @description	: This file contains functions to configure LETIMER
 * @author 		: Puneet Bansal
 *
 */
#include "letimer.h"
#include "main.h"
#include "native_gecko.h"
int i=1;

uint32_t overflow_count=0;

/*
 * @description
 * Populating LETIMER_Init structure with default values and initialising LETIMER.
 * Set the compare value in COMP0 register according to the required delay using CompareSet().
 * Enable underflow interrupt, NVIC and LETIMER.
 */
void letimer_init()
{

	LETIMER_Init_TypeDef init = LETIMER_INIT_DEFAULT;

	init.enable=false;				//start counting when init completed
	init.comp0Top=true;				//load comp0 value in CNT on underflow
	init.debugRun=false;			//not running counter in debug mode
	init.repMode=letimerRepeatFree;	//count until stopped
	init.out0Pol=0;
	init.out1Pol=0;
	init.ufoa0=letimerUFOANone;
	init.ufoa1=letimerUFOANone;

	prescale_set();

	LETIMER_Init(LETIMER0,&init);

	LETIMER_CompareSet(LETIMER0,0,ticks);

	//LedOn_Ticks= freq*LED_ONTIME;
	//LETIMER_CompareSet(LETIMER0,1,LedOn_Ticks);

	LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF ); /*Enable Underflow interrupts*/

	NVIC_EnableIRQ(LETIMER0_IRQn);

	LETIMER_Enable(LETIMER0, true);
}

/*
 * @description
 * Set the prescaler value depending on the selected energy mode and the desired
 * LED period.
 */

void prescale_set()
{
	int j=0;
	if(sleepEM>=0 && sleepEM<3)
	{
		freq= LFX0_FREQ;		/*Select freq=32768 for EM(0-2)*/
	}
	else if (sleepEM==3)
	{
		freq= ULFRCO_FREQ;		/*Select freq=1000Hz for EM(3)*/
	}
	ticks=LED_PERIOD*freq;

	if(ticks> MAX_TICKS)
	{
		while(ticks> MAX_TICKS)
		{
			ticks=ticks/2;
			freq=freq/2;
			j++;
		}
		CMU_ClockDivSet(cmuClock_LETIMER0,pow(2,j));
	}
}
/*
 * @description
 *  Function to generate delay in micro seconds.
 *
 * @param us_wait
 * delay in microsecond required
 *
 */
void timerWaitUs(uint32_t us_wait)
{
	uint32_t current_ticks,max_tick,count_upto;
	uint32_t us_ticks;

	us_ticks=freq * us_wait;
	us_ticks=us_ticks/1000000;	/*Calculate the ticks required in us*/

	current_ticks=LETIMER_CounterGet(LETIMER0); /*Get the present value of timer count*/

	if(current_ticks>us_ticks)
	{
		count_upto=current_ticks-us_ticks;
		//gpioLed0SetOn();
		while(LETIMER_CounterGet(LETIMER0)!=count_upto);
		//gpioLed0SetOff();
	}
	else
	{
		max_tick=LETIMER_CompareGet(LETIMER0,0);

		//gpioLed0SetOn();
		while(LETIMER_CounterGet(LETIMER0)!=(max_tick-(us_ticks-current_ticks)));
		//gpioLed0SetOff();

	}
}

/*
 * @description
 * triggers a comp1 interrupt on achieving the desired amount of ms delay.
 *
 */
void timerSetEventinms(uint32_t ms_until_wakeup)
{
	uint32_t current_ticks=0;
	uint32_t max_tick,count_upto;
	uint32_t ms_ticks;

	ms_ticks=freq * ms_until_wakeup;
	ms_ticks=ms_ticks/1000;	/*Calculate the ticks required in s*/

//	LETIMER_Enable(LETIMER0, false);
	current_ticks=LETIMER_CounterGet(LETIMER0); /*Get the present value of timer count*/
//	LOG_INFO("current ticks=%d\n",current_ticks);


	if(current_ticks>=ms_ticks)
	{
		count_upto=current_ticks-ms_ticks;
	}
	else
	{
		max_tick=LETIMER_CompareGet(LETIMER0,0);
		count_upto=(max_tick-(ms_ticks-current_ticks));
	}

	LETIMER_CompareSet(LETIMER0,1,count_upto);
	LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);
}



/*
 * @description
 * Interrupt handler for LETIMER0. Hitting the underflow interrupt means that 3seconds delay has passed
 * and a comp0underflow event is set.
 * Hitting the Comp1 interrupt means that the desired ms delay has been achieved and comp1 interrupt event
 * has been set
 *
 */

void LETIMER0_IRQHandler(void)
{

	CORE_ATOMIC_IRQ_DISABLE();
	uint32_t flags =LETIMER_IntGetEnabled(LETIMER0);
	LETIMER_IntClear(LETIMER0,flags);

	if(flags & LETIMER_IF_UF)
	{

		//LOG_INFO("entered comp0 interrupt\n");
		overflow_count++;
		//bt_event |= bt_Comp0Underflow;
		displayUpdate();
		//GPIO_PinOutToggle(LCD_PORT,LCD_PIN);

	}
	if(flags & LETIMER_IF_COMP1)
	{
		//LOG_INFO("entered comp1 interrupt\n");
		//bt_event |= bt_Comp1Interrupt;
		LETIMER_CompareSet(LETIMER0,1,0xFFFF);
		LETIMER_IntDisable(LETIMER0,LETIMER_IFC_COMP1);
		//bt_event= COMP1_INTERRUPT; //remove
		//event=Comp1Interrupt;  //to add
		prevState=powerOn_humid;//to add
		presentState=powerOn_humid;//to add

	}
	//gecko_external_signal(bt_event);//remove
	CORE_ATOMIC_IRQ_ENABLE();

}

