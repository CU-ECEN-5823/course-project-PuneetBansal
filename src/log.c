/*
 * log.c
 *
 *  Created on: Dec 18, 2018
 *      Author: Dan Walkes
 */

#include "retargetserial.h"
#include "log.h"
#include <stdbool.h>
#include "main.h"

#if INCLUDE_LOGGING
/**
 * @return a timestamp value for the logger, typically based on a free running timer.
 * This will be printed at the beginning of each log message.
 */
uint32_t loggerGetTimestamp(void)
{
	//return timerGetRunTimeMilliseconds();
	uint32_t current_ticks,max_tick;
	uint32_t ms_time=0;
	max_tick=LETIMER_CompareGet(LETIMER0,0);
	current_ticks=LETIMER_CounterGet(LETIMER0);
	ms_time=((max_tick-current_ticks)*1000)/freq;
	ms_time+=(overflow_count* LED_PERIOD*1000);
	return ms_time;

}

/**
 * Initialize logging for Blue Gecko.
 * See https://www.silabs.com/community/wireless/bluetooth/forum.topic.html/how_to_do_uart_loggi-ByI
 */
void logInit(void)
{
	RETARGET_SerialInit();
	/**
	 * See https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__RetargetIo.html#ga9e36c68713259dd181ef349430ba0096
	 * RETARGET_SerialCrLf() ensures each linefeed also includes carriage return.  Without it, the first character is shifted in TeraTerm
	 */
	RETARGET_SerialCrLf(true);
	LOG_INFO("Initialized Logging");
}

/**
 * Block for chars to be flushed out of the serial port.  Important to do this before entering SLEEP() or you may see garbage chars output.
 */
void logFlush(void)
{
	RETARGET_SerialFlush();
}
#endif
