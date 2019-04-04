/*
 * @filename	: cmu.c
 * @description	: This file contains functions to configure Clocks
 * @author 		: Puneet Bansal
 *
 */

#include "cmu.h"
#include "main.h"

/*
 * @description
 * For EM0,EM1 & EM2 selected LFX0 oscillator and for EM3 selected the ULFRCO
 * Selected LFA clock branch.
 * Enabled LETIMER0 and LFA clocks.
 */
void clock_init()
{
	if(sleepEM>=0 && sleepEM<3)
	{
		CMU_OscillatorEnable(cmuOsc_LFXO,true,true);			//Enable low frequency crystal oscillator
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); 		//Selecting the branch as LFA and clock source to be LFXO
	}
	else if(sleepEM==3)
	{
		CMU_OscillatorEnable(cmuOsc_ULFRCO,true,true);			//Enable low frequency crystal oscillator
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO); 		//Selecting the branch as LFA and clock source to be LFXO
	}

	CMU_ClockEnable(cmuClock_LFA, true);
	//CMU_ClockDivSet(cmuClock_LETIMER0, 4);

	CMU_ClockEnable(cmuClock_LETIMER0,true);
}
