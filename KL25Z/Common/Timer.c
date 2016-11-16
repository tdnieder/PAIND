/**
 * \file
 * \brief Timer driver
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This module implements the driver for all our timers.
  */

#include "Platform.h"
#if PL_CONFIG_HAS_TIMER
#include "Timer.h"
#include "Event.h"
#include "Trigger.h"
#include "Tacho.h"


void TMR_OnInterrupt(void) {
	TRG_AddTick();
#if PL_CONFIG_HAS_MOTOR_TACHO //make it work with FRDM
	TACHO_Sample();
#endif
	/* this one gets called from an interrupt!!!! */


}



void TMR_Init(void) {
}

void TMR_Deinit(void) {
}

#endif /* PL_CONFIG_HAS_TIMER*/
