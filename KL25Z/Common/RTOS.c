/**
 * \file
 * \brief Real Time Operating System (RTOS) main program.
 * \author Erich Styger, erich.styger@hslu.ch
 */

#include "Platform.h"
#if PL_CONFIG_HAS_RTOS
#include "RTOS.h"
#include "FRTOS1.h"
#include "LED.h"
#include "Event.h"
#include "Keys.h"
#include "Application.h"
/**
 * App-Task contains LED Heartbeat
 */
static void AppTask(void* param) {
  (void)param; /* avoid compiler warning */
  EVNT_SetEvent(EVNT_STARTUP); /* set startup event */
  for(;;) {
	  EVNT_HandleEvent(APP_EventHandler, TRUE);
	  KEY_Scan();
	  LED1_Neg();
      FRTOS1_vTaskDelay(500/portTICK_PERIOD_MS);

  }
}


void RTOS_Run(void) {
  FRTOS1_vTaskStartScheduler();  /* does usually not return! */
}

void RTOS_Init(void) {
  /*! \todo Create tasks here */
	if(FRTOS1_xTaskCreate(AppTask, (signed portCHAR *)"Blinky", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL) != pdPASS){
		for(;;){}
	}
	/* not necessary, already implemented in RADIO_Init()!!!!!
	if(FRTOS1_xTaskCreate(RadioTask, (signed portCHAR *)"Radio", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL) != pdPASS){
		for(;;){}
	}*/

//	// Task 2, Test motor
//	if(FRTOS1_xTaskCreate(MotorTask, (signed portCHAR *)"Test Moto", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL) != pdPASS){
//		for(;;){}
//	  }
}

void RTOS_Deinit(void) {
  /* nothing needed for now */
}

#endif /* PL_CONFIG_HAS_RTOS */
