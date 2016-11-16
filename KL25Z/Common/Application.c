/*
 * Application.c
 *
 *  Created on: 28.02.2016
 *      Author: Erich Styger
 */

#include "Platform.h"
#include "Application.h"
#include "Event.h"
#include "LED.h"
#include "WAIT1.h"
#include "CS1.h"
#include "Keys.h"
#include "CLS1.h"
#include "RApp.h"
#if PL_CONFIG_HAS_BUZZER
  #include "Buzzer.h"
#endif
#if PL_CONFIG_HAS_RTOS
  #include "RTOS.h"
#endif
#if PL_CONFIG_HAS_SHELL
  #include "Shell.h"
#endif

#if PL_CONFIG_HAS_EVENTS
void APP_EventHandler(EVNT_Handle event) {
	uint8_t val;  // value for transmission
  switch(event) {
#if PL_CONFIG_HAS_KEYS
  #if PL_CONFIG_NOF_KEYS>=1
  case EVNT_SW1_PRESSED:
	#if PL_CONFIG_HAS_SHELL
//  CLS1_SendStr("SW1 pressed\r\n", CLS1_GetStdio()->stdOut);
    SHELL_SendString("SW1 pressed\r\n");
	 #endif

	#if (PL_CONFIG_CONTROL_SENDER && PL_CONFIG_HAS_REMOTE)
    val = 'A';
    #endif

    #if PL_CONFIG_HAS_BUZZER
    BUZ_PlayTune();
	#endif
    break;
  case EVNT_SW1_RELEASED:
    //LED2_Neg();
    CLS1_SendStr("SW1 released\r\n", CLS1_GetStdio()->stdOut);
    //SHELL_SendString("SW1 released\r\n");
    break;
  case EVNT_SW1_LPRESSED:
    //LED2_Neg();
    CLS1_SendStr("SW1 long pressed\r\n", CLS1_GetStdio()->stdOut);
    //SHELL_SendString("SW1 long pressed\r\n");
    break;
  #endif
  #if PL_CONFIG_NOF_KEYS>=2
  case EVNT_SW2_PRESSED:
    //LED2_Neg();
    //CLS1_SendStr("SW2 pressed\r\n", CLS1_GetStdio()->stdOut);
    SHELL_SendString("SW2 pressed\r\n");

	#if (PL_CONFIG_CONTROL_SENDER && PL_CONFIG_HAS_REMOTE)
    val = 'B';
    #endif

    break;
  #endif
  #if PL_CONFIG_NOF_KEYS>=3
  case EVNT_SW3_PRESSED:
    //LED2_Neg();
    //CLS1_SendStr("SW3 pressed\r\n", CLS1_GetStdio()->stdOut);
    SHELL_SendString("SW3 pressed\r\n");

	#if (PL_CONFIG_CONTROL_SENDER && PL_CONFIG_HAS_REMOTE)
    val = 'C';
    #endif

    break;
  #endif
  #if PL_CONFIG_NOF_KEYS>=4
  case EVNT_SW4_PRESSED:
    //LED2_Neg();
    //CLS1_SendStr("SW4 pressed\r\n", CLS1_GetStdio()->stdOut);
    SHELL_SendString("SW4 pressed\r\n");

    #if (PL_CONFIG_CONTROL_SENDER && PL_CONFIG_HAS_REMOTE)
    val = 'D';
    #endif

    break;
  #endif
  #if PL_CONFIG_NOF_KEYS>=5
  case EVNT_SW5_PRESSED:
    //LED2_Neg();
    //CLS1_SendStr("SW5 pressed\r\n", CLS1_GetStdio()->stdOut);
    SHELL_SendString("SW5 pressed\r\n");

    #if (PL_CONFIG_CONTROL_SENDER && PL_CONFIG_HAS_REMOTE)
    val = 'E';
    #endif

    break;
  #endif
  #if PL_CONFIG_NOF_KEYS>=6
  case EVNT_SW6_PRESSED:
    //LED2_Neg();
    //CLS1_SendStr("SW6 pressed\r\n", CLS1_GetStdio()->stdOut);
    SHELL_SendString("SW6 pressed\r\n");

	#if (PL_CONFIG_CONTROL_SENDER && PL_CONFIG_HAS_REMOTE)
    val ='F';
    #endif

    break;
  #endif
  #if PL_CONFIG_NOF_KEYS>=7
  case EVNT_SW7_PRESSED:
    //LED2_Neg();
    //CLS1_SendStr("SW7 pressed\r\n", CLS1_GetStdio()->stdOut);
    SHELL_SendString("SW7 pressed\r\n");

	#if (PL_CONFIG_CONTROL_SENDER && PL_CONFIG_HAS_REMOTE)
    val = 'K';
    #endif
    break;
  #endif
#endif
  } /* switch */
#if (PL_CONFIG_CONTROL_SENDER && PL_CONFIG_HAS_REMOTE)
  if(val != '0') {
	  RAPP_SendPayloadDataBlock((uint8_t *)&val, sizeof(val), RAPP_MSG_TYPE_JOYSTICK_BTN, RNETA_GetDestAddr(), RPHY_PACKET_FLAGS_NONE);
	  val = '0';
	#if PL_CONFIG_HAS_SHELL
	  SHELL_SendString("command sent!\r\n");
	#endif
  }
  #endif
}
#endif /* PL_CONFIG_HAS_EVENTS */

void APP_Start(void) {
#if 1/* already done in keys.c */
  /* SW1: enable and turn on pull-up resistor for PTA14 (push button) */
  PORT_PDD_SetPinPullSelect(PORTA_BASE_PTR, 14, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTA_BASE_PTR, 14, PORT_PDD_PULL_ENABLE);
#endif
#if 1
  /* pull-ups for Quadrature Encoder Pins */
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 10, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 10, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 11, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 11, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 16, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 16, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 17, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 17, PORT_PDD_PULL_ENABLE);
#endif
  PL_Init();
#if PL_CONFIG_HAS_EVENTS
  EVNT_SetEvent(EVNT_STARTUP);
#endif
#if CLS1_DEFAULT_SERIAL
  CLS1_SendStr("Hello World!\r\n", CLS1_GetStdio()->stdOut);
#endif
#if PL_CONFIG_HAS_RTOS
  vTaskStartScheduler(); /* start the RTOS, create the IDLE task and run my tasks (if any) */
  /* does usually not return! */
#else
  for(;;) {
#if PL_CONFIG_HAS_KEYS
    KEY_Scan();
#endif
#if PL_CONFIG_HAS_EVENTS
    EVNT_HandleEvent(APP_EventHandler, TRUE);
#endif
    WAIT1_Waitms(25); /* just wait for some arbitrary time .... */
  }
#endif
}

void APP_DebugPrint(unsigned char *msg)
{
	SHELL_SendString(msg);
}
