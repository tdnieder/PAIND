/*
 * Platform_Local.h
 *
 */

#ifndef SOURCES_PLATFORM_LOCAL_H_
#define SOURCES_PLATFORM_LOCAL_H_

/* board identification: */
#define PL_LOCAL_CONFIG_BOARD_IS_FRDM     (1) /* I'm the FRDM board */
#define PL_LOCAL_CONFIG_NOF_LEDS 		  (2) /* number of LEDs */
#define PL_LOCAL_CONFIG_NOF_KEYS          (7) /* number of keys */
#define PL_LOCAL_CONFIG_KEY_1_ISR         (1) /* if SW1 is using interrupts */
#define PL_LOCAL_CONFIG_KEY_2_ISR         (1) /* if SW1 is using interrupts */
#define PL_LOCAL_CONFIG_KEY_3_ISR         (1) /* if SW1 is using interrupts */
#define PL_LOCAL_CONFIG_KEY_4_ISR         (1) /* if SW1 is using interrupts */
#define PL_LOCAL_CONFIG_KEY_5_ISR         (0) /* if SW1 is using interrupts */
#define PL_LOCAL_CONFIG_KEY_6_ISR         (0) /* if SW1 is using interrupts */
#define PL_LOCAL_CONFIG_KEY_7_ISR         (1) /* if SW1 is using interrupts */

//#define PL_LOCAL_CONFIG_HAS_LEDS_DISABLED  (1)

#endif /* SOURCES_PLATFORM_LOCAL_H_ */
