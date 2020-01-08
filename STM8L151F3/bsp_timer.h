/**
  ******************************************************************************
  * @file    Keyfob/bsp_timer.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   RTC Wakeup timer low level functions header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H
#include <stdint.h>

#define BSP_TIMER_RESOLUTION_US 421.0526    /*!< timer step in microseconds */
/*!< ~20.2 ms (20000/421 - 1) = ~47  */
#define BSP_TIMER_TICKS(x)  ((uint16_t)(((x)*1000L)/BSP_TIMER_RESOLUTION_US -1))

extern void (*timerIsr)(void);

extern void BSP_Timer_Init( void );
extern void BSP_Timer_Arm( uint16_t aPeriod );
extern void BSP_Timer_Disarm( void );
extern void BSP_Timer_DeInit( void );
// Engage RTC wakeup timer 
extern uint8_t BSP_Timer_Activate(void (*isr)(void), uint16_t timeout);
// Free RTC wakeup timer
extern void BSP_Timer_Deactivate(void);

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/