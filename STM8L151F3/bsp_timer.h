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

extern void BSP_Timer_Init( void );
extern void BSP_Timer_Arm( uint16_t aPeriod );
extern void BSP_Timer_Disarm( void );
extern void BSP_Timer_DeInit( void );

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/