/**
  ******************************************************************************
  * @file    Keyfob/bsp_buttons.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   GPIO,EXTI low level functions for buttons support header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#ifndef __BSP_BUTTONS_H
#define __BSP_BUTTONS_H
#include "stm8l15x.h"
#include "buttons.h"
#include "bsp_timer.h"

// Buttons pins definition
#define BUTTON_A_PORT             GPIOC
#define BUTTON_A_PIN              GPIO_Pin_5
#define BUTTON_A_EXTI             EXTI_Pin_5

#define BUTTON_NUM         1    /*!< Total number of buttons. 1..4 */
/*!< ~20.2 ms (20000/421 - 1) = ~47  */
#define BUTTON_TIMER_PERIOD_RAW  ((uint16_t)((BUTTON_TIMER_PERIOD_MS*1000)/BSP_TIMER_RESOLUTION_US -1))

extern void    BSP_Buttons_Init( void );
extern void    BSP_Buttons_PinInterruptDisable( void );
extern void    BSP_Buttons_EnablePullup( void );
extern uint8_t BSP_Buttons_GetState( void );

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/