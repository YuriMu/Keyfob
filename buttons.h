/**
  ******************************************************************************
  * @file    Keyfob/buttons.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Keyfob buttons driver header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  ******************************************************************************
  */ 

#ifndef __BUTTONS_H
#define __BUTTONS_H
#include <stdint.h>

/* Includes ------------------------------------------------------------------*/

/* Exported defines and types ------------------------------------------------*/
#define BUTTON_TIMER_PERIOD_MS 20    /*!< timer trigger period in milliseonds */

#define BUTTON_NORMAL_CLICKS_MSK  0x07 /*!< Mask for normal clicks accumulator*/
#define BUTTON_LONG_CLICKS_MSK    0x38 /*!< Mask for long clicks accumulator  */

typedef union {
    uint32_t asLong;
    uint8_t  asByte[4];
} ClicksSet;

/* Exported variables and constants ------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

//extern void Buttons_Init( void );
//extern void Buttons_onTimerIsr(void);
extern void Button_onPressIsr( void );

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/