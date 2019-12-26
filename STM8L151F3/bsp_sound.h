/**
  ******************************************************************************
  * @file    Keyfob/bsp_sound.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Sound generation timers low level functions header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#ifndef __BSP_SOUND_H
#define __BSP_SOUND_H
#include <stdint.h>

extern void BSP_Sound_Init( void );

extern void BSP_Sound_DurationTimer_Init( void );
extern void BSP_Sound_DurationTimer_Arm( uint8_t aDuration );
extern void BSP_Sound_DurationTimer_Disarm( void );
extern void BSP_Sound_DurationTimer_DeInit( void );

extern void BSP_Sound_ToneTimer_Init( void );
extern void BSP_Sound_ToneTimer_Arm( uint16_t aToneHalfPeriod );
extern void BSP_Sound_ToneTimer_Disarm( void );
extern void BSP_Sound_ToneTimer_DeInit( void );

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/