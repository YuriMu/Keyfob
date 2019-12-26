/**
  ******************************************************************************
  * @file    Keyfob/sound.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Sound driver header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  ******************************************************************************
  */ 

#ifndef __SOUND_H
#define __SOUND_H
#include <stdint.h>

/* Exported defines and types ------------------------------------------------*/
typedef enum{
    SOUNDS_SHORT_BEEP,
    SOUNDS_NORMAL_BEEP,
    SOUNDS_LONG_BEEP,
    SOUNDS_ARMED,
    SOUNDS_DISARMED,
} SoundTrackId;

/* Exported variables and constants ------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

extern void Sound_Init( void );
extern void Sound_Play( SoundTrackId aSoundTrackId, uint8_t aRepeatsNum );
extern void Sound_Stop( void );
extern void Sound_GenerateToneValues( void );

extern void Sound_onDurationTimerIsr( void );

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/