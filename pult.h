/**
  ******************************************************************************
  * @file    Keyfob/pult.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Aplication functionality header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  ******************************************************************************
  */ 

#ifndef __PULT_H
#define __PULT_H
#include <stdint.h>
#include "buttons.h"
#include "sound.h"

/* Exported defines and types ------------------------------------------------*/

/* Exported variables and constants ------------------------------------------*/
extern void (*TimerIsr)(void);

/* Exported functions ------------------------------------------------------- */

extern void Pult_Init(void);
extern void Pult_onClicks(uint32_t aClicksSet);
extern void Pult_onSoundPlayed(SoundTrackId aSoundTrackId);

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/