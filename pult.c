/**
  ******************************************************************************
  * @file    Keyfob/pult.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Application functionality
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "pult.h"
//#include "stm8l15x.h"


/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Pult
  * @{
  */

/* Global variables ---------------------------------------------------------*/

void (*TimerIsr)(void) = 0;

/* Private define and typedef ------------------------------------------------*/
typedef struct {
    ClicksSet clicksSet;
} Pult;

/* Private variables ---------------------------------------------------------*/
Pult pult;

/* Private functions ---------------------------------------------------------*/
void Pult_Init(void)
{
}


#include "stm8l15x.h"

void Pult_onClicks(uint32_t aClicksSet)
{
    uint8_t repeats;
    /*
    for(uint16_t j=0; j<1000; j++){
         GPIOB->ODR ^= 1;
        for(uint16_t i=0; i<300; i++);
        //GPIOB->ODR = 0xff;
        //for(uint16_t i=0; i<300; i++);
    }
*/

    pult.clicksSet.asLong = aClicksSet;
    
    repeats = pult.clicksSet.asByte[0] & BUTTON_NORMAL_CLICKS_MSK;
    if( repeats ) {
        pult.clicksSet.asByte[0] &= ~BUTTON_NORMAL_CLICKS_MSK;
        Sound_Play(SOUNDS_ARMED, repeats); //SOUNDS_SHORT_BEEP
        return;
    }
    
    repeats = (pult.clicksSet.asByte[0] & BUTTON_LONG_CLICKS_MSK) / (BUTTON_NORMAL_CLICKS_MSK + 1);
    if( repeats ) {
        pult.clicksSet.asByte[0]  = 0;
        Sound_Play(SOUNDS_NORMAL_BEEP, 0);//repeats);
        return;
    }

}
                            
void Pult_onSoundPlayed(SoundTrackId aSoundTrackId)
{
    aSoundTrackId = aSoundTrackId;
    
    Pult_onClicks(pult.clicksSet.asLong);
}


/**
  * @}
  */
/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/