/**
  ******************************************************************************
  * @file    Keyfob/bsp_buttons.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   GPIO,EXTI low level functions for buttons support
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  *
  ******************************************************************************
  */ 
#include "bsp_buttons.h"


/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Buttons
  * @{
  */

/* Private define and typedef ------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Public functions ---------------------------------------------------------*/
      
void BSP_Buttons_Init( void )
{
    // Pin configured in input pullup mode with interrupt enabled
    // connected to EXTIx Interrupt, where x:0..7 
    GPIO_Init(BUTTON_A_PORT, BUTTON_A_PIN, GPIO_Mode_In_PU_IT);
    EXTI_SetPinSensitivity(BUTTON_A_EXTI, EXTI_Trigger_Falling);
}
                            

/*
static void Buttons_PinInterruptsEnable()
{
    BUTTON_A_PORT->CR2 |= BUTTON_A_PIN;
    //BUTTON_B_PORT->CR2 |= BUTTON_B_PIN;
    //  ..
}
*/

void BSP_Buttons_PinInterruptDisable( void )
{
    BUTTON_A_PORT->CR2 &= (uint8_t)(~(BUTTON_A_PIN));
    //BUTTON_B_PORT->CR2 &= (uint8_t)(~(BUTTON_B_PIN));
    //..
}
                            
void BSP_Buttons_EnablePullup( void )
{
    GPIO_ExternalPullUpConfig(BUTTON_A_PORT, BUTTON_A_PIN, ENABLE);
    //GPIO_ExternalPullUpConfig(BUTTON_B_PORT, BUTTON_B_PIN, ENABLE);
    // ..
}


/**
  * @brief  Reads all button pins.
  * @param  None.
  * @retval Buttons states in defined order. 1-button pushed.
  * @note : Pullup resistors must be enabled before. 
  *         Disable pullup resistor, if button is beeng pressed .
  *         To reduce power consumption.
  */
uint8_t BSP_Buttons_GetState( void )
{
    uint8_t orderedState;
    
    orderedState = 0;
    
    if ( GPIO_ReadInputDataBit(BUTTON_A_PORT, BUTTON_A_PIN) == RESET )
    {
        GPIO_ExternalPullUpConfig(BUTTON_A_PORT, BUTTON_A_PIN, DISABLE);
        orderedState |=0x01;
    }
    //if ( GPIO_ReadInputDataBit(BUTTON_B_PORT, BUTTON_B_PIN) == RESET )
    //{
    //    GPIO_ExternalPullUpConfig(BUTTON_B_PORT, BUTTON_B_PIN, DISABLE);
    //    orderedState |=0x02;
    //}
    // ..
    
    return orderedState;
}


/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/