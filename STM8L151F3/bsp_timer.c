/**
  ******************************************************************************
  * @file    Keyfob/bsp_timer.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   RTC Wakeup timer low level functions
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
#include "bsp_timer.h"
#include "stm8l15x.h"


/** @addtogroup Keyfob
  * @{
  */

/* Private define and typedef ------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Public functions ---------------------------------------------------------*/
      
/**
  * @brief  Configure RTC wakeup timer peripherial 
  * @param  None
  * @retval None
  */
void BSP_Timer_Init( void )
{
    // Wake up after Intref stabilization.
    PWR_FastWakeUpCmd(DISABLE); 
    // Keep Iintref switched on in ActiveHalt mode 
    PWR_UltraLowPowerCmd(DISABLE); 

    // Enable RTC clock 
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_1);
    // Wait for LSI clock to be ready 
    while (CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == RESET);
  
    // Connect SYSCLK to RTC 
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);

    // Configures the RTC wakeup timer_step = RTCCLK/16 = LSI/16 = 421.0526 us 
    RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);

    // Clear wake up interrupt flag and enable wake up interrupt   
    RTC_ClearITPendingBit(RTC_IT_WUT);
    RTC_ITConfig(RTC_IT_WUT, ENABLE);
}

void BSP_Timer_DeInit( void )
{
    //  Disable and Clear wake up interrupt   
    RTC_ITConfig(RTC_IT_WUT, DISABLE);
    RTC_ClearITPendingBit(RTC_IT_WUT);

    // Cut off SYSCLK from RTC
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC, DISABLE);
    //CLK_PeripheralClockConfig(CLK_Peripheral_LCD, DISABLE);

    // Disable RTC clock(usually LSI) 
    CLK_RTCClockConfig(CLK_RTCCLKSource_Off, CLK_RTCCLKDiv_1);

    // Wake up before Intref stabilization. Analog features will be available in ~3ms
    PWR_FastWakeUpCmd(ENABLE); 
    // Switchs off Intref in Halt mode 
    PWR_UltraLowPowerCmd(ENABLE); 
}

// Set wakeup period (in timer steps) and launch counting
void BSP_Timer_Arm( uint16_t aPeriod )
{
    RTC_WakeUpCmd(DISABLE);
    RTC_SetWakeUpCounter(aPeriod);
    RTC_WakeUpCmd(ENABLE);
}


// Free wakeup timer(Wut) for use in other modules
void BSP_Timer_Disarm( void )
{
    RTC_WakeUpCmd(DISABLE);        
}


/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/