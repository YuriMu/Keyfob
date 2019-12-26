/**
  ******************************************************************************
  * @file    Keyfob/bsp_system.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   System low level functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  *
  ******************************************************************************
  */ 
#include "bsp_system.h"
#include "stm8l15x.h"


/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Bsp
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
void BSP_System_Init(void)
{
    // System clock prescaler: 1
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
    
    // Cut off SYSCLK from BOOTROM
    CLK_PeripheralClockConfig(CLK_Peripheral_BOOTROM, DISABLE);
    
    // Set all pins as inputs with pullup and disabled interrupts 
    GPIO_Init(GPIOA, GPIO_Pin_2|GPIO_Pin_3, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOB, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOC, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT);
}

inline void BSP_System_Sleep(void)
{
    wfi();
}

inline void BSP_System_Halt(void)
{
    halt();
}

/**
  * @brief  Is there working peripheral(besides RTC)
  * @param  None
  * @retval 0 - No active peripheral; #0 - there are active peripheral
*/
uint8_t BSP_System_HasWorkingPeripheral(void)
{
    uint8_t pckenr1;
    uint8_t pckenr2;
    uint8_t pckenr3;
    
    // Get peripheral's 'clocked' status
    pckenr1 = CLK->PCKENR1;
    pckenr2 = CLK->PCKENR2;
    pckenr3 = CLK->PCKENR3;
    
    // Exludes RTC 
    pckenr2 &= (uint8_t)(~(uint8_t)(((uint8_t)1 << ((uint8_t)CLK_Peripheral_RTC & (uint8_t)0x0F))));
    
    return pckenr1 | pckenr2 | pckenr3;
}

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/