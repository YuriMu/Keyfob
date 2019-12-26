/**
  ******************************************************************************
  * @file    Keyfob/main.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Main program body
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
#include "stm8l15x.h"
#include "bsp_system.h"
//#include "bsp_timer.h"
#include "bsp_buttons.h"
#include "pult.h"
#include "bsp_sound.h"


/** @addtogroup Keyfob
  * @{
  */ 

/* Private define and typedef ------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void main(void)
{
    //static uint8_t tmp;    
    //static uint8_t tmp1;    
    //static uint8_t tmp2;
    //static uint8_t tmp3;
    
    
    //Sound_GenerateToneValues();    
    
    BSP_System_Init();
    BSP_Buttons_Init();
    BSP_Sound_Init();
    Pult_Init();
    //CLK_PeripheralClockConfig(CLK_Peripheral_COMP, ENABLE);
    
    //tmp1 = RI->IOIR1;
    //tmp2 = RI->IOIR2;
    //tmp3 = RI->IOIR3;

    //// Set all pins as inputs with pullup and disabled interrupts 
    //GPIO_Init(GPIOA, GPIO_Pin_2|GPIO_Pin_3, GPIO_Mode_In_PU_No_IT);
    //GPIO_Init(GPIOB, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    //GPIO_Init(GPIOC, GPIO_Pin_All, GPIO_Mode_In_PU_No_IT);
    //GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT);

    //GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_Out_PP_Low_Fast);
    //GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Fast);
    //GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_Out_PP_Low_Slow);
    //GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Slow);

    //tmp1 = RI->IOIR1;
    //tmp2 = RI->IOIR2;
    //tmp3 = RI->IOIR3;

    
        //GPIOC->IDR = 0xff;
    //GPIOC->ODR = 0xff;
    //tmp = GPIOC->IDR;
    //tmp = tmp;
    //tmp = GPIOC->CR2;

    enableInterrupts();

    while (1) {
        if ( BSP_System_HasWorkingPeripheral() )
            BSP_System_Sleep();
        else
            BSP_System_Halt();
        
        //GPIOB->ODR = 0;
        //for(uint16_t i=0; i<300; i++);
        //GPIOB->ODR = 0xff;
        //for(uint16_t i=0; i<300; i++);
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/