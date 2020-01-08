/**
  ******************************************************************************
  * @file    Keyfob/bsp_rf.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Rf transceiver hw interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  *
  ******************************************************************************
  */ 
#include "bsp_rf.h"
#include "bsp_spi.h"

/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Rf
  * @{
  */

void BSP_Rf_Init(void)
{
    // Don't enable interrupts so far. Will be enabled on enter in rx mode. 
    // Rising edge - sync word received. Falling edge - receive complete.
    GPIO_Init(RF_GDO0_PORT, RF_GDO0_PIN, GPIO_Mode_In_PU_No_IT);
    EXTI_SetPinSensitivity(RF_GDO0_EXTI, EXTI_Trigger_Falling);
    
    // Test pin
    GPIO_Init(RF_GDO2_PORT, RF_GDO2_PIN, GPIO_Mode_In_PU_No_IT);
    
    // SPI chip select pin is output high
    GPIO_Init(RF_CS_PORT, RF_CS_PIN, GPIO_Mode_Out_PP_High_Fast); // CSN tbd Slow
}

void CC_CSN_LOW(void)
{
    GPIO_ResetBits(RF_CS_PORT, RF_CS_PIN);
    
    // If RF_SPI was busy by another device, reinitialize RF_SPI
    if (spiCurrentDevice != SPI_DEVICE_CC1101) {
        spiCurrentDevice = SPI_DEVICE_CC1101;
        BSP_SPI_Init(RF_SPI);
    }
    while (GPIO_ReadInputDataBit(RF_SPI_MISO_PORT, RF_SPI_MISO_PIN)!=0);
}

void BSP_Rf_SPIDeInit(void)
{
    BSP_SPI_DeInit(RF_SPI);
}

uint8_t SPI_ExchangeByte(uint8_t input)
{
    return BSP_SPI_ExchangeByte(RF_SPI, input);
}
      
void BSP_Rf_TimerActivate(uint8_t timeout)
{
    //------ TIM4 init ------------------------------------------------
    // Enable TIM4 clock 
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
    
    // Set TIM4 time base 
    // TIM4CLK is set to 16 MHz, the TIM4 Prescaler = 32768, so the TIM4 counter
    // clock used is 16 MHz / 32768 = ~488.28 Hz. Clock period = 2.048 ms,
    // so duration = (timeout+1) * 2.048 ms
    TIM4_TimeBaseInit(TIM4_Prescaler_32768, timeout);
  
    // Clear TIM4 update flag
    // The flag may be set inside TIM4_TimeBaseInit(..) by setting UG bit
    TIM4_ClearFlag(TIM4_FLAG_Update);
    // Enable update interrupt 
    TIM4_ITConfig(TIM4_IT_Update, ENABLE);

    TIM4_Cmd(ENABLE);
}

void BSP_Rf_TimerDeactivate( void )
{
    TIM4_Cmd(DISABLE);

    // Disable update interrupt 
    TIM4_ITConfig(TIM4_IT_Update, DISABLE);
    // Clear TIM4 update flag. Mandatory for successfully enter Halt mode.
    //? TIM4_ClearITPendingBit(TIM4_IT_Update);
    TIM4_ClearFlag(TIM4_FLAG_Update);

    // Disable TIM4 clock 
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
}

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/