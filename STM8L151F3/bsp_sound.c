/**
  ******************************************************************************
  * @file    Keyfob/bsp_sound.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Sound generation timers low level functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  *
  ******************************************************************************
  */ 
#include "bsp_sound.h"
#include "stm8l15x.h"

/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Sound
  * @{
  */

/* Private define and typedef ------------------------------------------------*/
#define BSP_SOUND_PORT     GPIOB         /*!< sound port*/
#define BSP_SOUND_PIN      GPIO_Pin_0    /*!< sound pin*/

/* Private variables ---------------------------------------------------------*/

/* Public functions ---------------------------------------------------------*/
void BSP_Sound_Init( void )
{
    // TIM2 Channel1 GPIO configuration: PB0 Tbd _Slow 
    GPIO_Init(BSP_SOUND_PORT, BSP_SOUND_PIN, GPIO_Mode_Out_PP_Low_Fast);
}
      
/**
  * @brief  Configure a sound duration timer peripheral 
  * @note   This timer is a cascade of two timers TIM4 + TIM3
  *         TIM4 generates a ~10 ms time base for TIM3
  * @param  None
  * @retval None
  */
void BSP_Sound_DurationTimer_Init( void )
{
    //------ TIM4 init ------------------------------------------------
    // Enable TIM4 clock 
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
    
    TIM4_SelectOutputTrigger(TIM4_TRGOSource_Update);

    // Set TIM4 time base 
    // TIM4CLK is set to 16 MHz, the TIM4 Prescaler = 32768, so the TIM4 counter
    // clock used is 16 MHz / 32768 = ~488.28 Hz.
    // We need to generate a time base equal to ~10 ms
    // so TIM4_PERIOD = (0.01 * 488.28 - 1) = 4. Precisely 10.25 ms. Freq = 488.28/5 = 97.65 Hz
    TIM4_TimeBaseInit(TIM4_Prescaler_32768, 4);
  
    //------ TIM3 init ------------------------------------------------
    // Enable TIM3 clock 
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);

    // Time Base configuration 
    // TIM3 is connected to TIM4 Update so TIM3CLK is equal to 97.65 Hz.
    // TIM3 Prescaler is equal to 1 so the TIM3 counter clock used is 97.65 / 1 = 97.65 Hz
    // TIM3 max update frequency = TIM3CLK / (TIM3_PERIOD + 1) * TIM3Prescaler
    //                                  = 97.65 / (1 + 1) * 1 = 48.82 Hz 
    TIM3_TimeBaseInit(TIM3_Prescaler_1, TIM3_CounterMode_Up, 1);

    // TIM3 Slave Mode selection: Gated mode 
    TIM3_SelectInputTrigger(TIM3_TRGSelection_TIM4);
    TIM3_SelectSlaveMode(TIM3_SlaveMode_Gated);
   
    //TIM3_UpdateRequestConfig(TIM3_UpdateSource_Global);// Already such after reset

    // Clear TIM3 update flag
    // The flag may be set inside TIM4_TimeBaseInit(..) by setting UG bit
    TIM3_ClearFlag(TIM3_FLAG_Update);
    // Enable update interrupt 
    TIM3_ITConfig(TIM3_IT_Update, ENABLE);
}

void BSP_Sound_DurationTimer_DeInit( void )
{
    //--------- TIM3, TIM4 disable -------------------------------------------------
    BSP_Sound_DurationTimer_Disarm();

    
    //--------- TIM3 deinit -------------------------------------------------
    // Disable slave mode to clock the prescaler directly with the internal clock 
    TIM3_InternalClockConfig();
    
    // Disable update interrupt 
    TIM3_ITConfig(TIM3_IT_Update, DISABLE);
    // Clear TIM3 update flag. Mandatory for successfully enter Halt mode.
    //? TIM3_ClearITPendingBit(TIM3_IT_Update);
    TIM3_ClearFlag(TIM3_FLAG_Update);    
    
    // Disable TIM3 clock 
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, DISABLE);
    
    
    //--------- TIM4 deinit -------------------------------------------------
    // Clear TIM4 update flag. Mandatory for successfully enter Halt mode.
    //? TIM4_ClearITPendingBit(TIM4_IT_Update);
    TIM4_ClearFlag(TIM4_FLAG_Update);

    // Disable TIM4 clock 
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
}

// Set sound duration (in 20 ms ticks) and launch counting
void BSP_Sound_DurationTimer_Arm( uint8_t aDuration )
{
    uint16_t arr;
    
    // Enable TIM4 
    TIM4_Cmd(ENABLE);
        
    // TIM3CLK = ~10.25 ms
    //  Timeout = 10.25 * (arr + 1) = 10.25 * aDuration * 2 = 20.5 ms * aDuration.
    arr = aDuration * 2 - 1 ;
    // Value is written immediatly into shadow registers, because
    // TIM3_ARR register is not buffered after device reset
    TIM3_SetAutoreload(arr);
    // Enable TIM3
    TIM3_Cmd(ENABLE);
}

// Stop sound duration timer
void BSP_Sound_DurationTimer_Disarm( void )
{
    // Disable TIM3
    TIM3_Cmd(DISABLE);
    
    // Disable TIM4 
    TIM4_Cmd(DISABLE);
}

/*-------------- Tone timer --------------------------------------------------*/

void BSP_Sound_ToneTimer_Init(void)
{
    // Enable TIM2 clock 
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);

    // Time Base configuration 
    
    // TIM2 counter clock used is HSI / 64 = 16 000 / 64 = 250 kHz
    //TIM2_PrescalerConfig(TIM2_Prescaler_64, TIM2_PSCReloadMode_Immediate);
    //TIM2_CounterModeConfig(TIM2_CounterMode_Up);
    TIM2_TimeBaseInit(TIM2_Prescaler_64, TIM2_CounterMode_Up, 4);

    // Output channel 1 configuration 
    
    // TIM2 Channel 1 output in toggle mode with low polarity
    //TIM2_SelectOCxM(TIM2_Channel_1, TIM2_OCMode_Toggle); // disables channel!
    //TIM2_OC1PolarityConfig(TIM2_OCPolarity_High); 
    //TIM2_CCxCmd(TIM2_Channel_1, ENABLE);
    // Write OCiPE = 0 to disable the preload register. Not important here
    TIM2_OC1Init(TIM2_OCMode_Toggle, TIM2_OutputState_Enable, 2, TIM2_OCPolarity_Low, TIM2_OCIdleState_Set);
    // TIM2 Main Output Enable
    TIM2_CtrlPWMOutputs(ENABLE);
}

void BSP_Sound_ToneTimer_DeInit(void)
{
    // TIM2 Disable
    BSP_Sound_ToneTimer_Disarm();
    
    // Clear TIM4 update and compare flags. Mandatory for successfully enter Halt mode.
    //TIM2_ClearITPendingBit(TIM2_IT_Update);
    TIM2_ClearFlag(TIM2_FLAG_Update);
    TIM2_ClearFlag(TIM2_FLAG_CC1);

    // TIM2 clock disable 
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, DISABLE);
}

/**
  * @brief  Set tone frequency and launch counting 
  * @param  Half period value (in timer ticks) of required frequency 
  * @retval None
  */
void BSP_Sound_ToneTimer_Arm( uint16_t aToneHalfPeriod )
{
    // Sets the TIM2 Channel1 output frequency 
    //  = TIM2CLK / (TIM2 Prescaler * 2 * (aToneHalfPeriod + 1))
    //  Max = 16 000 / (64 * 2 * (16+1)) = 7353 Hz 
    //TIM2_SetAutoreload(aToneHalfPeriod); Use TIM2_TimeBaseInit(..), because it sets UG bit. 
    TIM2_TimeBaseInit(TIM2_Prescaler_64, TIM2_CounterMode_Up, aToneHalfPeriod);
    // Sets the TIM2 Capture Compare1 Register value in the middle of halfperiod.
    TIM2_SetCompare1(aToneHalfPeriod/2);

    TIM2_Cmd(ENABLE);
}

/**
  * @brief  Disable timer 
  * @param  None
  * @retval None
  */
void BSP_Sound_ToneTimer_Disarm(void)
{
    TIM2_Cmd(DISABLE);
}

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/