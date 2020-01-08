/**
  ******************************************************************************
  * @file    Keyfob/bsp_spi.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Spi functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  *
  ******************************************************************************
  */ 
#include "bsp_spi.h"


/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Bsp
  * @{
  */

SpiDeviceId spi1CurrentDevice;

void BSP_SPI_Init(SPI_TypeDef* SPIx)
{
    // Enable SPI clock 
    if (SPIx == SPI1)
        CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
    else
        return;
    
    SPI_DeInit(SPIx);
    
    // 8MHZ
    SPI_Init(SPIx, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_2,
            SPI_Mode_Master, SPI_CPOL_Low, SPI_CPHA_1Edge,
            SPI_Direction_2Lines_FullDuplex, SPI_NSS_Soft, 0x07);

    SPI_Cmd(SPIx, ENABLE);
    
    // Configure SPI GPIO
    if (SPIx == SPI1) {
        GPIO_Init(GPIOB, GPIO_Pin_7, GPIO_Mode_In_PU_No_IT);      // MISO
        GPIO_Init(GPIOB, GPIO_Pin_5|GPIO_Pin_6, GPIO_Mode_Out_PP_Low_Slow); // SCK,MOSI
    }
    else
        return;
}

void BSP_SPI_DeInit(SPI_TypeDef* SPIx)
{
    // Disable SPI
    SPI_Cmd(SPIx, DISABLE);
    
    // DeConfigure SPI GPIO
    //if (SPIx == SPI1) 
    //    GPIO_Init(GPIOB, GPIO_Pin_5|GPIO_Pin_6, GPIO_Mode_In_PU_No_IT); // SCK,MOSI
    //else
    //    return;

    // Disable SPI clock 
    if (SPIx == SPI1)
        CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE);
    else
        return;
}


uint8_t BSP_SPI_ExchangeByte(SPI_TypeDef* SPIx, uint8_t input)
{
    SPI_SendData(SPIx, input);
    while (RESET == SPI_GetFlagStatus(SPIx, SPI_FLAG_TXE));  	
    while (RESET == SPI_GetFlagStatus(SPIx, SPI_FLAG_RXNE)); 
    return (SPI_ReceiveData(SPIx));
}

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/