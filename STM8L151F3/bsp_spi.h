/**
  ******************************************************************************
  * @file    Keyfob/bsp_spi.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Spi functions header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#ifndef __BSP_SPI_H
#define __BSP_SPI_H
#include "stm8l15x.h"

typedef enum {
    SPI_DEVICE_NONE = 0,
    SPI_DEVICE_CC1101
}SpiDeviceId;

extern SpiDeviceId spi1CurrentDevice;

extern void    BSP_SPI_Init(SPI_TypeDef* SPIx);
extern void    BSP_SPI_DeInit(SPI_TypeDef* SPIx);
extern uint8_t BSP_SPI_ExchangeByte(SPI_TypeDef* SPIx, uint8_t input);

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/