/**
  ******************************************************************************
  * @file    Keyfob/bsp_rf.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Rf transceiver hw interface header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#ifndef __BSP_RF_H
#define __BSP_RF_H
#include "stm8l15x.h"
#include "rf.h"

#define spiCurrentDevice  spi1CurrentDevice

#define RF_SPI                      SPI1
#define RF_SPI_CLK                  CLK_Peripheral_SPI1
#define RF_SPI_MISO_PIN             GPIO_Pin_7                  
#define RF_SPI_MISO_PORT            GPIOB                       
//#define RF_SPI_MOSI_PIN             GPIO_Pin_6                  
//#define RF_SPI_MOSI_PORT            GPIOB                       
//#define RF_SPI_SCK_PIN              GPIO_Pin_5                  
//#define RF_SPI_SCK_PORT             GPIOB                       
#define RF_CS_PORT                  GPIOB                       
#define RF_CS_PIN                   GPIO_Pin_4                  

#define RF_GDO0_PORT                GPIOC
#define RF_GDO0_PIN                 GPIO_Pin_4                  
#define RF_GDO0_EXTI                EXTI_Pin_4   // Interrupt configure edge bit
#define RF_GDO0_EXTI_IT             EXTI_IT_Pin4 // Interrupt pending bit

#define RF_GDO2_PORT                GPIOB
#define RF_GDO2_PIN                 GPIO_Pin_3                  

#define CC_CSN_HIGH()   GPIO_SetBits(RF_CS_PORT, RF_CS_PIN)

//#define CC_IRQ_READ()   GPIO_ReadInputDataBit(RF_GDO0_PORT, RF_GDO0_PIN)

#define BSP_Rf_InterruptsEnable()       EXTI_ClearITPendingBit(RF_GDO0_EXTI_IT);\
                                        RF_GDO0_PORT->CR2 |= RF_GDO0_PIN
    
#define BSP_Rf_InterruptsDisable()      RF_GDO0_PORT->CR2 &= (uint8_t)(~(RF_GDO0_PIN))
                                            
// timer step in milliseconds 
#define RF_TIMER_RESOLUTION_MS      2.048  
// Preamble duration in timer's ticks. Each tick = RF_TIMER_RESOLUTION_MS milliseconds
#define RF_PREAMBLE_DURATION_TICKS  ((uint8_t)(RF_PREAMBLE_DURATION_MS/RF_TIMER_RESOLUTION_MS - 1))

extern void    BSP_Rf_Init(void);
extern void    CC_CSN_LOW(void);
extern void    BSP_Rf_SPIDeInit(void);
extern uint8_t SPI_ExchangeByte(uint8_t input);
extern void    BSP_Rf_TimerActivate(uint8_t timeout);
extern void    BSP_Rf_TimerDeactivate(void);

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/