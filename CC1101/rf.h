/**
  ******************************************************************************
  * @file    Keyfob/rf.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Rf transceiver functions header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#ifndef __RF_H
#define __RF_H
#include <stdint.h>

#define RF_PREAMBLE_DURATION_MS   86

#define RF_MAX_PACKET_LENGTH      60
#define RF_MAX_ADDRESS            31
#define RF_DEFAULT_SELF_ADDRESS   1  // 1..RF_MAX_ADDRESS permitted

typedef enum {
    RF_STATE_POWERDOWN = 0,
    RF_STATE_IDLE      = 1,
    RF_STATE_TX        = 2,
    RF_STATE_RX        = 3
} RfState;

extern void    Rf_Init(void);
extern RfState Rf_GetState(void);
extern void    Rf_Send(uint8_t* aBuffer, uint8_t aBufLength);
extern int8_t  Rf_Recv(uint8_t* aBuffer, uint8_t aBufSize);
extern void    Rf_PowerDown(void);

extern void Rf_onTimerIsr(void);
extern void Rf_onTxRxCompleteIsr(void);

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/