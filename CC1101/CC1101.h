/**
  ******************************************************************************
  * @file    Keyfob/CC1101.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   CC1101 transceiver low level functions header.
  ******************************************************************************
  * @attention
  * Copyright   : Ebyte electronic co.,LTD
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#ifndef __CC1101_H
#define __CC1101_H

#include "CC1101Reg.h"
#include <stdint.h>

//typedef enum { TX_MODE, RX_MODE } TRMODE;
//typedef enum {
//    BROAD_ALL,
//    BROAD_NO,
//    BROAD_0,
//    BROAD_0AND255 
//} AddressMode;
//typedef enum { BROADCAST, ADDRESS_CHECK} TX_DATA_MODE;

#define RF_DEFAULT_SYNC_WORD   0xD391
#define RF_MAX_FREQ_OFFSET_SAMPLE  4 // For 433 MHz        =8 for 868 MHz

/* Read a byte from the specified register */
extern uint8_t CC1101ReadReg(uint8_t addr);

/* Read some bytes from the registers continously */
extern void CC1101ReadMultiReg( uint8_t addr, uint8_t* buff, uint8_t size );

/* Write a byte to the specified register */
extern void CC1101WriteReg( uint8_t addr, uint8_t value );

/* Write some bytes to the specified register */
extern void CC1101WriteMultiReg( uint8_t addr, uint8_t* buff, uint8_t size );

/*Read a status register*/
extern uint8_t CC1101ReadStatus(uint8_t addr);

/*Write a command byte to the device*/
extern void CC1101WriteCmd(uint8_t command);

/*Set the device as TX mode or RX mode*/
//extern void CC1101SetTRMode(TRMODE mode);

/*Set the CC1101 into IDLE mode*/
//extern void CC1101SetIdle(void);

/*Send a packet*/
//extern void CC1101SendPacket(uint8_t address, uint8_t* txBuffer, uint8_t size);

/*Set the SYNC bytes of the CC1101*/
extern void CC1101SetSYNC(uint16_t sync);

/* Get received bytes count of CC1101 */
extern uint8_t CC1101GetRXCnt(void);

/* Write a packet to tx fifo */
//extern void CC1101WriteTxFifo(uint8_t* txBuffer, uint8_t size);

/* Read a packet from rx fifo */
extern uint8_t CC1101ReadRxFifo(uint8_t* rxBuffer, uint8_t rxBufSize);

/*Initialize the WOR function of CC1101*/
extern void  CC1101WORInit(void);
extern void  CC1101WORDeInit(void);

/*Initialize the CC1101, User can modify it*/
extern void CC1101Init(uint8_t maxPacketLength, uint8_t selfAddress, uint8_t numPreamble);

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/