/**
  ******************************************************************************
  * @file    Keyfob/rf.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Rf transceiver functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
  * You may not use this file except with Keyfob hardware.
  *
  ******************************************************************************
  */ 
#include "rf.h"
#include "CC1101.h"
#include "bsp_rf.h"
#include "pult.h"

/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Rf
  * @{
  */

typedef struct {
    RfState  state;
    uint8_t  bufSize;
    uint8_t* buffer;
} Rf;

static Rf rf;

void Rf_Init(void)
{
    CC1101Init(RF_MAX_PACKET_LENGTH, RF_DEFAULT_SELF_ADDRESS);
}

RfState Rf_GetState(void)
{
    return rf.state;
}

void Rf_Send(uint8_t* aBuffer, uint8_t aBufLength)
{
    //If nothing to send, switch off the Rf transceiver chip 
    if (aBufLength == 0) {
        CC1101WriteCmd(CC1101_SIDLE);
        CC1101WriteCmd(CC1101_SPWD);
        BSP_Rf_SPIDeInit();
        rf.state = RF_STATE_POWERDOWN;
        return;
    }
    
    // Init fields
    if (aBufLength > RF_MAX_PACKET_LENGTH)
        aBufLength = RF_MAX_PACKET_LENGTH;
    rf.buffer = aBuffer;
    rf.bufSize = aBufLength;
    rf.state = RF_STATE_TX;
    
    // Disable 'TxRxComplete' external interrupt
    // Required, if currently in rx state
    BSP_Rf_InterruptsDisable();   

    // Start pramble duration timeout
    BSP_Rf_TimerActivate(RF_PREAMBLE_DURATION_TICKS);

    // Clear tx FIFO
    CC1101WriteCmd(CC1101_SIDLE);
    CC1101WriteCmd(CC1101_SFTX);
    
    // Start preamble
    CC1101WriteCmd(CC1101_STX);
}

// Preamble complete
void Rf_onTimerIsr(void)
{
    // Write tx fifo
    CC1101WriteReg(CC1101_TXFIFO, rf.bufSize);    
    CC1101WriteMultiReg(CC1101_TXFIFO, rf.buffer, rf.bufSize);
    
    // Disable this preamble duration timer
    BSP_Rf_TimerDeactivate();
    
    // Enable 'Send complete' external interrupt
    BSP_Rf_InterruptsEnable();
}

// Send or receive complete interrupt handler
void Rf_onTxRxCompleteIsr(void)
{
    // Disable 'TxRxComplete' external interrupt
    BSP_Rf_InterruptsDisable();   
    
    // Tx complete
    if (rf.state == RF_STATE_TX) {        

        // Set idle state
        rf.state = RF_STATE_IDLE;
        
        // Rise on send callback function
        Pult_onRfSend();
    }
    
    // Rx complete
    else {
        uint8_t len;
        
        // Set idle state
        rf.state = RF_STATE_IDLE;
        
        // Read and clear rx fifo. Returns actual bytes read.
        len = CC1101ReadRxFifo(rf.buffer, rf.bufSize);
        
        // Rise on receive callback function
        Pult_onRfRecv(len);
    }
}

void Rf_Recv(uint8_t* aBuffer, uint8_t aBufSize)
{
    //If no room, switch off the Rf transceiver chip 
    if (aBufSize == 0) {
        CC1101WriteCmd(CC1101_SIDLE);
        CC1101WriteCmd(CC1101_SPWD);
        BSP_Rf_SPIDeInit();
        rf.state = RF_STATE_POWERDOWN;
        return;
    }
    
    // Init fields
    rf.buffer  = aBuffer;
    rf.bufSize = aBufSize;
    rf.state   = RF_STATE_RX;
    
    // Clear rx FIFO
    CC1101WriteCmd(CC1101_SIDLE);
    CC1101WriteCmd(CC1101_SFRX);
    
    // Start low power receive
    CC1101WriteCmd(CC1101_SWOR);

    // Enable 'Recv complete' external interrupt
    BSP_Rf_InterruptsEnable();
}

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/