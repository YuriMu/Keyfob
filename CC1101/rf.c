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
    uint8_t  numPreamble; //0-2bytes; 1-3; 2-4;3-6;4-8;5-12;6-16;7-24;8...-86ms 
    uint8_t  bufSize;
    uint8_t* buffer;
    int8_t   freqOffsAcc[RF_MAX_ADDRESS + 1];
} Rf;

static Rf rf;

void Rf_Init(void)
{
    rf.numPreamble = 8; // 2 - 4 bytes
    CC1101Init(RF_MAX_PACKET_LENGTH, RF_DEFAULT_SELF_ADDRESS, rf.numPreamble & 0x07);
}

RfState Rf_GetState(void)
{
    return rf.state;
}

// Init data transmittion.
// If bufLength = 0, then issue infinite preamble only
// buffer[0] must contain destination address even if bufLength = 0. 
void Rf_Send(uint8_t* buffer, uint8_t bufLength)
{
    uint8_t addr;
    
    // Disable 'TxRxComplete' external interrupt
    // Required, if currently in rx state
    BSP_Rf_InterruptsDisable();   

    // Clear tx FIFO
    CC1101WriteCmd(CC1101_SIDLE);
    CC1101WriteCmd(CC1101_SFTX);


    // Adjust frequency with offset, accumulated in previous packet reading.
    // Offset value is unique for each remote device. 
    addr = buffer[0];
    if (addr <= RF_MAX_ADDRESS) 
        CC1101WriteReg(CC1101_FSCTRL0, rf.freqOffsAcc[addr]);    

    
    //If have data to send, save it and limit preamble duration 
    if (bufLength) {
//static uint8_t i;
// i = CC1101ReadStatus(CC1101_MARCSTATE);

        // Init fields
        if (bufLength > RF_MAX_PACKET_LENGTH)
            bufLength = RF_MAX_PACKET_LENGTH;
        rf.buffer = buffer;
        rf.bufSize = bufLength;
        rf.state = RF_STATE_TX;

        if (rf.numPreamble < 8) {
            // Write tx fifo
            CC1101WriteReg(CC1101_TXFIFO, rf.bufSize);    
            CC1101WriteMultiReg(CC1101_TXFIFO, rf.buffer, rf.bufSize);
        
            // Enable 'Send complete' external interrupt
            BSP_Rf_InterruptsEnable();
        }
        else {
            // Start pramble duration timeout
            BSP_Rf_TimerActivate(RF_PREAMBLE_DURATION_TICKS);
        }
    }

    // Start preamble
    CC1101WriteCmd(CC1101_STX);
}

// Preamble complete
void Rf_onTimerIsr(void)
{
    // Disable this preamble duration timer
    BSP_Rf_TimerDeactivate();
    
    // Write tx fifo
    CC1101WriteReg(CC1101_TXFIFO, rf.bufSize);    
    CC1101WriteMultiReg(CC1101_TXFIFO, rf.buffer, rf.bufSize);
    
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

        // Transceiver is configured to move to idle state after send
        rf.state = RF_STATE_IDLE;
        
        // Rise on send callback function
        Pult_onRfSend();
    }
    
    // Rx complete
    else {
        uint8_t len;
        uint8_t addr;
        
        // Set idle state
        rf.state = RF_STATE_IDLE;
        
        // Read and clear rx fifo. Returns actual bytes read.
        len = CC1101ReadRxFifo(rf.buffer, rf.bufSize);
        
        // Accumulate frequency offset between this transceiver and remote one.
        // Must be read in IDLE state, therefore run after CC1101ReadRxFifo(..).
        // The offset will be written to FSCTRL0.FREQOFF at next send.
        addr = rf.buffer[0];
        if (addr <= RF_MAX_ADDRESS) {        
            //rf.freqOffAcc[addr] += MIN(CC1101ReadStatus(CC1101_FREQEST), RF_MAX_FREQ_OFFSET_SAMPLE);
            rf.freqOffsAcc[addr] += CC1101ReadStatus(CC1101_FREQEST);
        }
        
        // Rise on receive callback function
        Pult_onRfRecv(len);
    }
}

// Init receive state, returns RSSI value.
// If aBufSize = 0, returns  RSSI value only.
int8_t Rf_Recv(uint8_t* aBuffer, uint8_t aBufSize)
{
    //If no room for received data, return RSSI value 
    if (aBufSize ==0 )
    {
        int8_t rssi;
        
        rssi = CC1101ReadStatus(CC1101_RSSI);
        return (rssi/2 - (int8_t)74);  
    }
    
    // Init fields
    rf.buffer  = aBuffer;
    rf.bufSize = aBufSize;
    rf.state   = RF_STATE_RX;
    
    // Clear rx FIFO
    CC1101WriteCmd(CC1101_SIDLE);
    CC1101WriteCmd(CC1101_SFRX);
    
    // Start low power receive
    CC1101WriteCmd(CC1101_SWORRST);
    CC1101WriteCmd(CC1101_SWOR);
    //CC1101WriteCmd(CC1101_SRX);

    // Enable 'Recv complete' external interrupt
    BSP_Rf_InterruptsEnable();

    return 1;
}

void Rf_PowerDown(void)
{
    // RC osc power down?
    
    
    
        CC1101WriteCmd(CC1101_SIDLE);
        CC1101WriteCmd(CC1101_SPWD);
        BSP_Rf_SPIDeInit();
        rf.state = RF_STATE_POWERDOWN;
}
/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/