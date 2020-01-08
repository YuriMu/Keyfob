/**
  ******************************************************************************
  * @file    Keyfob/pult.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Application functionality
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
#include "pult.h"
#include "bsp_timer.h"
#include "buttons.h"
#include "rf.h"


/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Pult
  * @{
  */

#define PULT_RF_SEND_TIMEOUT_MS  500
#define PULT_RF_RECV_TIMEOUT_MS  3000

#define PULT_RF_MODE_TRANSMITTER  0
#define PULT_RF_MODE_RECEIVER     1

typedef struct {
    ClicksSet clicksSet;
    uint8_t   rfMode;
} Pult;

/* Private variables ---------------------------------------------------------*/
Pult pult;

static uint8_t sendBuffer[RF_MAX_PACKET_LENGTH] = {
    RF_DEFAULT_SELF_ADDRESS,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16
};
static uint8_t recvBuffer[RF_MAX_PACKET_LENGTH];

/* Private functions ---------------------------------------------------------*/
void Pult_Init(void)
{
}



static void PultTimerIsr(void)
{
    // Free this timer for use in other modules 
    BSP_Timer_Deactivate();
    
    // In Receiver mode
    if (pult.rfMode == PULT_RF_MODE_RECEIVER) {
        // If send fails, return back to receive state  
        Rf_Recv(recvBuffer, sizeof(recvBuffer));
    }
    // In Transmitter mode
    else {
        // Power down rf transceiver
        Rf_Send(0, 0);
    
        // Play error 
        Sound_Play(SOUNDS_LONG_BEEP, 1);
    }
}


void Pult_onClicks(uint32_t aClicksSet)
{
    uint8_t repeats;
    /*
    for(uint16_t j=0; j<1000; j++){
         GPIOB->ODR ^= 1;
        for(uint16_t i=0; i<300; i++);
        //GPIOB->ODR = 0xff;
        //for(uint16_t i=0; i<300; i++);
    }
*/

    pult.clicksSet.asLong = aClicksSet;
    
    repeats = pult.clicksSet.asByte[0] & BUTTON_NORMAL_CLICKS_MSK;
    if( repeats ) {
        pult.clicksSet.asByte[0] &= ~BUTTON_NORMAL_CLICKS_MSK;
        if( repeats == 1 ) {
            if (BSP_Timer_Activate(PultTimerIsr, BSP_TIMER_TICKS(PULT_RF_SEND_TIMEOUT_MS))) {
                pult.rfMode = PULT_RF_MODE_TRANSMITTER;
                // Don't play sound between Rf_Send(..) and Pult_onRfSend(),
                // because they use the same timer TIM4.
                Rf_Send(sendBuffer, sizeof(sendBuffer));
            }    
        }
        else {            
            Sound_Play(SOUNDS_ARMED, repeats); //SOUNDS_SHORT_BEEP
        }
        return;
    }
    
    repeats = (pult.clicksSet.asByte[0] & BUTTON_LONG_CLICKS_MSK) / (BUTTON_NORMAL_CLICKS_MSK + 1);
    if( repeats ) {
        pult.clicksSet.asByte[0]  = 0;
        if( repeats == 1 ) {
            if (pult.rfMode == PULT_RF_MODE_TRANSMITTER) {
                // Set rf in receiver mode 
                pult.rfMode = PULT_RF_MODE_RECEIVER;
                Rf_Recv(recvBuffer, sizeof(recvBuffer));
                // Play 'In receiver mode' 
                Sound_Play(SOUNDS_SHORT_BEEP, 1);
            }
            else {
                // Return to transmitter mode
                pult.rfMode = PULT_RF_MODE_TRANSMITTER;
                
                // Power down the rf transceiver
                Rf_Send(0, 0);
    
                // Play success 
                Sound_Play(SOUNDS_SHORT_BEEP, 2);
            }
        }
        else {            
            Sound_Play(SOUNDS_ARMED, repeats); //SOUNDS_SHORT_BEEP
        }
        return;
    }
}
                            
void Pult_onSoundPlayed(SoundTrackId aSoundTrackId)
{
    aSoundTrackId = aSoundTrackId;
    
    Pult_onClicks(pult.clicksSet.asLong);
}

void Pult_onRfSend(void)
{
    // In Receiver mode
    if (pult.rfMode == PULT_RF_MODE_RECEIVER) 
        // free this timer for use in other modules 
        BSP_Timer_Deactivate();
    // In Transmitter mode
    else
        // start receive packet timeout
        BSP_Timer_Arm(BSP_TIMER_TICKS(PULT_RF_RECV_TIMEOUT_MS));
    
    // Set rf transceiver in receive state 
    Rf_Recv(recvBuffer, sizeof(recvBuffer));
}

void Pult_onRfRecv(uint8_t length)
{
    // Packet received    
    if (length == sizeof(recvBuffer)) {
        // In Receiver mode
        if (pult.rfMode == PULT_RF_MODE_RECEIVER) {
            // Send back the received packet. If impossible, return to receive state  
            for( int i = 0; i < sizeof(recvBuffer); i ++ )
                sendBuffer[i] = recvBuffer[i];
            if (BSP_Timer_Activate(PultTimerIsr, BSP_TIMER_TICKS(PULT_RF_SEND_TIMEOUT_MS))) {
                // Don't play sound between Rf_Send(..) and Pult_onRfSend(),
                // because they use the same timer TIM4.
                Rf_Send(sendBuffer, sizeof(sendBuffer));
            }    
            else {
                Rf_Recv(recvBuffer, sizeof(recvBuffer));
            }
        }
        // In Transmitter mode
        else {
            // Power down rf transceiver
            Rf_Send(0, 0);
    
            // Free this timer for use in other modules 
            BSP_Timer_Deactivate();
        
            // Play success 
            Sound_Play(SOUNDS_SHORT_BEEP, 2);
        }
    }
    // Packet don't received or pult is in permanent receiver mode - try to receive again.  
    else {
        Rf_Recv(recvBuffer, sizeof(recvBuffer));
    }
}

/**
  * @}
  */
/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/