/**
  ******************************************************************************
  * @file    Keyfob/buttons.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Low power buttons driver
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
#include "buttons.h"
#include "bsp_buttons.h"
#include "pult.h"


/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Buttons
  * @{
  */

/* Private define and typedef ------------------------------------------------*/

/*!< 75 Margin for long pressure detection  */
#define BUTTON_LONG_CLICK_LIMIT     ((uint8_t)(1500/BUTTON_TIMER_PERIOD_MS))
/*!< 15 Time between clicks for double click detection */
#define BUTTON_BETWEEN_CLICKS_LIMIT ((uint8_t)(300/BUTTON_TIMER_PERIOD_MS)) 

#define BUTTON_STATE_MSK   0xc0 /*!< Mask of states for clicks generating state machine*/

#define BUTTON_STATE_IDLE          (0 << 6)    /*!<  */
#define BUTTON_STATE_PRESSED       (1 << 6)    /*!<  */
#define BUTTON_STATE_RELEASED      (2 << 6)    /*!<  */

typedef struct{
    uint8_t clicksAccum;  /*!< State and clicks quantity according to masks */
    uint8_t counter;      /*!< Duration of pressed or released phase */
} Button;

typedef struct{
    uint8_t depressed;    /*!< Bitmask of debounced buttons */
    uint8_t previous;     /*!< Bitmask of previous sample of button's inputs */
    uint8_t timeout;      /*!< Counts time, when all buttons released. This is 'end of  pattern' criteria*/
    Button  item[BUTTON_NUM]; /*!< Buttons array */
} Buttons;


/* Private variables ---------------------------------------------------------*/
static Buttons buttons;          /*!< Button's set info */  

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Generate normal- and long- click series from multiple buttons
  * @param  None.
  * @retval None.
  * Note : 
  */
static void ButtonsTimerIsr(void)
{
    uint8_t depressed;
    
    //Enable pullups in very beginning, to have time for input voltage to stabilize.
    BSP_Buttons_EnablePullup();
        
    depressed = buttons.depressed; // get buttons debounced pins states. 1-depressed. 
    
    for (int i = 0; i < BUTTON_NUM; ++i)
    {
        uint8_t state;       // phase of execution
        uint8_t clicksAccum; // click quantity accuumulator
        uint8_t counter;     // pressed/released state duration counter 
        
        state  =     buttons.item[i].clicksAccum & BUTTON_STATE_MSK;
        clicksAccum = buttons.item[i].clicksAccum & (BUTTON_NORMAL_CLICKS_MSK | BUTTON_LONG_CLICKS_MSK);
        counter =    buttons.item[i].counter;
        
        switch (state) {            
            // Waiting for first pressure.
            case BUTTON_STATE_IDLE: {
                // First pressure occured - init variables and go to 'pressed' state.
                if (depressed & 0x01) {
                    clicksAccum = 0;
                    counter = 0;
                    state = BUTTON_STATE_PRESSED;
                }                    
                break;
            }
                        
            // Pressure in progress
            case BUTTON_STATE_PRESSED: {                
                // Pressure proceeds - keep track of click duration.
                if (depressed & 0x01) {
                    if (counter < 0xff)
                        counter++;
                } 
                
                // Button released - means click composed
                else {
                    // normal click occured
                    if (counter < BUTTON_LONG_CLICK_LIMIT) {
                        if ((clicksAccum & BUTTON_NORMAL_CLICKS_MSK) < BUTTON_NORMAL_CLICKS_MSK) 
                            clicksAccum++;         // increment normal clicks quantity
                    }                    
                    // long click occured
                    else {
                        if (clicksAccum < BUTTON_LONG_CLICKS_MSK) 
                            clicksAccum += (BUTTON_NORMAL_CLICKS_MSK + 1); // increment long clicks quantity 
                    }

                    counter = 0;                // prepare counter               
                    state = BUTTON_STATE_RELEASED; // go to 'button released' state
                }
                break;
            }
            
            // Button is released
            case BUTTON_STATE_RELEASED: {                
                // Button pressed again before guard interval elapsed
                if (depressed & 0x01) {
                    counter = 0;                    
                    state = BUTTON_STATE_PRESSED; // go to 'button pressed' state
                }                    
                
                // Release proceeds - increment duration counter until guard 
                // interval elapse. This is criteria of clicks series completion.
                else {                    
                    if (counter++ >= BUTTON_BETWEEN_CLICKS_LIMIT) {
                        state = BUTTON_STATE_IDLE;
                    }                    
                }
                break;
            }
            
            default: {
                break;
            }
        }
        
        buttons.item[i].counter = counter;
        buttons.item[i].clicksAccum = state | clicksAccum;

        depressed >>= 1; // select next button debounced pin state
    }
    
    // Count, how long all buttons are beeng released 
    if (buttons.depressed)
        buttons.timeout = 0;
    else 
        buttons.timeout++;
    
    // Max buttons inactivity interval not yet elapsed - monitor and debounce pins 
    if (buttons.timeout < (BUTTON_BETWEEN_CLICKS_LIMIT +3)) {
    
        uint8_t current;
    
        /* Perform the debouncing of buttons. The algorithm for debouncing
        * adapted from the book "Embedded Systems Dictionary" by Jack Ganssle
        * and Michael Barr, page 71.
        */
        current = BSP_Buttons_GetState(); /* read all buttons */
        //tmp = buttons.depressed; /* save the debounced depressed buttons */
        buttons.depressed |= (buttons.previous & current); /* set depressed */
        buttons.depressed &= (buttons.previous | current); /* clear released */
        buttons.previous   = current; /* update the history */
        //tmp ^= buttons.depressed;     /* changed debounced depressed */
    }
    
    // Max buttons inactivity interval elapsed - work finished,
    // do end housekeeping and give away the results   
    else {        
        ClicksSet clicksSet;

        buttons.timeout = 0;
        
        // Gather resulting set of clicks from all buttons
        clicksSet.asLong = 0;     
        for (int i = 0; (i < BUTTON_NUM) && (i < 4); ++i) {
            clicksSet.asByte[i] = buttons.item[i].clicksAccum & (BUTTON_NORMAL_CLICKS_MSK | BUTTON_LONG_CLICKS_MSK);
        }
        
        // Free RTC wakeup timer, specifically: 
        // 1) Unbind a specific Isr handler from TimerIsr for use in other modules
        TimerIsr = 0;
        // 2) Disable timer
        BSP_Timer_Disarm();
        // 3) Disable interrupt and deattach from clocks 
        BSP_Timer_DeInit();
        
        // Enable external interrupts and pullup resistors of button inputs. 
        BSP_Buttons_Init();
        
        Pult_onClicks(clicksSet.asLong);
    }
}
 
// Public functions ------------------------------------------------------------

void Button_onPressIsr( void )
{
    if (TimerIsr == 0) {
        // Engage RTC wakeup timer, specifically: 
        // 1) Bind a specific Isr to TimerIsr
        TimerIsr = ButtonsTimerIsr;        
        // 2) Configure RTC wakeup timer
        BSP_Timer_Init();
        // 3) Set wakeup period (in timer steps) and launch counting
        BSP_Timer_Arm( BUTTON_TIMER_PERIOD_RAW );
        
        // Disable input interrupts for all buttons 
        BSP_Buttons_PinInterruptDisable();
    }    
}

/**
  * @}
  */
/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/