/**
  ******************************************************************************
  * @file    Keyfob/sound.c
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   Sound driver
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
#include "sound.h"
#include "bsp_sound.h"
#include "pult.h"


/** @addtogroup Keyfob
  * @{
  */

/** @addtogroup Sound
  * @{
  */

/* Global variables ---------------------------------------------------------*/

/* Private define and typedef ------------------------------------------------*/
typedef struct{
    uint8_t tone;   /*!< Sound tone index */
    uint8_t duration; /*!< Sound duration in timer clocks */
} Sound;


typedef struct{
    SoundTrackId id;     /*!< SoundTrack currently beeng played */
    uint8_t repeatsLeft; /*!< Downcounts of number of repeats left */
    Sound*  sound;       /*!< Pointer to sound structure currently beeng played */
} SoundTrack;

/* Private constants and variables -------------------------------------------*/
// Duration of half period of the sound tone in 4 us ticks.
static const uint16_t ToneHalfPeriod[] = { 
    0x0010, 0x0011, 0x0012, 0x0013, 0x0014, 0x0015, 0x0016, 0x0017, 
    0x0018, 0x0019, 0x001a, 0x001b, 0x001c, 0x001d, 0x001e, 0x001f, 
    0x0020, 0x0022, 0x0024, 0x0026, 0x0028, 0x002a, 0x002c, 0x002e, 
    0x0030, 0x0033, 0x0036, 0x0039, 0x003c, 0x003f, 0x0042, 0x0046, 
    0x004a, 0x004e, 0x0052, 0x0057, 0x005c, 0x0061, 0x0067, 0x006d, 
    0x0073, 0x007a, 0x0081, 0x0089, 0x0091, 0x009a, 0x00a3, 0x00ad, 
    0x00b7, 0x00c2, 0x00ce, 0x00da, 0x00e7, 0x00f5, 0x0104, 0x0114, 
    0x0125, 0x0137, 0x014a, 0x015e, 0x0173, 0x018a, 0x01a2, 0x01bc, 
    0x01d7, 0x01f4, 0x0213, 0x0234, 0x0257, 0x027c, 0x02a3, 0x02cd, 
    0x02f9, 0x0328, 0x035a, 0x038f, 0x03c7, 0x0403, 0x0443, 0x0487, 
    0x04cf, 0x051b, 0x056c, 0x05c2, 0x061e, 0x067f, 0x06e6, 0x0754, 
    0x07c9, 0x0845, 0x08c9, 0x0955, 0x09ea, 0x0a88, 0x0b30, 0x0be3, 
    0x0ca1, 0x0d6b, 0x0e41, 0x0f25, 0x1017, 0x1118, 0x1229, 0x134b, 
    0x147f, 0x15c6, 0x1722, 0x1894, 0x1a1d, 0x1bbe, 0x1d79, 0x1f50, 
    0x2145, 0x2359, 0x258e, 0x27e6, 0x2a64, 0x2d0a, 0x2fda, 0x32d7, 
    0x3604, 0x3964, 0x3cfa, 0x40c9, 0x44d5, 0x4922, 0x4db4, 0x528f, 
};

static const Sound SoundsShortBeep[] = {
    {35,   5},
    {0xff, 5},
    {0xff, 0}
};

static const Sound SoundsNormalBeep[] = {
    {40,   15},
    {0xff, 5},
    {0xff, 0}
};

static const Sound SoundsLongBeep[] = {
    {50,   50},
    {0xff, 15},
    {0xff, 0}
};
/*
static const Sound SoundsArmed[] = {
    {50,   1},
    {48,   1},
    {46,   1},
    {44,   1},
    {42,   1},
    {40,   1},
    {38,   1},
    {36,   1},
    {34,   1},
    {32,   1},
    {30,   1},
    {0xff, 5},
    {0xff, 0}
};

static const Sound SoundsDisarmed[] = {
    {30,   1},
    {32,   1},
    {34,   1},
    {36,   1},
    {38,   1},
    {40,   1},
    {42,   1},
    {44,   1},
    {46,   1},
    {48,   1},
    {50,   1},
    {0xff, 5},
    {0xff, 0}
};
*/
static const Sound SoundsArmed[] = {
    {60,   3},
    {50,   3},
    {40,   3},
    {30,   3},
    {20,   3},
    {0xff, 5},
    {0xff, 0}
};

static const Sound SoundsDisarmed[] = {
    {20,   3},
    {30,   3},
    {40,   3},
    {50,   3},
    {60,   3},
    {0xff, 5},
    {0xff, 0}
};

static const Sound*  const SoundTrackLookup[] = { 
    SoundsShortBeep,
    SoundsNormalBeep,
    SoundsLongBeep,
    SoundsArmed,
    SoundsDisarmed,
};

static SoundTrack soundTrack;

/* Public functions ---------------------------------------------------------*/
void Sound_Init( void )
{
}

/**
  * @brief  Starts to play soundtrack.
  * @param  aSoundTrackId - Id of soundtrack to play
  * @param  aRepeatsNum - repeats number. 0 - infinite play
  * @retval 0 - can't play; 1- play started successfully
  * @note : Used from interrupts or with disabled interrupts
  */
/*
uint8_t Sound_Play( SoundTrackId aSoundTrackId, uint8_t aRepeatsNum )
{
    uint8_t byte;
    
    // Stop previous soundtrack possibly beeng played 
    Sound_Stop();
    
    // Assign working vars
    if ( aSoundTrackId >= (sizeof(SoundTrackLookup[])/sizeof(SoundTrackLookup[0])) )
        aSoundTrackId  = 0;
    soundTrack.Id = aSoundTrackId;
    soundTrack.repeatsLeft = aRepeatsNum;
    soundTrack.sound = SoundTrackLookup[aSoundTrackId];
    
    // Invalid pointer to sound structure - fix //can't play
    if ( soundTrack.sound == 0 )
        //return 0;
        soundTrack.sound = SoundTrackLookup[0];
    
    // 'End of track' in first sound encountered - can't play
    byte = soundTrack.sound->duration;
    if ( byte == 0 )
        return 0;
    
    // Configure sound generation timers  
    BSP_Sound_ToneTimer_Init();
    BSP_Sound_DurationTimer_Init();
    
    // Arm sound generation timers  
    BSP_Sound_DurationTimer_Arm( byte );
    byte = soundTrack.sound->tone;
    if ( byte >= (sizeof(ToneHalfPeriod[])/sizeof(ToneHalfPeriod[0])) )
        toneHalfPeriod  = 0xfffe;
    else
        toneHalfPeriod  = ToneHalfPeriod[byte];
    BSP_Sound_ToneTimer_Arm( toneHalfPeriod );
    
    return 1;
}
*/

void Sound_Play( SoundTrackId aSoundTrackId, uint8_t aRepeatsNum )
{
    
    // Stop previous soundtrack possibly beeng played 
    Sound_Stop();
    
    // Assign working vars
    if ( aSoundTrackId >= (sizeof(SoundTrackLookup)/sizeof(SoundTrackLookup[0])) )
        aSoundTrackId  = SOUNDS_SHORT_BEEP;
    soundTrack.id = aSoundTrackId;
    soundTrack.repeatsLeft = aRepeatsNum;
    soundTrack.sound = (Sound*)SoundTrackLookup[aSoundTrackId];
    
    // Invalid pointer to sound structure - fix
    if ( soundTrack.sound == 0 )
        soundTrack.sound = (Sound*)SoundTrackLookup[0];
    
    // Configure sound generation timers  
    BSP_Sound_ToneTimer_Init();
    BSP_Sound_DurationTimer_Init();
    
    // Arm duration timer in 20 ms  
    BSP_Sound_DurationTimer_Arm( 1 );
    //BSP_Sound_ToneTimer_Arm( 0xfffe );
}

void Sound_Stop( void )
{
    // Disarm and deinit sound generation timers
    BSP_Sound_DurationTimer_Disarm();
    BSP_Sound_ToneTimer_Disarm();
    BSP_Sound_DurationTimer_DeInit();
    BSP_Sound_ToneTimer_DeInit();
}

void Sound_onDurationTimerIsr( void )
{
    uint16_t toneHalfPeriod;
    
    // End of track encountered
    while ( soundTrack.sound->duration == 0 ) {
        // Decrement repeats counter. Stop play, if repeats over
        if ( --soundTrack.repeatsLeft == 0 ) {
            Sound_Stop();
            Pult_onSoundPlayed(soundTrack.id);
            return;
        }        
        // else update the sound pointer with the first sound in the track.
        // Updated sound needs to be checked for 'end of track' again.
        // Therefore 'while' operator is nesessary here.
        soundTrack.sound = (Sound*)SoundTrackLookup[soundTrack.id];
    }
    
    // Arm sound generation timers  
    BSP_Sound_DurationTimer_Arm( soundTrack.sound->duration );
    if ( soundTrack.sound->tone >= (sizeof(ToneHalfPeriod)/sizeof(ToneHalfPeriod[0])) )
        toneHalfPeriod  = 0xfffe;
    else
        toneHalfPeriod  = ToneHalfPeriod[soundTrack.sound->tone];
    BSP_Sound_ToneTimer_Arm( toneHalfPeriod );
    
    // Move to next sound
    soundTrack.sound++;
}


#include <stdio.h>
void Sound_GenerateToneValues( void )
{
    uint16_t value;
    
    for ( int i = 0; i < 128; ++i )
    {
        value = 0x0010;     
        for ( int j = 0; j < i; ++j )
        {
            value += value/16;      
        } 
        printf( "0x%04x, ", value );
        if ( i % 8 == 7 )
            printf( "\r\n" );
    }    
}


/**
  * @}
  */
/**
  * @}
  */
/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/