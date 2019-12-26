/**
  ******************************************************************************
  * @file    Keyfob/bsp_system.h
  * @author  Yuri Mu
  * @version V1.0
  * @date    16-December-2019
  * @brief   System low level functions header.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 Yuri Mu</center></h2>
  *
*/
#ifndef __BSP_SYSTEM_H
#define __BSP_SYSTEM_H
#include <stdint.h>


extern void BSP_System_Init( void );
extern inline void BSP_System_Sleep( void );
extern inline void BSP_System_Halt( void );
extern uint8_t BSP_System_HasWorkingPeripheral( void );

#endif

/************************ (C) COPYRIGHT Yuri Mu *****END OF FILE****/