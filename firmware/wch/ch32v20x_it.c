/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v20x_it.h"

void NMI_Handler(void) __attribute__((interrupt("machine")));
void HardFault_Handler(void) __attribute__((interrupt("machine")));
void SW_Handler(void) __attribute__((interrupt("machine")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      SW_Handler
 *
 * @brief   Software interrupt handler — jumps to main application.
 *
 * @return  none
 */
void SW_Handler(void)
{
    __asm("la  a6, _firmware_start_addr");
    __asm("jr  a6");

    while(1);
}


