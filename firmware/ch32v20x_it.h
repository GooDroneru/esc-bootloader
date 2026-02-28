/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.h
 * Description        : Interrupt handler prototypes.
*******************************************************************************/
#ifndef __CH32V20X_IT_H
#define __CH32V20X_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __CH32V20X_IT_H */
