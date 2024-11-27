/*
 * STM32f407VG_RccDriver.h
 *
 *  Created on: Nov 20, 2024
 *      Author: yywvi
 */

#ifndef INC_STM32F407VG_RCCDRIVER_H_
#define INC_STM32F407VG_RCCDRIVER_H_
#include "STM32f407VG.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F407VG_RCCDRIVER_H_ */
