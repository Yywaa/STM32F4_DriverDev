/*
 * STM32f407VG_RccDriver.c
 *
 *  Created on: Nov 20, 2024
 *      Author: yywvi
 */
#include "STM32f407VG_RccDriver.h"
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};
uint16_t APB2_PreScaler[4] = {2,4,8,16};




uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t  clksrc,ahbp,ahpb1,temp;
	clksrc = ((RCC->CFGR >> 2)& 0x3);
	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk =RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}
	temp = ((RCC->CFGR >> 10)& 0x7);
	if(temp < 4)
	{
		ahpb1 = 1;
	}else
	{
		ahpb1 = APB1_PreScaler[temp - 4];
	}
	pclk1 = SystemClk/ahbp/ahpb1;


	return pclk1;

}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t  clksrc,ahbp,ahpb2,temp;
	clksrc = ((RCC->CFGR >> 2)& 0x3);
	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk =RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}
	temp = ((RCC->CFGR >> 13)& 0x7);
	if(temp < 4)
	{
		ahpb2 = 1;
	}else
	{
		ahpb2 = APB2_PreScaler[temp - 4];
	}
	pclk1 = SystemClk/ahbp/ahpb2;


	return pclk1;

}
