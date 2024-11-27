/*
 * 005Button_Interrupt.c
 *
 *  Created on: Nov 2, 2024
 *      Author: Yongwei Yuan
 *      Success!
 */

#include "STM32f407VG.h"
#include <string.h>
#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay()
{
    for (uint32_t i = 0; i < 500000 / 2; i++)
    {
    }
}
int main(void)
{

    GPIO_Handle_t GpioLed, GpioBtn;
    memset(&GpioLed,0,sizeof(GpioLed));
    memset(&GpioBtn,0,sizeof(GpioLed));

    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

    GPiO_PeriClockCtrl(GPIOD, ENABLE);
    GPIO_Init(&GpioLed);

    GpioBtn.pGPIOx = GPIOD;
    GpioBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_5;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;

    GPiO_PeriClockCtrl(GPIOD, ENABLE);
    GPIO_Init(&GpioBtn);

    GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_RESET);
    // IRQ configurations
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

    while(1);
}

void EXTI9_5_IRQHandler(void)
{
	delay();
    GPIO_IRQHandling(GPIO_PIN_NO_5);
    GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
