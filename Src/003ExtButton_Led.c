/*
 * 003ExtButton_Led.c
 *
 *  Created on: Nov 1, 2024
 *      Author: yywvi
 *      Success!!
 */


#include "STM32f407VG.h"

#define HIGH 1
#define LOW  0
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

    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_8;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

    GPiO_PeriClockCtrl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed);

    GpioBtn.pGPIOx = GPIOB;
    GpioBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_12;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;

    GPiO_PeriClockCtrl(GPIOB, ENABLE);
    GPIO_Init(&GpioBtn);

    while (1)
    {
        if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED)
        {
            delay(); // used to avoid de-bounce
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
        }
        //delay();

    }
    return 0;
}
