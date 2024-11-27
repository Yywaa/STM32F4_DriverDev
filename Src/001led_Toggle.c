/*
 * 001led_Toggle.c
 *
 *  Created on: Oct 31, 2024
 *      Author: Yongwei Yuan
 *      Success!
 */


#include "STM32f407VG.h"

void delay()
{
    for (uint32_t i = 0; i < 500000; i++);

}
int main(void)
{
/*
    GPIO_Handle_t GpioLed;

    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    // Open drain mode, Need breadboard and resistor, jump wire
    //GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    //GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

    // Push pull mode, don't need extra wire resistor. recommended
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;



    GPiO_PeriClockCtrl(GPIOD, ENABLE);
    GPIO_Init(&GpioLed);

    while (1)
    {
        //GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        //delay();
    }*/
}
