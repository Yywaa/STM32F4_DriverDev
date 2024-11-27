/*
 * 006SPI_Test.c
 *
 *  Created on: Nov 5, 2024
 *      Author: yywvi
 */
#include "STM32f407VG.h"
#include <string.h>
/*
 *PB12 ---> SPI2_NSS
 *PB13 ---> SPI2_Sclk
 *PB14 ---> SPI2_MISO
 *PB15 ---> SPI2_MOSI
 * Alternate function mode: AF5
 */

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // MISO
    // SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_14;
    // GPIO_Init(&SPIPins);

    // NSS
    // SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_12;
    // GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
    SPI_Handle_t SPI2Handle;

    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // 8MHz
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // software slave managment enabled for NSS pin

    SPI_Init(&SPI2Handle);
}
int main(void)
{
	/*
    char user_data[] = "Hello world";
    // Initialize the GPIO pins to behave as SPI2 pis
    SPI2_GPIOInits();

    // Initialize SPI2 peripheral parameters
    SPI2_Inits();

    // this function makes NSS signal internally high and avoids MODF error
       SPI_SSIConfig(SPI2, ENABLE);

    // enable SPI2 peripheral
    SPI_PeripheralControl(SPI2, ENABLE);

    // Send the data
    SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

    //Disable the clock after communications
    SPI_PeripheralControl(SPI2, DISABLE);

    while (1);*/

    return 0;
}
