/*
 * 007Spi_TxArduino.c
 *
 *  Created on: Nov 6, 2024
 *      Author: yywvi
 *      Unsucesss!
 */

#include "STM32f407VG.h"
#include <string.h>

void delay()
{
    for (uint32_t i = 0; i < 500000 / 2; i++)
    {
    }
}
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
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;
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
    SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
    SPI_Handle_t SPI2Handle;

    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // 2MHz
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // software slave managment enabled for NSS pin

    SPI_Init(&SPI2Handle);
}
void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GpioBtn;
    GpioBtn.pGPIOx = GPIOA;
    GpioBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_0;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

    GPIO_Init(&GpioBtn);
}
int main(void)
{

    char user_data[] = "Hello world";

    GPIO_ButtonInit();
    // Initialize the GPIO pins to behave as SPI2 pis
    SPI2_GPIOInits();

    // Initialize SPI2 peripheral parameters
    SPI2_Inits();

    // this function makes NSS signal internally high and avoids MODF error
    // SPI_SSIConfig(SPI2, ENABLE); //only for software management

    SPI_SSOEConfig(SPI2, ENABLE);
    while (1)
    {
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        // To avoid button de-bouncing related issues 200ms of delay
        delay();
        // enable SPI2 peripheral
        SPI_PeripheralControl(SPI2, ENABLE);

        // Send length information
        uint8_t datalen = strlen(user_data);
        SPI_SendData(SPI2, &datalen, 1);
        //  Send the data
        SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

        // Confirm the SPI is not busy
        while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)); // Disable the clock after communications
        SPI_PeripheralControl(SPI2, DISABLE);
    }

    return 0;
}
