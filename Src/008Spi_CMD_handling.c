/*
 * 008Spi_CMD_handling.c
 *
 *  Created on: Nov 6, 2024
 *      Author: yywvi
 */

#include "STM32f407VG.h"
#include <string.h>

//command codes
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ		    0x54

#define LED_ON  1
#define LED_OFF 0

//Arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

//Arduino Led
#define LED_PIN			9

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
    SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

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
uint8_t SPI_verifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	return 0;

}
int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

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

        //1,CMD_LED_CTRL <pin no(1),value(1)>
        uint8_t commndcode = COMMAND_LED_CTRL;
        uint8_t ackbyte;
        uint8_t args[2];
        SPI_SendData(SPI2, &commndcode, 1);

        //do dummy read to clear off the RXNE
        SPI_ReceiveData(SPI2, &dummy_read, 1);
        //send some dummy bits (1byte) to fetch response from slave
        SPI_SendData(SPI2, &dummy_write, 1);

        //Read the ack byte received
        SPI_ReceiveData(SPI2, &ackbyte, 1);

        if(SPI_verifyResponse(ackbyte))
        {
        	//send arguments
        	args[0] = LED_PIN;
        	args[1] = LED_ON;
        	//send arguments
        	SPI_SendData(SPI2, args, 2); //sending one byte

        }
        //2, CMD_SENSOR_LED_CTRL
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        // To avoid button de-bouncing related issues 200ms of delay
        delay();
        commndcode = COMMAND_SENSOR_READ;

        //send command
        SPI_SendData(SPI2, &commndcode, 1);


        //do dummy read to clear off the RXNE
        SPI_ReceiveData(SPI2, &dummy_read, 1);
        //send some dummy bits (1byte) to fetch response from slave
        SPI_SendData(SPI2, &dummy_write, 1);

        //Read the ack byte received
        SPI_ReceiveData(SPI2, &ackbyte, 1);

        if(SPI_verifyResponse(ackbyte))
        {
        	//send arguments
        	args[0] = ANALOG_PIN0;
        	//args[1] = LED_ON;
        	//send arguments
        	SPI_SendData(SPI2, args, 1);
        	//do dummy read to clear off the RXNE
        	SPI_ReceiveData(SPI2, &dummy_read, 1);

        	//insert some delay
        	delay();
            //send some dummy bits (1byte) to fetch response from slave
        	SPI_SendData(SPI2, &dummy_write, 1);
        	uint8_t analog_read;
        	SPI_ReceiveData(SPI2, &analog_read, 1);

        }



        // Confirm the SPI is not busy
        while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)); // Disable the clock after communications
        SPI_PeripheralControl(SPI2, DISABLE);
    }

    return 0;
}
