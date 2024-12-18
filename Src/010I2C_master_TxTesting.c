/*
 * 010I2C_master_TxTesting.c
 *
 *  Created on: Nov 12, 2024
 *      Author: yywvi
 */


#include<stdio.h>
#include<string.h>
#include "STM32f407VG.h"

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;


//some data
uint8_t some_data[] = "we are testing I2C master Tx\n";
/*
 * PB6 --> SCL
 * PB9 --> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;// if acting master, it doesn't master. you should give if acting as slave
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2; //not using FM
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM; //use standard mode

	I2C_Init(&I2C1Handle);

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
	uint8_t sr = I2C_ACK_ENABLE;
	GPIO_ButtonInit();

	//i2c pin Inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//Enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	while(1)
	{
		//wait until button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//send data to salve
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDR,sr);

	}

}
