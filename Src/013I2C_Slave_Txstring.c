/*
 * 010I2C_master_TxTesting.c
 *
 *  Created on: Nov 12, 2024
 *      Author: yywvi
 */


#include<stdio.h>
#include<string.h>
#include "STM32f407VG.h"



#define SLAVE_ADDR 0x68
#define MY_ADDR    SLAVE_ADDR
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;


//Receive buffer
uint8_t Tx_buf[32] = "STM32 Slave mode testing..";
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

	GPIO_ButtonInit();

	//i2c pin Inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);

	//Enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE = 1;
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);

}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);

}
void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	static uint8_t commandcode;
	static uint8_t Cnt = 0;
	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants some data, slave has to send it
		if(commandcode == 0x51)
		{
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buf));
		}else if(commandcode ==0x52)
		{
			// Sent the content of Tx_buff
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[Cnt++]);
		}

	}else if(AppEv == I2C_EV_DATA_RCV)
	{
		//Data is waiting for slave to read, slave has to read it
		commandcode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}else if(AppEv == I2C_ERROR_AF)
	{
		//This happens only during slave txing
		//Master has sent NACK, SO slave should understand master doesn't need more data
        commandcode == 0xff;
        Cnt = 0;

	}else if(AppEv == I2C_EV_STOP)
	{
		//This happens only during slave reception
		//Master has ended the I2C communication with salve
	}

}

