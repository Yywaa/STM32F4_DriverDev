/*
 * 010I2C_master_TxTesting.c
 *
 *  Created on: Nov 12, 2024
 *      Author: yywvi
 */


#include<stdio.h>
#include<string.h>
#include "STM32f407VG.h"

extern void initialise_monitor_handles();

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;


//Receive buffer
uint8_t rcv_buf[32];
uint8_t RxCMPLT = RESET;
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
	uint8_t commandcode;
	uint8_t Len;

	initialise_monitor_handles();
	printf("Application is running\n");
	GPIO_ButtonInit();

	//i2c pin Inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	//Enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE = 1;
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		//wait until button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = 0x51;

		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR) !=I2C_READY);
		while(I2C_MasterReceiveDataIT(&I2C1Handle, &Len, 1,SLAVE_ADDR,I2C_ENABLE_SR)!=I2C_READY);

		commandcode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR) !=I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, Len, SLAVE_ADDR,I2C_DISABLE_SR)!=I2C_READY);

		RxCMPLT = RESET;
		//wait till Rx completed
		while(RxCMPLT != SET);
		rcv_buf[Len+1] = '\0';

		printf("Data: %s:",rcv_buf);
		RxCMPLT = RESET;
	}
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
	if(AppEv == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed \n");
	}else if(AppEv == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed\n");
		RxCMPLT = SET;
	}else if(AppEv == I2C_ERROR_AF)
	{
		printf("Error: Ack failure");
		//Master ack failure happens when slave fails to send ack for the byte
		//sent from the master
		I2C_CloseSendData(pI2CHandle);

		//Generate stop condition to release the bus
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//hang in infinite loop
		//while(1);//because if first master send data failed, there is no meaning to continue to receive
	}
}

