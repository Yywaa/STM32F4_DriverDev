/*
 * STM32f407VG_I2C_driver.c
 *
 *  Created on: Nov 11, 2024
 *      Author: yywvi
 */

#include "STM32f407VG_I2C_driver.h"



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhraseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhraseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhraseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // Slaveaddr is slave address + r/w bit =0
	pI2Cx->DR = SlaveAddr;

}
static void I2C_ExecuteAddressPhraseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= 1; // Slaveaddr is slave address + r/w bit =0
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//Check for device mode
	if(pI2CHandle->pI2Cx->SR2 &(1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear the ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;

			}

		}else
		{
			//clear the ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}

	}else
	{
		//Device is in slave mode
		//clear the ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}


void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}else
	{
		pI2Cx->CR2 |= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= ~(1 << I2C_CR2_ITERREN);
	}
}
/*******************************************************************
 * @fn               - I2C_PeripheralControl
 *
 * @brief            - This function enables or disables peripheral for the given I2C
 *
 * @param            - Base address of the GPIO peripheral
 * @param            - ENABLE or DISABLE macros
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi== ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);

	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*******************************************************************
 * @fn               - I2C_PeriClockCtrl
 *
 * @brief            - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param            - Baseaddress of the GPIO peripheral
 * @param            - ENABLE or DISABLE macros
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}





/*******************************************************************
 * @fn               - I2C_Init
 *
 * @brief            - This function initializes I2C basic configuration
 *
 * @param            - Based address of I2C_Handle
 * @param            -
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//Enable the clock for the i2c peripheral
	I2C_PeriClockCtrl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//Configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device address,if your device acting as slave
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1= tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value()/(2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value &0xFFF);
	}else
	{
		// mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value()/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value()/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);


	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		//uint8_t trise;
		tempreg = (RCC_GetPCLK1Value()/1000000U)+1;

	}else
	{
		//mode is fast mode, 300ns, UM10204 pdf, for fast mode,Trise max is 300ns
		tempreg = ((RCC_GetPCLK1Value()* 300)/1000000000U) + 1;

	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}
/*******************************************************************
 * @fn               - I2C_Deinit
 *
 * @brief            - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param            - Base address of I2C_Handle
 * @param            -
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void I2C_DeInit(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	if(pI2CHandle->pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	if(pI2CHandle->pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}

}




/*******************************************************************
 * @fn               - SPI_GetFlagStatus
 *
 * @brief            - This function send data from master to slave
 *
 * @param            - Base address of I2C_Handle
 * @param            -
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
    if (pI2Cx->SR1 & FlagName) // 这个函数是不是有风险？？？
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}
/*******************************************************************
 * @fn               - I2C_MasterSendData
 *
 * @brief            - This function send data from master to slave
 *
 * @param            - Base address of I2C_Handle
 * @param            -
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr)
{
	//1,generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2, confirm start generation is completed by checking the SB flag in the SR1
	//note:until SB is cleared SCL will be stretched (pulled to low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3, Send the address of the slave with 1/w bot set to w(0)(total 8 bit)
	I2C_ExecuteAddressPhraseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4,confirm that address phrase is completed by checking ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5,clear the ADDR flag according to its software sequence
	//Note until ADDR is cleared SCL will be stretched (pull to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6,send the data until len becomes 0
	while(Len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));//wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7,when Len becomes zero wait for TXE = 1 and BTF = 1 before generating STOP condition
	//Note: TXE = 1, BTF = 1,means both SR and DR are empty and next transmission should begin
	//when BTF = 1,SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));


	//8,Generate STOP condition and master need not to wait for the completion of stop conditon
	//Note: generating STOP, automatically clears the
    if( sr == I2C_DISABLE_SR)
    {
    	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }


}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr)
{
	//1,generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2, confirm start generation is completed by checking the SB flag in the SR1
	//note:until SB is cleared SCL will be stretched (pulled to low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3, Send the address of the slave with 1/w bot set to r(1)(total 8 bit)
	I2C_ExecuteAddressPhraseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4,confirm that address phrase is completed by checking ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//Procedure to read only one byte
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//Generate STOP condition
		if( sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}


		//read the data into buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Read the data until Len becomes zero
		for(uint32_t i = Len; i >0 ; i--)
		{
			//wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) // if last 2 bytes are remaining
			{
				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if( sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

			}
			//read the data from data register into buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxbuffer++;
		}

	}
	//Re- enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}

}



void I2C_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNum <= 31)
		{
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNum);
		}
		else if (IRQNum > 31 && IRQNum < 64)
		{
			// Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNum % 32));
		}
		else if (IRQNum >= 64 && IRQNum < 96)
		{
			// Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNum % 64));
		}
	}
	else
	{
		if (IRQNum <= 31)
		{
			// Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNum);
		}
		else if (IRQNum > 31 && IRQNum < 64)
		{
			// Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNum % 32));
		}
		else if (IRQNum >= 64 && IRQNum < 96)
		{
			// Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNum % 64));
		}
	}
}
void I2C_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority)
{
	// 1,find the interrupt priority register number, IPR
	uint8_t iprx = IRQNum / 4;
	uint8_t iprxsection = IRQNum % 4;
	uint8_t shift_amount = (8 * iprxsection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}





uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;

}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen >0 )
	{
		//1, load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2, decrement the Txlen
		pI2CHandle->TxLen--;

		//3, Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}

}


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
	}
	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2) // if last 2 bytes are remaining
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		}
		//read the data from data register into buffer
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		//increment the buffer address
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0)
	{
		//close I2C data reception and notify the application
		//1, generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//2,close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}

}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITENFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}
void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->DR = data;

}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}





void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2,temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 &(1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 &(1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 &(1 << I2C_SR1_SB);

	//1, Handle for interrupt generated by SB event
	//Note: SB flag is only applicable in master mode
	if(temp1 && temp3)
	{
		// The interrupt is generated because of SBevent
		// This block will not be executed in slave mode because for slave SB is always 0;
		//in this phrase, execute address phrase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhraseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else
		{
			I2C_ExecuteAddressPhraseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}

	}
	//2,Handle for interrupt generated by ADDR event
	//Note: When master mode : address is sent
	//		When slave mode: address is matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 &(1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		//ADDR flag is set
		//clear the flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3, Handle for interrupt generated by BTF(Byte transfer finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 &(1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 &(1 << I2C_SR1_TXE))
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0)
				{
					//1,Generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2, reset all  the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//3,notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			// DR is full, SR is full in RX mode, nothing can be done
			;
		}
	}

	//4,Handle for interrupt generated by STOPF event
	//Note: Stop detection flag is applicable only slave mode, for master this flag will never be set
	//Below code will not be executed by the master since STOPF will not set in master mode
	temp3 = pI2CHandle->pI2Cx->SR1 &(1 << I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//STOPF flag is set
		//Clear the STOPF, 1, read SR1,2,Write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000; //write this will not affect CR1 register
		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}

	//5,Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 &(1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode, only in master mode, below code will be executed because this is for master send or receive IT function
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			// we have to do data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}

		}else
		{
			//slave, make sure that slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 &(1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	//6,Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 &(1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{

		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave, make sure that slave is really in RECV mode
			if(!(pI2CHandle->pI2Cx->SR2 &(1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);// arbitration error

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

		//Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

		//Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

		//Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_ERROR_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}




