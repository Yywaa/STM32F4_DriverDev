/*
 * STM32f407VG_SPI_driver.c
 *
 *  Created on: Nov 4, 2024
 *      Author: yywvi
 */
#include "STM32f407VG_SPI_driver.h"


static void spi_handle_txe(SPI_Handle_t *pHandle); //static keeps the function only in this file
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_over_err_interrupt_handle(SPI_Handle_t *pHandle);
/*******************************************************************
 * @fn               - SPI_PeriClockCtrl
 *
 * @brief            - This function enables or disables peripheral clock for the given SPIx
 *
 * @param            - Baseaddress of SPIx
 * @param            - ENABLE or DISABLE macros
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }
}

/*
 * Peripheral Init and Deinit
 */
/*******************************************************************
 * @fn               - SPI_Init
 *
 * @brief            - This function initializes configuration of SPIx
 *
 * @param            - Baseaddress of SPIx Handle
 * @param            -
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    /*configure the SPI_CR1 register*/
    uint32_t tempreg = 0;

    // Enable the peripheral clock
    SPI_PeriClockCtrl(pSPIHandle->pSPIx, ENABLE);

    // 1,configure the device mode
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_OFFSET;

    // 2,Configure the bus configure
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // Bidi mode should be cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE_OFFSET);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // Bidi mode should be set
        tempreg |= (1 << SPI_CR1_BIDIMODE_OFFSET);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // Bidi mode should cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE_OFFSET);
        // RXONLY bit must be set
        tempreg |= (1 << SPI_CR1_RXONLY_OFFSET);
    }
    // Configure the spi serial clock speed (baud rate)
    tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_OFFSET;

    // Configure the DFF
    tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF_OFFSET;

    // Configure the CPOL
    tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_OFFSET;

    // Configure the CPHA
    tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_OFFSET;

    // Configure the SSM
    tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM_OFFSET;

    pSPIHandle->pSPIx->CR1 = tempreg;
}
/*******************************************************************
 * @fn               - SPI_Deinit
 *
 * @brief            - This function De-initializes configuration of SPIx
 *
 * @param            - Base address of SPIx
 * @param            -
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_Deinit(SPI_Handle_t *pSPIx)
{
	if(pSPIx->pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	if(pSPIx->pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	if(pSPIx->pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	if(pSPIx->pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/*******************************************************************
 * @fn               - SPI_GetFlagStatus
 *
 * @brief            - Check the SPI communication status in SR,status register
 *
 * @param            - Base address of SPIx
 * @param            -
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if (pSPIx->SR & FlagName) // 这个函数是不是有风险？？？
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*
 * Data send or receive, this is blocking based (polling based)- non interrupt
 */
/*******************************************************************
 * @fn               - SPI_SendData
 *
 * @brief            - This function sends data by Txbuffer
 *
 * @param            - Base address of SPIx Handle
 * @param            - Data to be sent
 * @param            - Data length
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1,Wait until TXE is set
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET); // We are polling for TXE flag to set.

        // 2,check DFF bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF_OFFSET)) // 这样直接运算？感觉不对！
        {
            // 16 bit DFF
            // 1,Load the data into DR, data register
            pSPIx->DR = *((uint16_t *)pTxBuffer); // take 2 Bytes data, *((uint32_t *)pTxBuffer):take 4 Bytes data
            Len--;
            Len--;
            (uint16_t *)pTxBuffer++; // Pointer increase according to its type, increase 2 bytes if uint16
        }
        else
        {
            // 8 bit DFF,
            pSPIx->DR = *pTxBuffer; // take 1Byte data
            Len--;
            pTxBuffer++; // Pointer increase according to its type, increase 1 byte if uint8
        }
    }
}

/*******************************************************************
 * @fn               - SPI_ReceiveData
 *
 * @brief            - This function receives data by Rxbuffer
 *
 * @param            - Base address of SPIx handle
 * @param            - Data to be read
 * @param            - Data length
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1,Wait until RXNE is set
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET) // We are polling for TXE flag to set.
            ;
        // 2,check DFF bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF_OFFSET)) // 这样直接运算？感觉不对！
        {
            // 16 bit DFF
            // 1,Load the data into DR, data register
            *((uint16_t *)pRxBuffer) = pSPIx->DR; // take 2 Bytes data, *((uint32_t *)pTxBuffer):take 4 Bytes data
            Len--;
            Len--;
            (uint16_t *)pRxBuffer++; // Pointer increase according to its type, increase 2 bytes if uint16
        }
        else
        {
            // 8 bit DFF,
            *pRxBuffer = pSPIx->DR; // take 1Byte data
            Len--;
            pRxBuffer++; // Pointer increase according to its type, increase 1 byte if uint8
        }
    }
}

/*
 * Peripheral IRQ Configuration and ISR handling
 */
/*******************************************************************
 * @fn               - SPI_IRQInterruptConfig
 *
 * @brief            - This function enables or disables peripheral clock for the given SPIx
 *
 * @param            - IRQ number
 * @param            - ENABLE or DISABLE macros
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi)
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
/*******************************************************************
 * @fn               - SPI_IRQPriorityConfig
 *
 * @brief            - This function enables or disables peripheral clock for the given SPIx
 *
 * @param            - IRQ number
 * @param            - IRQ priority
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority)
{
	// 1,find the interrupt priority register number, IPR
	uint8_t iprx = IRQNum / 4;
	uint8_t iprxsection = IRQNum % 4;
	uint8_t shift_amount = (8 * iprxsection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}



/*******************************************************************
 * @fn               - SPI_SendDataIT
 *
 * @brief            - This function sends data by Txbuffer, interrupt mode.
 *
 * @param            - Base address of SPIx Handle
 * @param            - Data to be sent
 * @param            - Data length
 *
 * @return           - none
 *
 * @note             - none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
	    //Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;\
		pSPIHandle->TxLen = Len;
		//2,mark the SPI state as busy om transimission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3, Enable the TXEIE control bi tto get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE_OFFSET);

	}
	return state;

}
/*******************************************************************
 * @fn               - SPI_SendData
 *
 * @brief            - This function sends data by Txbuffer, interrupt mode
 *
 * @param            - Base address of SPIx Handle
 * @param            - Data to be sent
 * @param            - Data length
 *
 * @return           - none
 *
 * @note             - none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;\
		pSPIHandle->RxLen = Len;
		//2,mark the SPI state as busy om transimission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3, Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE_OFFSET);

	}
	return state;
}


/*******************************************************************
 * @fn               - SPI_IRQHandling
 *
 * @brief            - This function sends data by Txbuffer, interrupt mode
 *
 * @param            - Base address of SPIx Handle
 * @param            - Data to be sent
 * @param            - Data length
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//first check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE_OFFSET);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE_OFFSET);

	if(temp1 && temp2)
	{
		//Handle TXE
		spi_handle_txe(pHandle);
	}

	//first check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE_OFFSET);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE_OFFSET);

	if(temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	//first check for overrun flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR_OFFSET);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE_OFFSET);

	if(temp1 && temp2)
	{
		//handle over run error
		spi_over_err_interrupt_handle(pHandle);
	}
}

//Helper function

static void spi_handle_txe(SPI_Handle_t *pHandle)
{
	// 2,check DFF bit in CR1
	if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF_OFFSET)) // 这样直接运算？感觉不对！
	{
		// 16 bit DFF
		// 1,Load the data into DR, data register
		pHandle->pSPIx->DR = *((uint16_t *)pHandle->pTxBuffer); // take 2 Bytes data, *((uint32_t *)pTxBuffer):take 4 Bytes data
		pHandle->TxLen--;
		pHandle->TxLen--;
		(uint16_t *)pHandle->pTxBuffer++; // Pointer increase according to its type, increase 2 bytes if uint16
	}
	else
	{
		// 8bit DFF
		pHandle->pSPIx->DR = *pHandle->pTxBuffer; // take 2 Bytes data, *((uint32_t *)pTxBuffer):take 4 Bytes data
		pHandle->TxLen--;
		(uint8_t *)pHandle->pTxBuffer++; // Pointer increase according to its type, increase 2 bytes if uint16
	}
	if(!pHandle->TxLen)
	{
		//TxLen is zero , close the SPI communication and inform the application that
		// Tx is over
		// this prevents interrupt from setting up of TXE flag
		//pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE_OFFSET);
		//pHandle->pTxBuffer = NULL;
		//pHandle->TxLen = 0;
		//pHandle->TxState = SPI_READY;

		SPI_CloseTransmission(pHandle);

		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);

	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle)
{
// 2,check DFF bit in CR1
	if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF_OFFSET)) // 这样直接运算？感觉不对！
	{
		// 16 bit DFF
		// 1,Load the data into DR, data register
		*((uint16_t *)pHandle->pRxBuffer) = (uint16_t)pHandle->pSPIx->DR; // take 2 Bytes data, *((uint32_t *)pTxBuffer):take 4 Bytes data
		pHandle->RxLen--;
		pHandle->RxLen--;
		(uint16_t *)pHandle->pRxBuffer++; // Pointer increase according to its type, increase 2 bytes if uint16
	}
	else
	{
		// 8 bit DFF,
		*(pHandle->pRxBuffer) = (uint8_t)pHandle->pSPIx->DR; // take 1Byte data
		pHandle->RxLen--;
		//WHy ?? --??while should be ++??
		pHandle->pRxBuffer--; // Pointer increase according to its type, increase 1 byte if uint8
	}
	if(!pHandle->RxLen)
	{
		//TxLen is zero , close the SPI communication and inform the application that
		// Tx is over
		// this prevents interrupt from setting up of TXE flag
		//pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXEIE_OFFSET);
		//pHandle->pRxBuffer = NULL;
		//pHandle->RxLen = 0;
		//pHandle->RxState = SPI_READY;

		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
	}


}
static void spi_over_err_interrupt_handle(SPI_Handle_t *pHandle)
{
	uint8_t temp;
	//clear the Ovr flag
	if(pHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;
	}
	(void)temp;
	//inform the application
	SPI_ApplicationEventCallback(pHandle,SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE_OFFSET);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE_OFFSET);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	//clear the Ovr flag

	temp = pSPIx->DR; // this is variable initialization, not using
	temp = pSPIx->SR;
	(void)temp;

}

_weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// this is a weak implementation, the application may override this function
	//this function should be implemented in application (user)

}


/*******************************************************************
 * @fn               - SPI_IRQHandling
 *
 * @brief            - This function enables or disables peripheral clock for the given SPIx
 *
 * @param            - SPIx handling
 * @param            - ENABLE or DISABLE macros
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*******************************************************************
 * @fn               - SPI_PeripheralControl
 *
 * @brief            - This function enables or disables SPIx
 *
 * @param            - SPI_RegDef_t
 * @param            - ENABLE or DISABLE macros
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE_OFFSET);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE_OFFSET);
    }
}

/*******************************************************************
 * @fn               - SPI_PeripheralControl
 *
 * @brief            - This function enables or disables SPIx
 *
 * @param            - SPI_RegDef_t
 * @param            - ENABLE or DISABLE macros
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI_OFFSET);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI_OFFSET);
    }
}

/*******************************************************************
 * @fn               - SPI_SSOEConfig
 *
 * @brief            - This function enables or disables SPIx
 *
 * @param            - SPI_RegDef_t
 * @param            - ENABLE or DISABLE macros
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR2_SSOE_OFFSET);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR2_SSOE_OFFSET);
    }
}
