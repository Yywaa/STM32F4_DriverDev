/*
 * STM32f407VG_SPI_driver.h
 *
 *  Created on: Nov 4, 2024
 *      Author: Yongwei Yuan
 */

#ifndef INC_STM32F407VG_SPI_DRIVER_H_
#define INC_STM32F407VG_SPI_DRIVER_H_
#include "STM32f407VG.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
    uint8_t SPI_DeviceMode; /*Device mode: master or slave,@SPI_DeviceMode*/
    uint8_t SPI_BusConfig;  /*Full duplex half duplex or simplex mode,@SPI_BusConfig*/
    uint8_t SPI_SclkSpeed;  /*Serial clock Speed,@SPI_SclkSpeed*/
    uint8_t SPI_DFF;        /*Data frame format: 8bits or 16 bits,@SPI_DIFF*/
    uint8_t SPI_CPHA;       /*CPHA (optional) 0 or 1,defalut: 0，@SPI_CPHA*/
    uint8_t SPI_CPOL;       /*CPOL (optional) 0 or 1,defalut: 0，@SPI_CPOL*/
    uint8_t SPI_SSM;        /*Slave selection management:software or hardware，@SPI_SSM */
} SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
    SPI_RegDef_t *pSPIx;    /*This holds the base address of SPIx(x:1,2,3)periphreal*/
    SPI_Config_t SPIConfig; /*Memory remap register,address offset: 0x00*/
    uint8_t *pTxBuffer;    	/*To store the Txbuffer address*/
	uint8_t *pRxBuffer;		/*To store the Rxbuffer address*/
	uint8_t TxLen;			/*To store Tx len*/
	uint8_t RxLen;			/*To store Rx len*/
	uint8_t TxState;		/*To store Tx state*/
	uint8_t RxState;		/*To store Rx state*/
} SPI_Handle_t;


/*
 *@SPI_State Macros
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX  2

/*
 * Possible SPI Apllication events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4


/*
 *@SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER 1 /* Only master mode can produce clock*/
#define SPI_DEVICE_MODE_SLAVE 0

/*
 *@SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD 1 /* Full duplex*/
#define SPI_BUS_CONFIG_HD 2 /* Half duplex*/
// #define SPI_BUS_CONFIG_SIMPLEX_TXONLY 2 /* Simplex Transmit only, remove RX line,in fact it's full duplex*/
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3 /* Simplex Receive only*/

/*
 *@SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2 0   /* fPCLK/2*/
#define SPI_SCLK_SPEED_DIV4 1   /* fPCLK/4*/
#define SPI_SCLK_SPEED_DIV8 2   /* fPCLK/8*/
#define SPI_SCLK_SPEED_DIV16 3  /* fPCLK/16*/
#define SPI_SCLK_SPEED_DIV32 4  /* fPCLK/32*/
#define SPI_SCLK_SPEED_DIV64 5  /* fPCLK/64*/
#define SPI_SCLK_SPEED_DIV128 6 /* fPCLK/128*/
#define SPI_SCLK_SPEED_DIV256 7 /* fPCLK/256*/

/*
 *@SPI_DIFF
 */
#define SPI_DFF_8BITS 0 /*Default value*/
#define SPI_DFF_16BITS 1

/*
 *@SPI_CPOL
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/*
 *@SPI_CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
 *@SPI_SSM
 */
#define SPI_SSM_EN 1 /*Enable software management*/
#define SPI_SSM_DI 0 /*Software management disabled by default, 0*/

/*
 *@SPI status flags definitions in SR, status register
 */
#define SPI_TXE_FLAG (1 << SPI_SR_TXE_OFFSET)
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE_OFFSET)
#define SPI_BUSY_FLAG (1 << SPI_SR_BSY_OFFSET)

/*********************************************************
 *
 *         APIs supported by this driver
 * Check function definitions for more informations
 *
 ********************************************************/
/*
 *Peripheral Clock setup
 */
void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Peripheral Init and Deinit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Deinit(SPI_Handle_t *pSPIx);

/*
 * Data send or receive, this is blocking based- non interrupt
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*
 * Data send or receive, this is blocking based- non interrupt
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data send or receive, this is non-blocking based- interrupt based
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Peripheral IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
/*
 * Application callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);
#endif /* INC_STM32F407VG_SPI_DRIVER_H_ */
