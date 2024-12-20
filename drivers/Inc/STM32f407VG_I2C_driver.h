/*
 * STM32f407VG_I2C_driver.h
 *
 *  Created on: Nov 11, 2024
 *      Author: yywvi
 */

#ifndef INC_STM32F407VG_I2C_DRIVER_H_
#define INC_STM32F407VG_I2C_DRIVER_H_
#include "STM32f407VG.h"


/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;				/*Serial clock speed,possible value from @I2C_SCLSpeed*/
	uint8_t I2C_DeviceAddress;			/*If the device is slave, user has to mention its address,We don't have option to initialize this*/
	uint8_t I2C_ACKControl;				/*automatically ACK is disable by default, so,user can decide enable or disable,possible value from @I2C_ACKControl*/
	uint8_t I2C_FMDutyCycle;			/*clock duty cycle can be varied in fast mode,possible value from @I2C_FMDutyCycle*/
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;			/*To store the applicaton Tx buffer address*/
	uint8_t *pRxBuffer;			/*T o store the applicaton Rx buffer address*/
	uint32_t TxLen;				/* To store Tx len*/
	uint32_t RxLen;				/*To store Rx Len*/
	uint8_t  TxRxState;			/*To store communicaiton state, because,I2C,is half duplex, SPI full,so I2C,only has one variable*/
	uint8_t  DevAddr;			/*To store slave/device address*/
	uint8_t  RxSize;			/*To store Rx address*/
	uint8_t  Sr;				/*To store repeated start valve, SR: repeated start*/
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K		200000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*
 * @I2C application states
 */
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

/*
 *@I2C status flags definitions in SR, status register
 */
#define I2C_FLAG_TXE		(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FALG_OVR		(1 << I2C_SR1_OVR)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR		RESET  //SR: repeated start
#define I2C_ENABLE_SR		SET


/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR		3			/*I2C bus error*/
#define I2C_ERROR_ARLO		4			/*Arbition loss (master)*/
#define I2C_ERROR_AF		5			/*Acknowledge failure*/
#define I2C_ERROR_OVR		6			/*Overrun/under-run*/
#define I2C_ERROR_TIMEOUT	7			/*Timeout*/
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9
/*********************************************************
 *
 *         APIs supported by this driver
 * Check function definitions for more informations
 *
 ********************************************************/
/*
 *Peripheral Clock setup
 */
void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Peripheral Init and Deinit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2Cx);

/*
 * Data send or receive-this is blocking based- non interrupt
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t ,uint8_t sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr);
/*
 * Data send or receive, this is  interrupt based, these two API return state of application, so return type is uint8_t
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t ,uint8_t sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);
/*
 * Peripheral IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other Peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);

/*
 * Application callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);// definition in user application





#endif /* INC_STM32F407VG_I2C_DRIVER_H_ */
