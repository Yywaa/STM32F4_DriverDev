/*
 * STM32f407VG_gpio_driver.h
 *
 *  Created on: Oct 31, 2024
 *      Author: yywvi
 */

#ifndef INC_STM32F407VG_GPIO_DRIVER_H_
#define INC_STM32F407VG_GPIO_DRIVER_H_
#include "STM32f407VG.h"

/*
 * Configuration structure for a GPIO pin
 */
typedef struct
{
    uint8_t GPIO_PinNum;      /*Possible values from @GPIO_PIN_Nums*/
    uint8_t GPIO_PinMode;     /*Possible values from @GPIO_PIN_MODES*/
    uint8_t GPIO_PinSpeed;    /*Possible values from @GPIO_PIN_SPEED*/
    uint8_t GPIO_PinPuPdCtrl; /*Possible values from @GPIO_Pin_PUPD*/
    uint8_t GPIO_PinOPType;   /*Possible values from @GPIO_OP_TYPE*/
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;           /*This holds the base address of GPIO port to which the pin belongs*/
    GPIO_PinConfig_t GPIO_PinConfig; /*This holds GPIO pin configuration settings*/
} GPIO_Handle_t;

/*
 *@ GPIO_PIN_Nums
 * GPIO pin possible modes
 */
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15
/*
 *@ GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 0     /*GPIO input mode*/
#define GPIO_MODE_OUT 1    /*GPIO output mode*/
#define GPIO_MODE_ALTFN 2  /*GPIO alternate function mode*/
#define GPIO_MODE_ANALOG 3 /*GPIO analog mode*/
#define GPIO_MODE_IT_FT 4  /*GPIO interrupt mode falling trigger*/
#define GPIO_MODE_IT_RT 5  /*GPIO interrupt mode rising trigger*/
#define GPIO_MODE_IT_RFT 6 /*GPIO interrupt mode rising and falling trigger*/

/*
 *@GPIO_OP_TYPE
 * GPIO pin possible output type
 */
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

/*
 *@GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MDEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

/*
 *@GPIO_Pin_PUPD
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

/*********************************************************
 *
 *         APIs supported by this driver
 * Check function definitions for more informations
 *
 ********************************************************/
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Peripheral Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pSPIx); // 有些疑问？？？

/*
 * Peripheral read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

/*
 * Peripheral IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNum);

#endif /* INC_STM32F407VG_GPIO_DRIVER_H_ */
