/*
 * STM32f407VG_gpio_driver.c
 *
 *  Created on: Oct 31, 2024
 *      Author: yywvi
 */

#include "STM32f407VG_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
/*******************************************************************
 * @fn               - GPIO_PeriClockCtrl
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
void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DIS();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DIS();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DIS();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DIS();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DIS();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DIS();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DIS();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DIS();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DIS();
        }
    }
}

// Peripheral Init and Deinit//
/*******************************************************************
 * @fn               - GPIO_Init
 *
 * @brief            - This function initialize for the given GPIO port
 *
 * @param            - Base address of the GPIOHandle
 * @param            -
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0; // temp register

    // enable the peripheral clock
    GPIO_PeriClockCtrl(pGPIOHandle->pGPIOx, ENABLE);
    // 1,Configure the mode of GPIO pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)

    {
        // not interrupt mode,
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum)); // clear bits
        pGPIOHandle->pGPIOx->MODER |= temp;                                                    // set bits
        temp = 0;
    }
    else
    {
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // 1,Configure the FTSR
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
            // clear the corresponding RTSR bit
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // 1, Configure the RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
            // clear the corresponding FTSR bit
            EXTI->FTSR &= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // 1,Configure both FTSR abd RTSR
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
        }
        // 2, Configure GPIO port selection in SYSCFG_EXTICR
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

        // 3, enable the exti interrupt delivery using IMR
        EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum;
    }
    temp = 0;

    // 2,Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum)); // clear bits
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;                                                    // set bits
    temp = 0;

    // 3,Configure the pupd settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum)); // clear bits
    pGPIOHandle->pGPIOx->PUPDR |= temp;                                                    // set bits
    temp = 0;

    // 4,Configure the optype
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNum)); // clear bits
    pGPIOHandle->pGPIOx->OTYPER |= temp;                                                // set bits

    // 5,Configure the alternate function
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        // Configure the alterate function mode
        uint8_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << 4 * temp2);                                             // clear bits
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // set bits
    }
}
/*******************************************************************
 * @fn               - GPIO_Deinit
 *
 * @brief            - his function Deinitialize for the given GPIO port
 *
 * @param            - Base address of the GPIO
 * @param            -
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
}

/*
 * Peripheral read and write
 */
/*******************************************************************
 * @fn               - GPIO_ReadFromInputPin
 *
 * @brief            - This function return 0 or 1 from the specific Pin number
 *
 * @param            - Baseaddress of the GPIO peripheral and the Pin number
 * @param            -
 * @param
 *
 * @return           - 0 or 1
 *
 * @note             - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNum) & 0x00000001);
    return value;
}

/*******************************************************************
 * @fn               - GPIO_ReadFromInputPort
 *
 * @brief            - This function returns the value of specific port
 *
 * @param            - Base address of the GPIO peripheral
 * @param            -
 * @param
 *
 * @return           - value of specific port
 *
 * @note             - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

/*******************************************************************
 * @fn               - GPIO_WriteToOutputPin
 *
 * @brief            - This function writes 1 or 0 to specific pinnumber
 *
 * @param            - Baseaddress of the GPIO peripheral
 * @param            - Pinnumber
 * @param            - Value
 *
 * @return           - none
 *
 * @note             - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        // Write 1 to the output data regsiter at the bit field corresponding to the pin number
        pGPIOx->ODR |= (1 << PinNum);
    }
    else
    {
        // write 0
        pGPIOx->ODR &= ~(1 << PinNum);
    }
}

/*******************************************************************
 * @fn               - GPIO_WriteToOutputPort

 *
 * @brief            - This function writes value to specific port
 *
 * @param            - Base address of the GPIO peripheral
 * @param            - Value
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/*******************************************************************
 * @fn               - GPIO_ToggleOutputPin
 *
 * @brief            - This function changes the bit corresponding to the pin number
 *
 * @param            - Base address of the GPIO peripheral
 * @param            - Pin number
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
    pGPIOx->ODR ^= (1 << PinNum);
}

/*
 * Peripheral IRQ Configuration and ISR handling
 */
/*******************************************************************
 * @fn               - GPIO_IRQInterruptConfig
 *
 * @brief            - This function configures the IRQ interrupt
 *
 * @param            - IRQ number
 * @param            - ENABLE or DISABLE macros
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi)
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
 * @fn               - void GPIO_IRQPriorityConfig
 *
 * @brief            - This function configure the priority of different IRQ number
 *
 * @param            - IRQ number
 * @param            - IRQ priority
 * @param
 *
 * @return           - none
 *
 * @note             - none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority)
{
    // 1,find the interrupt priority register number, IPR
    uint8_t iprx = IRQNum / 4;
    uint8_t iprxsection = IRQNum % 4;
    uint8_t shift_amount = (8 * iprxsection) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/*******************************************************************
 * @fn               - GPIO_IRQHandling
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
void GPIO_IRQHandling(uint8_t PinNum)
{
    // clear the exit pr register corresponding to the pin number
    if (EXTI->PR & (1 << PinNum))
    {
        // clear
        EXTI->PR |= (1 << PinNum);
    }
}
