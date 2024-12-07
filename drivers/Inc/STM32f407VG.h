/*
 * STM32f407VG.h
 *
 *  Created on: Oct 30, 2024
 *      Author: yywvi
 */

#ifndef INC_STM32F407VG_H_
#define INC_STM32F407VG_H_

#include "stdint.h"
#include <stddef.h>
#define _VO volatile
#define _weak __attribute__((weak))
/***************************************************************************
 * ARM Cortex Mx Processor NVIC ISERx register Address
 *
 */
#define NVIC_ISER0 ((_VO uint32_t *)0xE000E100) // ISER: Interrupt Set-enable register
#define NVIC_ISER1 ((_VO uint32_t *)0xE000E104)
#define NVIC_ISER2 ((_VO uint32_t *)0xE000E108)
#define NVIC_ISER3 ((_VO uint32_t *)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Address
 */
#define NVIC_ICER0 ((_VO uint32_t *)0xE000E180) // ICER:Interrupt Clear-enable Register
#define NVIC_ICER1 ((_VO uint32_t *)0xE000E184)
#define NVIC_ICER2 ((_VO uint32_t *)0xE000E188)
#define NVIC_ICER3 ((_VO uint32_t *)0xE000E19C)

/*
 * ARM Cortex Mx Processor priority register address
 */
#define NVIC_PR_BASE_ADDR ((_VO uint32_t *)0xE000E400) // Priority regitser

#define NO_PR_BITS_IMPLEMENTED 4 // priority register each seciton 8bits, but high 4 bits implemented

/*
 * base address of Flash memory and SRAM memories
 */
#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U // 112kb
#define SRAM2_BASEADDR 0x2001C000U
#define ROM_BASEADDR 0x1FFF0000U
#define SRAM SRAM1_BASEADDR

/*
 * AHBx and APBx bus peripheral base address
 */
#define PERIPH_BASE 0x40000000U
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE 0x40010000U
#define AHB1PERIPH_BASE 0x40020000U
#define AHB2PERIPH_BASE 0x50000000U

/*
 * Base Addresses of peripherals which hang on AHB1 bus
 */
#define GPIOA_BASEADDR (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASE + 0x2000)
#define RCC_BASEADDR (AHB1PERIPH_BASE + 0x3800)

/*
 * Base Addresses of peripherals which hang on APB1 bus
 */
#define I2C1_BASEADDR (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0x5000)

/*
 * Base Addresses of peripherals which hang on APB2 bus
 */

#define SPI1_BASEADDR (APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR (APB2PERIPH_BASE + 0x3400)
#define EXTI_BASEADDR (APB2PERIPH_BASE + 0x3C00)
#define SPI1_BASEADDR (APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASE + 0x1400)
#define SYSCFG_BASEADDR (APB2PERIPH_BASE + 0x3800)

/***************peripherals register definition structures***********/

/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
    _VO uint32_t MODER;   /*GPIO port mode register,address offset: 0x00*/
    _VO uint32_t OTYPER;  /*GPIO output type register,address offset: 0x04*/
    _VO uint32_t OSPEEDR; /*GPIO output speed register,address offset: 0x08*/
    _VO uint32_t PUPDR;   /*GPIO pull up/down register,address offset: 0x0C*/
    _VO uint32_t IDR;     /*GPIO input data register,address offset: 0x10*/
    _VO uint32_t ODR;     /*GPIO output data register,address offset: 0x14*/
    _VO uint32_t BSRR;    /*GPIO set/reset register,address offset: 0x18*/
    _VO uint32_t LCKR;    /*GPIO configuration lock register,address offset: 0x1C*/
    _VO uint32_t AFR[2];  /*GPIO AFRL[0]:alternate function low, offset: 0x20; AFRL[0]:alternate function high,address offset: 0x24*/
} GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
    _VO uint32_t CR;
    _VO uint32_t PLLCFGR;
    _VO uint32_t CFGR;
    _VO uint32_t CIR;
    _VO uint32_t AHB1RSTR;
    _VO uint32_t AHB2RSTR;
    _VO uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    _VO uint32_t APB1RSTR;
    _VO uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    _VO uint32_t AHB1ENR;
    _VO uint32_t AHB2ENR;
    _VO uint32_t AHB3ENR;
    uint32_t RESERVED2;
    _VO uint32_t APB1ENR;
    _VO uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    _VO uint32_t AHB1LPENR;
    _VO uint32_t AHB2LPENR;
    _VO uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    _VO uint32_t APB1LPENR;
    _VO uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    _VO uint32_t BDCR;
    _VO uint32_t CSR;
    uint32_t RESERVED6[2];
    _VO uint32_t SSCGR;
    _VO uint32_t PLLI2SCFGR;
    _VO uint32_t PLLSAICFGR;
    _VO uint32_t DCKCFGR;
    _VO uint32_t CKGATENR;
    _VO uint32_t DCKCFGR2;
} RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
    _VO uint32_t IMR;   /*Interrupt mask  register,address offset: 0x00*/
    _VO uint32_t EMR;   /*Event mask  register,address offset: 0x04*/
    _VO uint32_t RTSR;  /*Rising trigger selection register,address offset: 0x08*/
    _VO uint32_t FTSR;  /*Falling trigger selection register,address offset: 0x0C*/
    _VO uint32_t SWIER; /*Software interrupt event register,address offset: 0x10*/
    _VO uint32_t PR;    /*Pending register,address offset: 0x14*/
} EXTI_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
    _VO uint32_t CR1;     /*SPI control register1,address offset: 0x00*/
    _VO uint32_t CR2;     /*SPI control register2,address offset: 0x04*/
    _VO uint32_t SR;      /*SPI status register,address offset: 0x08*/
    _VO uint32_t DR;      /*SPI data register,address offset: 0x0C*/
    _VO uint32_t CRCPR;   /*SPI CRC polynomial register,address offset: 0x10*/
    _VO uint32_t RXCRCR;  /*SPI RX CRC register,address offset: 0x14*/
    _VO uint32_t TXCRCR;  /*SPI TX CRC register,address offset: 0x18*/
    _VO uint32_t I2SCFGR; /*SPI I2S configuration register,address offset: 0x1C*/
    _VO uint32_t I2SPR;   /*SPI I2S prescaler register,address offset: 0x20*/

} SPI_RegDef_t;
/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
    _VO uint32_t MEMRMP;    /*Memory remap register,address offset: 0x00*/
    _VO uint32_t PMC;       /*Periphreal mode configuration register,address offset: 0x04*/
    _VO uint32_t EXTICR[4]; /*External interrupt configuration register,address offset: 0x08-0x14*/
    uint32_t RESERVE1[2];   /*Reserved,address offset: 0x18-0x1C*/
    _VO uint32_t CMPCR;     /*Compensation cell control register,address offset: 0x20*/
    uint32_t RESERVED1[2];  /*Reserved,address offset: 0x24-0x28*/
    _VO uint32_t CFGR;      /*External interrupt configuration register,address offset: 0x08-0x18*/
} SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
	_VO uint32_t CR1;			/*I2C control register1 CR1*/
	_VO uint32_t CR2;			/*I2C control register2 CR2*/
	_VO uint32_t OAR1;			/*I2C Own address register1*/
	_VO uint32_t OAR2;			/*I2C Own address register2*/
	_VO uint32_t DR;			/*I2C Data register*/
	_VO uint32_t SR1;			/*I2C status register*/
	_VO uint32_t SR2;			/*I2C status register*/
	_VO uint32_t CCR;			/*I2C clock control register*/
	_VO uint32_t TRISE;			/*I2C TRISE, FM/SM mode*/
	_VO uint32_t FLTR;			/*I2C Analog and digital noise filter register*/
}I2C_RegDef_t;


/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	_VO uint32_t SR;       /*Status register*/
	_VO uint32_t DR;		 /*Data register*/
	_VO uint32_t BRR;	     /*Baud rate register*/
	_VO uint32_t CR1;		 /*Control register1*/
	_VO uint32_t CR2;		 /*Control register2*/
	_VO uint32_t CR3;		 /*Control register3*/
	_VO uint32_t GTPR;	 /*Guard time and pre-scaler register*/
}USART_RegDef_t;




/*
 * peripheral definition peripheral base address to xxx_RegDef_t
 */
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)

#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)

#define USART1 ((USART_RegDef_t *)USART1_BASEADDR)
#define USART2 ((USART_RegDef_t *)USART2_BASEADDR)
#define USART3 ((USART_RegDef_t *)USART3_BASEADDR)
#define USART4 ((USART_RegDef_t *)UART4_BASEADDR)
#define USART5 ((USART_RegDef_t *)UART5_BASEADDR)
#define USART6 ((USART_RegDef_t *)USART6_BASEADDR)
/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() RCC->AHB1ENR |= (1 << 0)
#define GPIOB_PCLK_EN() RCC->AHB1ENR |= (1 << 1)
#define GPIOC_PCLK_EN() RCC->AHB1ENR |= (1 << 2)
#define GPIOD_PCLK_EN() RCC->AHB1ENR |= (1 << 3)
#define GPIOE_PCLK_EN() RCC->AHB1ENR |= (1 << 4)
#define GPIOF_PCLK_EN() RCC->AHB1ENR |= (1 << 5)
#define GPIOG_PCLK_EN() RCC->AHB1ENR |= (1 << 6)
#define GPIOH_PCLK_EN() RCC->AHB1ENR |= (1 << 7)
#define GPIOI_PCLK_EN() RCC->AHB1ENR |= (1 << 8)

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() RCC->APB1ENR |= (1 << 21)
#define I2C2_PCLK_EN() RCC->APB1ENR |= (1 << 22)
#define I2C3_PCLK_EN() RCC->APB1ENR |= (1 << 23)

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() RCC->APB2ENR |= (1 << 12)
#define SPI2_PCLK_EN() RCC->APB1ENR |= (1 << 14)
#define SPI3_PCLK_EN() RCC->APB1ENR |= (1 << 15)
#define SPI4_PCLK_EN() RCC->APB2ENR |= (1 << 13)

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() RCC->APB2ENR |= (1 << 4)
#define USART2_PCLK_EN() RCC->APB1ENR |= (1 << 17)
#define USART3_PCLK_EN() RCC->APB1ENR |= (1 << 18)
#define UART4_PCLK_EN() RCC->APB1ENR |= (1 << 19)
#define UART5_PCLK_EN() RCC->APB1ENR |= (1 << 20)
#define USART6_PCLK_EN() RCC->APB2ENR |= (1 << 5)

/*
 * Clock Enable Macros for SYSCFGx peripherals
 */
#define SYSCFG_PCLK_EN() RCC->APB2ENR |= (1 << 14)

/*
 * Clock disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DIS() RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_PCLK_DIS() RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_PCLK_DIS() RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_PCLK_DIS() RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_PCLK_DIS() RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_PCLK_DIS() RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_PCLK_DIS() RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_PCLK_DIS() RCC->AHB1ENR &= ~(1 << 7)
#define GPIOI_PCLK_DIS() RCC->AHB1ENR &= ~(1 << 8)

/*
 * Clock disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() RCC->APB1ENR &= ~(1 << 21)
#define I2C2_PCLK_DI() RCC->APB1ENR &= ~(1 << 22)
#define I2C3_PCLK_DI() RCC->APB1ENR &= ~(1 << 23)

/*
 * Clock disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() RCC->APB2ENR &= ~(1 << 12)
#define SPI2_PCLK_DI() RCC->APB1ENR &= ~(1 << 14)
#define SPI3_PCLK_DI() RCC->APB1ENR &= ~(1 << 15)
#define SPI4_PCLK_DI() RCC->APB2ENR &= ~(1 << 13)

/*
 * Clock disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() RCC->APB2ENR &= ~(1 << 4)
#define USART2_PCLK_DI() RCC->APB1ENR &= ~(1 << 17)
#define USART3_PCLK_DI() RCC->APB1ENR &= ~(1 << 18)
#define UART4_PCLK_DI() RCC->APB1ENR &= ~(1 << 19)
#define UART5_PCLK_DI() RCC->APB1ENR &= ~(1 << 20)
#define USART6_PCLK_DI() RCC->APB2ENR &= ~(1 << 5)

/*
 * Clock disable Macros for SYSCFGx peripherals
 */
#define SYSCFG_PCLK_DI() RCC->APB2ENR &= ~(1 << 14)

/*
 * Macros to reset GPIOx peripheral
 */
// Macro function for more than one parameters, use do..while
#define GPIOA_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 0));  \
        (RCC->AHB1RSTR &= ~(1 << 0)); \
    } while (0)
#define GPIOB_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 1));  \
        (RCC->AHB1RSTR &= ~(1 << 1)); \
    } while (0)
#define GPIOC_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 2));  \
        (RCC->AHB1RSTR &= ~(1 << 2)); \
    } while (0)
#define GPIOD_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 3));  \
        (RCC->AHB1RSTR &= ~(1 << 3)); \
    } while (0)
#define GPIOE_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 4));  \
        (RCC->AHB1RSTR &= ~(1 << 4)); \
    } while (0)
#define GPIOF_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 5));  \
        (RCC->AHB1RSTR &= ~(1 << 5)); \
    } while (0)
#define GPIOG_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 6));  \
        (RCC->AHB1RSTR &= ~(1 << 6)); \
    } while (0)
#define GPIOH_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 7));  \
        (RCC->AHB1RSTR &= ~(1 << 7)); \
    } while (0)
#define GPIOI_REG_RESET()             \
    do                                \
    {                                 \
        (RCC->AHB1RSTR |= (1 << 8));  \
        (RCC->AHB1RSTR &= ~(1 << 8)); \
    } while (0)

/*
 * Macros to reset SPIx  peripheral
 */
// Macro function for more than one parameters, use do..while

#define SPI1_REG_RESET()		do {RCC->APB2RSTR |= (1 << 12);RCC->APB2RSTR &= ~(1 << 12);}while(0)
#define SPI2_REG_RESET()		do {RCC->APB1RSTR |= (1 << 14);RCC->APB1RSTR &= ~(1 << 14);}while(0)
#define SPI3_REG_RESET()		do {RCC->APB1RSTR |= (1 << 15);RCC->APB1RSTR &= ~(1 << 15);}while(0)
#define SPI4_REG_RESET()		do {RCC->APB2RSTR |= (1 << 13);RCC->APB2RSTR &= ~(1 << 13);}while(0)

/*
 * Macros to reset I2Cx  peripheral
 */
// Macro function for more than one parameters, use do..while
#define I2C1_REG_RESET()		do{RCC->APB1RSTR |= (1 << 21);RCC->APB1RSTR &= ~(1 << 21);}while(0)
#define I2C2_REG_RESET()		do{RCC->APB1RSTR |= (1 << 22);RCC->APB1RSTR &= ~(1 << 22);}while(0)
#define I2C3_REG_RESET()		do{RCC->APB1RSTR |= (1 << 23);RCC->APB1RSTR &= ~(1 << 23);}while(0)




/*
 * Return port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : (x == GPIOB) ? 1 \
                                                 : (x == GPIOC)   ? 2 \
                                                 : (x == GPIOD)   ? 3 \
                                                 : (x == GPIOE)   ? 4 \
                                                 : (x == GPIOF)   ? 5 \
                                                 : (x == GPIOG)   ? 6 \
                                                 : (x == GPIOH)   ? 7 \
                                                 : (x == GPIOI)   ? 8 \
                                                                  : 0)

/*
 * IRQ(interrupt Request number) of STM32F407VG MCU
 *NOTE: update these macros with valid values according to your MCU
 *
 */
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40

#define IRQ_NO_SPI1  35
#define IRQ_NO_SPI2	 36
#define IRQ_NO_SPI3	 51
#define IRQ_NO_SPI4  84
#define IRQ_NO_SPI5	 85
#define IRQ_NO_SPI6  86

#define IRQ_NO_I2C1_EV	31
#define IRQ_NO_I2C1_ER	32
#define IRQ_NO_I2C2_EV	33
#define IRQ_NO_I2C2_ER	34
#define IRQ_NO_I2C3_EV	72
#define IRQ_NO_I2C3_ER	73

#define IRQ_NO_USART1	37
#define IRQ_NO_USART2   38
#define IRQ_NO_USART3	39
#define IRQ_NO_UART4	52
#define IRQ_NO_UART5	53
#define IRQ_NO_USART6	71
/*
 * Macros for all possible priority levels
 */
#define NVIC_IRQ_PRI0 0
#define NVIC_IRQ_PRI15 15

// generic macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET RESET
#define FLAG_SET SET

/****************************************************
 * Bit position definitions of SPI peripheral
 * *************************************************/
/*
 * Bit position definition in SPI_CR1,control register
 */
#define SPI_CR1_CPHA_OFFSET 0
#define SPI_CR1_CPOL_OFFSET 1
#define SPI_CR1_MSTR_OFFSET 2
#define SPI_CR1_BR_OFFSET 3
#define SPI_CR1_SPE_OFFSET 6
#define SPI_CR1_LSBFIRST_OFFSET 7
#define SPI_CR1_SSI_OFFSET 8
#define SPI_CR1_SSM_OFFSET 9
#define SPI_CR1_RXONLY_OFFSET 10
#define SPI_CR1_DFF_OFFSET 11
#define SPI_CR1_CRCNEXT_OFFSET 12
#define SPI_CR1_CRCEN_OFFSET 13
#define SPI_CR1_BIDIOE_OFFSET 14
#define SPI_CR1_BIDIMODE_OFFSET 15

/*
 * Bit position definition in SPI_CR2,control register
 */
#define SPI_CR2_RXDMAEN_OFFSET 0
#define SPI_CR2_TXDMAEN_OFFSET 1
#define SPI_CR2_SSOE_OFFSET 2
#define SPI_CR2_RES_OFFSET 3
#define SPI_CR2_FRF_OFFSET 4
#define SPI_CR2_ERRIE_OFFSET 5
#define SPI_CR2_RXNEIE_OFFSET 6
#define SPI_CR2_TXEIE_OFFSET 7

/*
 * Bit position definition in SPI_SR, status register
 */
#define SPI_SR_RXNE_OFFSET 0
#define SPI_SR_TXE_OFFSET 1
#define SPI_SR_CHSIDE_OFFSET 2
#define SPI_SR_UDR_OFFSET 3
#define SPI_SR_CRCERR_OFFSET 4
#define SPI_SR_MODF_OFFSET 5
#define SPI_SR_OVR_OFFSET 6
#define SPI_SR_BSY_OFFSET 7
#define SPI_SR_FRE_OFFSET 8


/****************************************************
 * Bit position definitions of I2C peripheral
 * *************************************************/

/*
 * Bit position definition in I2C_CR1, control register
 */
#define I2C_CR1_PE 			0			/*Peripheral enable*/
#define I2C_CR1_NOSTRETCH	7			/*Clock stretching disable (Slave mode)*/
#define I2C_CR1_START		8			/*Start generation*/
#define I2C_CR1_STOP		9			/*Stop generation*/
#define I2C_CR1_ACK			10			/*Acknowledge enable*/
#define I2C_CR1_SWRST		15			/*Software Reset*/

/*
 * Bit position definition in I2C_CR2, control register
 */
#define I2C_CR2_FREQ		0			/*Peripheral clock frequency:0b000001 2Mhz;..*/
#define I2C_CR2_ITERREN		8			/*Error interrupt enable*/
#define I2C_CR2_ITEVTEN		9			/*Event interrupt enable*/
#define I2C_CR2_ITBUFEN		10			/*Buffer interrupt enable*/

/*
 * Bit position definition in I2C_OAR1, Own address register1
 */
#define I2C_OAVR1_ADD0		0			/*Interface address,bit 0*/
#define I2C_OVAR1_ADD71		1			/*Interface address,bits 7:1*/
#define I2C_OVAR1_ADD98		8			/*Interface address,bits 9:8*/
#define I2C_OVAR1_ADDMODE	15			/*Addressing mode (slave mode)*/

/*
 * Bit position definition in I2C_SR1, Status register1
 */
#define I2C_SR1_SB			0			/*Start bit: (master mode)*/
#define I2C_SR1_ADDR		1			/*Address sent(master mode)/matched (slave mode)*/
#define I2C_SR1_BTF			2			/*Byte transfer finished*/
#define I2C_SR1_ADD10		3			/*10-bit header sent (Master mode)*/
#define I2C_SR1_STOPF		4			/*Stop detection (slave mode)*/
#define I2C_SR1_RXNE		6			/*Data register not empty (receivers),0:empty. 1:not empty*/
#define I2C_SR1_TXE			7			/*Data register empty, 0:not empty, 1:empty*/
#define I2C_SR1_BERR		8			/*Buss error*/
#define I2C_SR1_ARLO		9			/*Arbitration lost (master mode)*/
#define I2C_SR1_AF			10			/*acknowledge failure: 0: Not failure, 1: failure*/
#define I2C_SR1_OVR			11			/*Over-runn/Under-run*/
#define I2C_SR1_TIMEOUT		14			/*Time out or Tlow error*/

/*
 * Bit position definition in I2C_SR2, Status register2
 */
#define I2C_SR2_MSL			0			/*Master /Slave. 0: slave mode, 1: master mode*/
#define I2C_SR2_BUSY		1			/*Bus busy, */
#define I2C_SR2_TRA			2			/*Transmitter / receivers,0:data bytes received,1: Data bytes transmitted*/
#define I2C_SR2_GENCALL		4			/*General call address(slave mode),0:No general call, 1:General call address received when ENGC =1*/
#define I2C_SR2_DUALF		7			/*Dual flag(slave mode, 0:received address matched with OAR1,1:received address matched with OAR2*/

/*
 * Bit position definition in I2C_CCR, clock control register
 */
#define I2C_CCR_CCR			0			/*Clock control reister in Fm/Sm mode(master mode)*/
#define I2C_CCR_DUTY		14			/*Fm mode duty cycle,0:Fm mode tlow/thigh = 1*/
#define I2C_CCR_FS			15			/*I2C master mode selection*/

/****************************************************
 * Bit position definitions of USART peripheral
 * *************************************************/
/*
 * Bit position definition in USART_CR1, control register
 */
#define USART_CR1_SBK		0			/*Send break*/
#define USART_CR1_RWU		1			/*Receiver wake-up*/
#define USART_CR1_RE		2           /*Receiver enable*/
#define USART_CR1_TE		3 			/*Transmitter enable*/
#define USART_CR1_IDLEIE	4			/*IDLE interrupt enable*/
#define USART_CR1_RXNEIE	5			/*RXNE interrupt enable*/
#define USART_CR1_TCIE		6			/*Transmission completed interrupt enable*/
#define USART_CR1_TXEIE		7			/*TXE interrupt enable*/
#define USART_CR1_PEIE		8			/*PE interrupt enable*/
#define USART_CR1_PS		9			/*Parity selection*/
#define USART_CR1_PCE		10			/*Parity control enable*/
#define USART_CR1_WAKE		11			/*wake-up method*/
#define USART_CR1_M			12			/*Word Length*/
#define USART_CR1_UE		13			/*USART enable*/
#define USART_CR1_OVER8		15			/*Over-sampling mode*/

/*
 * Bit position definition in USART_CR2, control register
 */
#define USART_CR2_ADD   	0
#define USART_CR2_LBDL   	5
#define USART_CR2_LBDIE  	6
#define USART_CR2_LBCL   	8
#define USART_CR2_CPHA   	9
#define USART_CR2_CPOL   	10
#define USART_CR2_STOP      12          /*Stop bits selection */
#define USART_CR2_LINEN		14

/*
 * Bit position definition in USART_CR3, control register
 */
#define USART_CR3_EIE      0
#define USART_CR3_IREN     1
#define USART_CR3_IRLP     2
#define USART_CR3_HDSEL    3
#define USART_CR3_NACK     4
#define USART_CR3_SCEN     5
#define USART_CR3_DMAR     6
#define USART_CR3_DMAT     7
#define USART_CR3_RTSE     8           /*RTS enable bit*/
#define USART_CR3_CTSE     9           /*CTS enable bit*/
#define USART_CR3_CTSIE	   10		   /*CTS interrupt enable*/
#define USART_CR3_ONEBIT   11

/*
 * Bit position definition in USART_SR, control register
 */
#define USART_SR_PE			0			/*Parity error*/
#define USART_SR_FE			1			/*Framing error*/
#define USART_SR_NF			2			/*Noise detected flag*/
#define USART_SR_ORE		3			/*Overrun error*/
#define USART_SR_IDLE		4			/*IDLE line detected*/
#define USART_SR_RXNE		5			/*Read data register not empty*/
#define USART_SR_TC			6			/*Transmission complete*/
#define USART_SR_TXE		7			/*Transmit data register empty*/
#define USART_SR_LBD		8			/*LIN break detection flag*/
#define USART_SR_CTS		9			/*CTS flag*/







#include "STM32f407VG_gpio_driver.h"
#include "STM32f407VG_SPI_driver.h"
#include "STM32f407VG_I2C_driver.h"
#include "STM32f407VG_USART_driver.h"
#include "STM32f407VG_RccDriver.h"

#endif /* INC_STM32F407VG_H_ */
