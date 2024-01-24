/*
 * stm32f446re.h
 *
 *  Created on: Jan 23, 2024
 *      Author: Alister
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_
#include<stdint.h>

#define __vo volatile

/*
 *  base address of FLASH and SRAM1 memories
 */
#define FLASH_BASEADDR				Ox08000000U 			/* Base Address of Flash Memory */
#define SRAM1_BASEADDR				0x20000000U  			/* Base Address of SRAM1 Memory */
#define SRAM2_BASEADDR				0x20001C00U				/* Base Address of SRAM2 Memory */
#define ROM							0x1FFF0000U				/* Base Address of ROM Memory */
#define OTP_BASEADDR				0x1FFF7800U				/* Base Address of OTP Memory */
#define SRAM 						SRAM1_BASEADDR			/* Base Address of SRAM Memory */

/*
 * bus domain address of MCU. AHBx and APHx Bus
 * */
#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x50000000U

/*
 * Base address of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR 			(AHB1PERIPH_BASE + 0x3800)

/*
 * Base address of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)

/*
 * Base address of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 3C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 3000)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 1400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 3800)





/*********************** Structuring of peripheral registers ***************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g: Number of registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Please check your Device RM for more details.
 */

typedef struct
{
	__vo uint32_t MODER;					/*!< GPIO port mode register, Address offset : 0x00 */
	__vo uint32_t OTYPER;				/*!< GPIO port output type register, Address offset : 0x04 */
	__vo uint32_t OSPEEDER;				/*!< GPIO port output speed register, Address offset : 0x08 */
	__vo uint32_t PUPDR; 				/*!< GPIO port pull-up/pull-down register , Address offset : 0x0C */
	__vo uint32_t IDR;					/*!< GPIO port pull-up/pull-down register , Address offset : 0x10 */
	__vo uint32_t ODR;					/*!< GPIO port output data register , Address offset : 0x14 */
	__vo uint32_t BSRR;					/*!< GPIO port bit set/reset register , Address offset : 0x18 */
	__vo uint32_t LCKR;					/*!< GPIO port configuration lock register , Address offset : 0x1C */
	__vo uint32_t AFR[2];				/*!< GPIO port configuration lock register , Address offset : 0x20-0x24 */
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t	  RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t      RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;


}RCC_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses type casted to xxx_RegDef_t)
 */

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

/*
 * Clock enable macros for USARTx peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PLCK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))


/*
 * Clock enable macros for SYFCFG peripherals
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

/*
 * Clock DISABLE macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<23))

/*
 * Clock Disable macros for SPIx peripherals
 */
#define SPI1_PCLK_IN()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_IN()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_IN()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_IN()		(RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock Disable macros for SYFCFG peripherals
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

// Generic Macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET


#endif /* INC_STM32F446RE_H_ */
