/*
 * stm32f401xx.h
 *
 *  Created on: Nov 23, 2024
 *      Author: Nirvan
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_
#include <stdint.h>
#define __vo volatile
/*
 * Base addresses of SRAM and FLASH Memories
 * */
#define FLASH_BASEADDR		0x08000000U	/*Main Memory base address*/
#define SRAM1_BASEADDR		0x20000000U /*SRAM base address*/
#define ROM_BASEADDR		0x1FFF0000U	/*System Memory or ROM Base address*/
#define SRAM 				SRAM1_BASEADDR

/*
 * Base addresses of Peripheral Bus (APBx, AHBX)
 * */

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/*
 * Base addresses of Peripherals on AHB1 Bus
 * */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)

/*
 * Base addresses of Peripherals on APB1 Bus
 * */
#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0X5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0X5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0X5C00)

#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0X3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0X3C00)

#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)

/*
 * Base addresses of Peripherals on APB2 Bus
 * */
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0X3C00)

#define SPI1_BASEADDR		(APB1PERIPH_BASE + 0X3000)
#define SPI4_BASEADDR		(APB1PERIPH_BASE + 0X3400)

#define USART1_BASEADDR		(APB1PERIPH_BASE + 0x1000)
#define USART6_BASEADDR		(APB1PERIPH_BASE + 0x1400)

#define SYSCFG_BASEADDR		(APB1PERIPH_BASE + 0x3800)

/*
 * Defining the GPIOx peripheral registers
 * */

typedef struct{
	__vo uint32_t MODER; 			/*GPIO port mode register 			-> 	Address offset: 0x00*/
	__vo uint32_t OTYPER; 			/*GPIO o/p type register 			-> 	Address offset: 0x04*/
	__vo uint32_t OSPEEDR; 			/*GPIO o/p speed register 			-> 	Address offset: 0x08*/
	__vo uint32_t PUPDR; 			/*GPIO pull up/ pull down register 	-> 	Address offset: 0x0C*/
	__vo uint32_t IDR; 				/*GPIO i/p data register 			-> 	Address offset: 0x10*/
	__vo uint32_t ODR; 				/*GPIO o/p data register 			-> 	Address offset: 0x14*/
	__vo uint32_t BSRR; 			/*GPIO port bit set/reset register 	-> 	Address offset: 0x18*/
	__vo uint32_t LCKR; 			/*GPIO port lock  register 			-> 	Address offset: 0x1C*/
	__vo uint32_t AFR[2]; 			/*GPIO port AFR[0], AFR[1]- low,high -> Address offset: 0x20 - 0x24*/
}GPIO_RegDef_t;

/*SPIx Peripheral registers*/

typedef struct{
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
}SPI_RegDef_t;

typedef struct{
	__vo uint32_t 	CR;
	__vo uint32_t 	PLLCFGR;
	__vo uint32_t 	CFGR;
	__vo uint32_t 	CIR;
	__vo uint32_t  	AHB1RSTR;
	__vo uint32_t 	AHB2RSTR;
	uint32_t 	RESERVED0[2];
	__vo uint32_t 	APB1RSTR;
	__vo uint32_t	APB2RSTR;
	uint32_t 	RESERVED1[2];
	__vo uint32_t	AHB1ENR;
	__vo uint32_t	AHB2ENR;
	uint32_t	RESERVED2[2];
	__vo uint32_t	APB1ENR;
	__vo uint32_t	APB2ENR;
	uint32_t	RESERVED3[2];
	__vo uint32_t 	AHB1LPENR;
	__vo uint32_t 	AHB2LPENR;
	uint32_t 	RESERVED4[2];
	__vo uint32_t	APB1LPENR;
	__vo uint32_t	APB2LPENR;
	uint32_t	RESERVED5[2];
	__vo uint32_t	BDCR;
	__vo uint32_t 	CSR;
	uint32_t 	RESERVED6[2];
	__vo uint32_t	SSCGR;
	__vo uint32_t	PLLI2SCFGR;
	__vo uint32_t	DCKCFGR;

}RCC_RegDef_t;

/*
 * Defining macros for GPIO peripherals type casted using GPIO_RegDef_t
 * */

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
/*
 * Defining macros for RCC peripheral definition type casted using RCC_RegDef_t
 * */
#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

/*SPIx type casted using SPI_RegDef_t*/
#define SPI1 ((SPI_RegDef_t*)(SPI1_BASEADDR))
#define SPI2 ((SPI_RegDef_t*)(SPI2_BASEADDR))
#define SPI3 ((SPI_RegDef_t*)(SPI3_BASEADDR))
#define SPI4 ((SPI_RegDef_t*)(SPI4_BASEADDR))

/*
 * Enabling Clock for GPIOx Peripherals
 * */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1<<7))

/*
 * Enabling Clock for I2Cx Peripherals hanging on APB1
 * */

#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1<<23))

/*
 * Enabling Clock for SPIx Peripherals hanging on APBx
 * SPI2, and SPI3 on APB1
 * SPI1, and SPI4 on APB2
 * */

#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1<<12))
#define SPI4_PCLK_EN()	(RCC->APB2ENR |= (1<<13))

#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1<<15))

/*
 * Enabling Clock for USARTx Peripherals hanging on APBx
 * USART1, and USART6 on APB2
 * USART2 on APB1
 * */
#define USART2_PCLK_EN() 	(RCC->APB1ENR |= (1<<17))
#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1<<4))
#define USART6_PCLk_EN() 	(RCC->APB2ENR |= (1<<5))
/*
 * Enabling Clock for SYSCFG Peripherals hanging on APB2
 * */
#define SYSCFG_PCLK_EN() 	(RCC->APB2ENR |= (1<<14))


/*
 * Disabling Clock for GPIOx Peripherals
 * */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<7))

/*
 * Disabling Clock for I2Cx Peripherals hanging on APB1
 * */

#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()	(RCC->APB1ENR &= ~(1<<23))

/*
 * Disabling Clock for SPIx Peripherals hanging on APBx
 * SPI2, and SPI3 on APB1
 * SPI1, and SPI4 on APB2
 * */

#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1<<12))
#define SPI4_PCLK_DI()	(RCC->APB2ENR &= ~(1<<13))

#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~(1<<15))

/*
 * Disabling Clock for USARTx Peripherals hanging on APBx
 * USART1, and USART6 on APB2
 * USART2 on APB1
 * */
#define USART2_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<17))
#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<4))
#define USART6_PCLk_DI() 	(RCC->APB2ENR &= ~(1<<5))

/*
 * Disabling Clock for SYSCFG Peripherals hanging on APB2
 * */
#define SYSCFG_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<14))

/*
 * Register Reset for GPIOx Peripherals
 * */

#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)

/*
 * Miscellaneous Macros
 * */

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

#endif /* INC_STM32F401XX_H_ */
