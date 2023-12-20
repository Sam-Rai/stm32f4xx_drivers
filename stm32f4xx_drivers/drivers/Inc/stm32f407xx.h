/*
 * stm32f407xx.h
 *
 *  Created on: Oct 13, 2023
 *      Author: samra
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

#define __vo volatile

/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR		0x80000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x20001C00U
#define ROM_BASEADDR		0x1FFF0000U
#define SRAM				SRAI1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 *
 */
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASE
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */
#define GPIOA_BASEADDR 		(AHB1PERIPH_BASEADDR +	0x0000) // AHB1PERIPH_BASE + offset of GPIOx
#define GPIOB_BASEADDR 		(AHB1PERIPH_BASEADDR +	0x0400)
#define GPIOC_BASEADDR 		(AHB1PERIPH_BASEADDR +	0x0800)
#define GPIOD_BASEADDR 		(AHB1PERIPH_BASEADDR +	0x0C00)
#define GPIOE_BASEADDR 		(AHB1PERIPH_BASEADDR +	0x1000)
#define GPIOF_BASEADDR 		(AHB1PERIPH_BASEADDR +	0x1400)
#define GPIOG_BASEADDR 		(AHB1PERIPH_BASEADDR +	0x1800)
#define GPIOH_BASEADDR 		(AHB1PERIPH_BASEADDR +	0x1C00)
#define GPIOI_BASEADDR 		(AHB1PERIPH_BASEADDR +	0x2000)

#define RCC_BASEADDR		(AHB1PERIPH_BASEADDR +	0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */

#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */

#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDr + 0x1400)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	__vo uint32_t MODER;			// GPIO port mode register at off set of 0x00
	__vo uint32_t OTYPER;		// GPIO port output type register at off set of 0x04
	__vo uint32_t OSPEEDR;		// GPIO port output speed register at off set of 0x08
	__vo uint32_t PUPDR;			// GPIO port pull-up/pull-down register at off set of 0x0C
	__vo uint32_t IDR;			// GPIO port input data register at off set of 0x10
	__vo uint32_t ODR;			// GPIO port output data register at off set of 0x14
	__vo uint32_t BSRR;			// GPIO port bit set/reset register at off set of 0x18
	__vo uint32_t LCKR;			// GPIO port configuration lock register at off set of 0x1C
	__vo uint32_t AFR[2]; 		//GPIO alternate function low register
							//array of AFRL/AFR[0] for low registers and AFRH/AFR[1] for high registers (one for low and one for high) offset of 0x20 and 0x24

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			//Clock control register									                       	Address Offset: 0x00
	__vo uint32_t PLLCFGR;		//PLL configuration register									  					Address Offset: 0x04
	__vo uint32_t CFGR;			//Clock configuration register														Address Offset: 0x08
	__vo uint32_t CIR;			//Clock interrupt register															Address Offset: 0x0C
	__vo uint32_t AHB1RSTR;		//AHB1 peripheral reset register													Address Offset: 0x10
	__vo uint32_t AHB2RSTR;		//AHB2 peripheral reset register													Address Offset: 0x14
	__vo uint32_t AHB3RSTR;		//AHB3 peripheral reset register													Address Offset: 0x18
	uint32_t RESERVED0;																								//Address Offset: 0x1C
	__vo uint32_t APB1RSTR;		//APB1 peripheral reset register													Address Offset: 0x20
	__vo uint32_t APB2RSTR; 	//APB2 peripheral reset register													Address Offset: 0x24
	uint32_t RESERVED1[2];																							//Address Offset: 0x28  and 0x2C
	__vo uint32_t AHB1ENR;		//AHB1 peripheral clock enable register												Address Offset: 0x30
	__vo uint32_t AHB2ENR;		//AHB2 peripheral clock enable register												Address Offset: 0x34
	__vo uint32_t AHB3ENR;		//AHB3 peripheral clock enable register												Address Offset: 0x38
	uint32_t RESERVED2;																								//Address Offset: 0x3C
	__vo uint32_t APB1ENR;		//APB1 peripheral clock enable register												Address Offset: 0x40
	__vo uint32_t APB2ENR;		//APB2 peripheral clock enable register												Address Offset: 0x44
	uint32_t RESERVED3[2];																							//Address Offset: 0x48 and 0x4C
	__vo uint32_t AHB1LPENR;	//AHB1 peripheral clock enable in low power mode register							Address Offset: 0x50
	__vo uint32_t AHB2LPENR;	//AHB2 peripheral clock enable in low power mode register							Address Offset: 0x54
	__vo uint32_t AHB3LPENR;	//AHB3 peripheral clock enable in low power mode register							Address Offset: 0x58
	uint32_t RESERVED4;																								//Address Offset: 0x5C
	__vo uint32_t APB1LPENR;	//APB1 peripheral clock enable in low power mode register							Address Offset: 0x60
	__vo uint32_t APB2LPENR;	//APB2 peripheral clock enable in low power mode register							Address Offset: 0x64
	uint32_t RESERVED5[2];																							//Address Offset: 0x68 and 0x6C
	__vo uint32_t BDCR;			//Backup domain control register													Address Offset: 0x70
	__vo uint32_t CSR;			//Clock control & status register													Address Offset: 0x74
	uint32_t RESERVED6[2];																							//Address Offset: 0x78 and 0x7C
	__vo uint32_t SSCGR;		//Spread spectrum clock generation register											Address Offset: 0x80
	__vo uint32_t PLLI2SCFGR;	//PLLI2S configuration register														Address Offset: 0x84





}RCC_RegDef_t;

/*
 * peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))
/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN()		(RCC->APB2ENR |= (1 << 4))

#define USART2_PCCK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN()		(RCC->APB1ENR |= (1 << 18))

#define UART4_PCCK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()		(RCC->APB1ENR |= (1 << 20))

#define USART6_PCCK_EN()		(RCC->APB2ENR |= (1 << 5))
/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))
//**************************************
/*
 * BELOW ARE DISABLE PERPHERALS
 */
//**************************************
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
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))
/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))
/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= (1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= (1 << 13))
/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCCK_DI()		(RCC->APB2ENR &= (1 << 4))

#define USART2_PCCK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCCK_DI()		(RCC->APB1ENR &= ~(1 << 18))

#define UART4_PCCK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCCK_DI()			(RCC->APB1ENR &= ~(1 << 20))

#define USART6_PCCK_DI()		(RCC->APB2ENR &= (1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 *
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0)); }while(0)  //do-while loop
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2)); }while(0)


//Some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET


#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
