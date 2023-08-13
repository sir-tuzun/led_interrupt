/*
 * stm32f411xx.h
 *
 *  Created on: Aug 4, 2023
 *      Author: murattuzun
 */
#include <stdint.h>
#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#define __vol volatile

//***************************************** PROCESSOR SPECIFIC DETAILS ******************************************
/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0				((__vol uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vol uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vol uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vol uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0				((__vol uint32_t*)0xE000E180)
#define NVIC_ICER1				((__vol uint32_t*)0xE000E184)
#define NVIC_ICER2				((__vol uint32_t*)0xE000E188)
#define NVIC_ICER3				((__vol uint32_t*)0xE000E18C)

/*
 * ARM Cortex Mx Processor NVIC IPRx register base addresses
 */
#define NVIC_PR_BASE_ADDR 		((__vol uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor NVIC IRQx register's number of unimplemented bits.
 */
#define NO_PR_BITS_IMPLEMENTED  4
/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM 					SRAM1_BASEADDR

//STM32F411XC/E BUS REGISTERS BASE ADDRESS
#define APB1_BASEADDR			0x40000000U
#define APB2_BASEADDR			0x40010000U
#define AHB1_BASEADDR			0x40020000U
#define AHB2_BASEADDR			0x50000000U

/*
 * HERE IS ALL OF THE AHB1 BUS PERIPHERAL
 */

//STM32F411XC/E GPIOx REGISTERS BASE ADDRESS
#define	GPIOA_BASEADDR			(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1_BASEADDR + 0x1000)
#define GPIOH_BASEADDR			(AHB1_BASEADDR + 0x1C00)

#define CRC_BASEADDR			(AHB1_BASEADDR + 0x3000)
#define RCC_BASEADDR			(AHB1_BASEADDR + 0x3800)

/*
 * HERE IS ALL OF APB1 BUS PERIGHERAL
 */
#define I2C1_BASEADDR			(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1_BASEADDR + 0x5C00)
#define SPI3_BASEADDR			(APB1_BASEADDR + 0x5C00)

#define USART2_BASEADDR			(APB1_BASEADDR + 0x5C00)

/*
 * HERE IS ALL OF APB2 BUS PERIGHERAL
 */
#define SPI1_BASEADDR			(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2_BASEADDR + 0x3400)
#define SPI5_BASEADDR			(APB2_BASEADDR + 0x5000)

#define USART1_BASEADDR			(APB2_BASEADDR + 0x1400)
#define USART6_BASEADDR			(APB2_BASEADDR + 0x1000)

#define EXTI_BASEADDR			(APB2_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR			(APB2_BASEADDR + 0x3800)

/*
 * The registers which are handled by SPI
 */

#define SPI1_CR1_BASEADDR 		(SPI1_BASEADDR + 0x00) 		// SPI1 Control Register

#define SPI1_SR_BASEADDR 		(SPI1_BASEADDR + 0x08) 		// SPI1 Satus Register
#define SPI1_DR_BASEADDR 		(SPI1_BASEADDR + 0x0C) 		// SPI1 Data Register

#define SPI1_CRCPR_BASEADDR 	(SPI1_BASEADDR + 0x10) 		// SPI1 CRC Polynomial Register
#define SPI1_RXCRCR_BASEADDR 	(SPI1_BASEADDR + 0x14) 		// SPI1 RX CRC Register
#define SPI1_TXCRCR_BASEADDR 	(SPI1_BASEADDR + 0x18) 		// SPI1 TX CRC Register

#define SPI1_I2SCFGR_BASEADDR 	(SPI1_BASEADDR + 0x1C) 		// SPI1 Configuration Register

#define SPI1_I2SPR_BASEADDR 	(SPI1_BASEADDR + 0x20) 		// SPI1 Prescaler Register

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct {
	__vol uint32_t MODER;		/* GPIO port mode register 							Address offset: 0x00 */
	__vol uint32_t OTYPER;		/* GPIO port output type register 					Address offset: 0x04 */
	__vol uint32_t OSPEEDER;	/* GPIO port output speed register 					Address offset: 0x08 */
	__vol uint32_t PUPDR;		/* GPIO port pull-up/pull-down register 			Address offset: 0x0C */
	__vol uint32_t IDR;			/* GPIO port input data register 					Address offset: 0x10 */
	__vol uint32_t ODR;			/* GPIO port output data register 					Address offset: 0x14 */
	__vol uint32_t BSRR;		/* GPIO port bit set/reset register 				Address offset: 0x18 */
	__vol uint32_t LCKR;		/* GPIO port configuration lock register 			Address offset: 0x1C */
	__vol uint32_t AFR[2];		/* AFR[0] for GPIO alternate function low  register	Address offset: 0x20
								 * AFR[1] for GPIO alternate function high register Address offset: 0x24
								 */
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */

typedef struct {
	__vol uint32_t CR;			/* GPIO port mode register 							Address offset: 0x00 */
	__vol uint32_t PLLCFGR;		/* GPIO port output type register 					Address offset: 0x04 */
	__vol uint32_t CFGR;		/* GPIO port output speed register 					Address offset: 0x08 */
	__vol uint32_t CIR;			/* GPIO port pull-up/pull-down register 			Address offset: 0x0C */
	__vol uint32_t AHB1RSTR;	/* GPIO port input data register 					Address offset: 0x10 */
	__vol uint32_t AHB2RSTR;	/* GPIO port output data register 					Address offset: 0x14 */
		  uint32_t res1[2];
	__vol uint32_t APB1RSTR;	/* GPIO port bit set/reset register 				Address offset: 0x18 */
	__vol uint32_t APB2RSTR;	/* GPIO port configuration lock register 			Address offset: 0x1C */
		  uint32_t res2[2];
	__vol uint32_t AHB1ENR;		/* AFR[0] for GPIO alternate function low  register	Address offset: 0x20 */
	__vol uint32_t AHB2ENR;
		  uint32_t res3[2];
	__vol uint32_t APB1ENR;
	__vol uint32_t APB2ENR;
		  uint32_t res4[2];
	__vol uint32_t AHB1LPENR;
	__vol uint32_t AHB2LPENR;
		  uint32_t res5[2];
	__vol uint32_t APB1LPENR;
	__vol uint32_t APB2LPENR;
		  uint32_t res6[2];
	__vol uint32_t BDCR;
	__vol uint32_t CSR;
		  uint32_t res7[2];
	__vol uint32_t SSCGR;
	__vol uint32_t PLLI2SCFGR;
		  uint32_t res8;
	__vol uint32_t DCKCF;

}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct {
	__vol uint32_t IMR;
	__vol uint32_t EMR;
	__vol uint32_t RTSR;
	__vol uint32_t FTSR;
	__vol uint32_t SWIER;
	__vol uint32_t PR;

}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */

typedef struct {
	__vol uint32_t MEMRMP;
	__vol uint32_t PMC;
	__vol uint32_t EXTICR[4];
	__vol uint32_t CMPCR;

}SYSCFG_RegDef_t;

#define	GPIOA	((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG 	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/*
 * @GPIO_PIN_NUMBERS
 * GPIO PIN NUMBERS
 */

#define GPIO_PIN_NO_0 	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible mode types
 */

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible output types
 */

#define GPIO_OUTPUT_TYPE_PP	0
#define GPIO_OUTPUT_TYPE_OD 1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible speed types
 */

#define GPIO_SPEED_TYPE_LS	0
#define GPIO_SPEED_TYPE_MS	1
#define GPIO_SPEED_TYPE_FS	2
#define GPIO_SPEED_TYPE_HS	3

/*
 * @GPIO_PIN_PU_PD_CONTROL
 * GPIO pin possible pull-up/pull-down types
 */

#define GPIO_PU_PD_TYPE_NONE	0
#define GPIO_PU_PD_TYPE_UP		1
#define GPIO_PU_PD_TYPE_DOWN	2
#define GPIO_PU_PD_TYPE_RES		3

/*
 *
 */

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PERI_CLC_EN() 	(RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PERI_CLC_EN() 	(RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PERI_CLC_EN() 	(RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PERI_CLC_EN() 	(RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PERI_CLC_EN() 	(RCC -> AHB1ENR |= (1 << 4))
#define GPIOH_PERI_CLC_EN() 	(RCC -> AHB1ENR |= (1 << 7))


/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PERI_CLC_EN()		(RCC -> APB1ENR |= (1 << 21))
#define I2C2_PERI_CLC_EN()		(RCC -> APB1ENR |= (1 << 22))
#define I2C3_PERI_CLC_EN()		(RCC -> APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PERI_CLC_EN()		(RCC -> APB2ENR |= (1 << 12))
#define SPI4_PERI_CLC_EN()		(RCC -> APB2ENR |= (1 << 13))
#define SPI5_PERI_CLC_EN()		(RCC -> APB2ENR |= (1 << 20))

#define SPI2_PERI_CLC_EN()		(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PERI_CLC_EN()		(RCC -> APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART2_PERI_CLC_EN()	(RCC -> APB1ENR |= (1 << 17))

#define USART1_PERI_CLC_EN()	(RCC -> APB2ENR |= (1 << 4))
#define USART6_PERI_CLC_EN()	(RCC -> APB2ENR |= (1 << 5))


/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PERI_CLC_EN() 	(RCC -> APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PERI_CLC_DI() 	(RCC -> AHB1ENR &= ~(1 << 0))
#define GPIOB_PERI_CLC_DI() 	(RCC -> AHB1ENR &= ~(1 << 1))
#define GPIOC_PERI_CLC_DI() 	(RCC -> AHB1ENR &= ~(1 << 2))
#define GPIOD_PERI_CLC_DI() 	(RCC -> AHB1ENR &= ~(1 << 3))
#define GPIOE_PERI_CLC_DI()		(RCC -> AHB1ENR &= ~(1 << 4))
#define GPIOH_PERI_CLC_DI() 	(RCC -> AHB1ENR &= ~(1 << 7))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PERI_CLC_DI()		(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PERI_CLC_DI()		(RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_PERI_CLC_DI()		(RCC -> APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PERI_CLC_DI()		(RCC -> APB2ENR &= ~(1 << 12))
#define SPI4_PERI_CLC_DI()		(RCC -> APB2ENR &= ~(1 << 13))
#define SPI5_PERI_CLC_DI()		(RCC -> APB2ENR &= ~(1 << 20))

#define SPI2_PERI_CLC_DI()		(RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PERI_CLC_DI()		(RCC -> APB1ENR &= ~(1 << 15))

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART2_PERI_CLC_DI()	(RCC -> APB1ENR &= ~(1 << 17))

#define USART1_PERI_CLC_DI()	(RCC -> APB2ENR &= ~(1 << 4))
#define USART6_PERI_CLC_DI()	(RCC -> APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSFG_PERI_CLC_DI()		(RCC -> APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET() 	do{ (RCC -> AHB1RSTR |= (1 << 0)); (RCC -> AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() 	do{ (RCC -> AHB1RSTR |= (1 << 1)); (RCC -> AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() 	do{ (RCC -> AHB1RSTR |= (1 << 2)); (RCC -> AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() 	do{ (RCC -> AHB1RSTR |= (1 << 3)); (RCC -> AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 4)); (RCC -> AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOH_REG_RESET() 	do{ (RCC -> AHB1RSTR |= (1 << 7)); (RCC -> AHB1RSTR &= ~(1 << 7)); } while(0)

/*
 * This macro returns port code for given GPIO base address
 */

#define GPIO_BASEADDR_TO_CODE(x) (	(x == GPIOA) ? 0x0000 : \
									(x == GPIOB) ? 0x0001 : \
									(x == GPIOC) ? 0x0010 : \
									(x == GPIOD) ? 0x0011 : \
									(x == GPIOE) ? 0x0100 : \
									(x == GPIOH) ? 0x0111 : 0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F411x MCU
 * NOTE: update these macros with valid values accourding to your MCU
 *
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/*
 * Some Generic Macros
 */

#define ENABLE 					1
#define DISABLE 				1
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET

#include "stm32f411xx_gpio_driver.h"


#endif /* INC_STM32F411XX_H_ */
