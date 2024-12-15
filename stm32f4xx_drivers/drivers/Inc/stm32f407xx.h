/*
 * stm32f407xx.h
 *
 *  Created on: Nov 19, 2024
 *      Author: vibhu
 */
/*
 * This is a header file („C‟ header file in our case) which contains Microcontroller
specific details such as
1) The base addresses of various memories present in the microcontroller such
as (Flash, SRAM1,SRAM2,ROM,etc)
2) The base addresses of various bus domains such as (AHBx domain, APBx
domain)
3) Base addresses of various peripherals present in different bus domains of the
microcontroller
4) Clock management macros ( i.e clock enable and clock disable macros)
5) IRQ definitions
6) Peripheral Register definition structures
7) Peripheral register bit definitions
8) Other useful microcontroller configuration macros

Basically this header file describes the microcontroller

 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define FLASH_BASEADDR 0x08000000U  // base address of flash memory
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define ROM_BASEADDR 0x1FFF0000
#define SRAM SRAM1_BASEADDR

// will define the base addresses of peripherals hanging onto different buses in the microcontroller
//like APB1 base address for peripheral is 0x40000000 it means this is the base address of the peripherals hanging onto the APB1 bus check the memory map for more info
#define PERIPH_BASE       0x40000000U //in the memory map from this address the pripherals registers start
#define APB1PERIPH_BASE   0x40000000U //base address of the peripheral hanging onto the apb1 bus
#define APB2PERIPH_BASE   0x40010000U //base address of the peripheral hanging onto the apb2 bus
#define AHB1PERIPH_BASE   0x40020000U //base address of the peripheral hanging onto the ahb1 bus
#define AHB2PERIPH_BASE   0x50000000U //base address of the peripheral hanging onto the AHB2bus


//now lets define the base address to each and every peripheral what we are going to use
//first we will define for the peripheral; hanging onto the peripheral bus AHB1 that is GPIOs

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
// PROCESSOR SPECIFIC NVIC ENGINE REGISTERS
#define NVIC_ISER_BASEADDR 0xE000E100U
#define NVIC_ICER_BASEADDR 0xE000E180U
#define NVIC_ISPR_BASEADDR 0xE000E200U
#define NVIC_ICPR_BASEADDR 0xE000E280U
#define NVIC_IABR_BASEADDR 0xE000E300
#define NVIC_IPR_BASEADDR  0xE000E400
#define NVIC_STIR_BASEADDR 0xE000EF00

// now define base address of the peripherals hanging onto APB1

#define I2C1_BASEADDR (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0x5000)

//now define base address of the peripherals hanging on to APB2

#define SPI1_BASEADDR (APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR (APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR (APB2PERIPH_BASE + 0x3800)

//SOME GENERIC MACROs

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_HIGH SET
#define GPIO_PIN_LOW RESET

#define PA 0000
#define PB 0001
#define PC 0010
#define PD 0011
#define PE 0100
#define PF 0101
#define PG 0110
#define PH 0111
#define PI 1000

//IRQ numbers for STM32f4xx series microcontrollers
// UPDATE FOR ALL PERIPHERALS FROM Reference manual of the MCU

#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40



#include <stdint.h>

//    Peripheral register definitions structures
/*
 * Registers of a peripheral of SPI peripheral of STM32f4xx family of MCUs may be different
 * compared to number of registers of SPI peripherals of STM32Lx or STM32F0x family of MCUs
 * check your device ref manual
 */
 /*
  * if the pointer UART1 is initialized to 0x40011000, here’s the memory layout:

Address  	Value	Description
0x40011000	0x01	CR register
0x40011004	0x02	SR register
0x40011008	0x55	DR register
  */
// to start with creating the peripheral register structure refer the register map of that peripheral not the individual registers

typedef struct                // when a pointer to this type of tructure is created and the pointer is initialized with the base addess of the GPIOx
{                             // then when pointer->OTYPER it means *(pointer + 0x04) it means you are accessing the memory location base+offset
	                          // by writing pointer -> IDR where the offset depends on the the size of members of the structure
	// this structure will impose an entity on the memory (here memory mapped peripherals)
	volatile uint32_t MODER;           // provide description of all the registers depending on the datasheet
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}GPIO_RegDef_t;

typedef struct // PERIPHERAL REGISTER SRUCTURE FOR RCC
{
	volatile  uint32_t RCC_CR;
	volatile  uint32_t RCC_PLLCFGR;
	volatile  uint32_t RCC_CFGR;
	volatile  uint32_t RCC_CIR;
	volatile  uint32_t RCC_AHB1RSTR;
	volatile  uint32_t RCC_AHB2RSTR;
	volatile  uint32_t RCC_AHB3RSTR;
	uint32_t resreved1;
	volatile  uint32_t  RCC_APB1RSTR;
	volatile  uint32_t  RCC_APB2RSTR;
	uint32_t resreved2;
	uint32_t resreved3;
	volatile uint32_t RCC_AHB1ENR;
	volatile uint32_t RCC_AHB2ENR;
	volatile uint32_t RCC_AHB3ENR;
	uint32_t reserved4;
	volatile uint32_t RCC_APB1ENR;
	volatile uint32_t RCC_APB2ENR;
	uint32_t reserved5;
	uint32_t reserved6;
	volatile uint32_t RCC_AHB1LPENR;
	volatile uint32_t RCC_AHB2LPENR;
	volatile uint32_t RCC_AHB3LEPNR;
	uint32_t reserved7;
	volatile uint32_t RCC_APB1LPENR;
	volatile uint32_t RCC_APB2LPENR;
	uint32_t reserved8;
	uint32_t reserved9;
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
	uint32_t reserved10;
	uint32_t reserved11;
	volatile uint32_t RCC_SSCGR;
	volatile uint32_t RCC_PLLI2SCFGR;
}RCC_RegDef_t;
/*
 * peripherals definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

typedef struct
{
	uint32_t reserved1;
	uint32_t reserved2;
	volatile uint32_t SYSCFG_EXTICR[4]; // EXTICR 1,2,3,4 REGISTERS

}SYSCFG_RegDef_t; // apointer to this structure when derefereced can be used to access the SYSCFG block registers

typedef struct
{
	volatile uint32_t EXTI_IMR;
	volatile uint32_t EXTI_EMR;
	volatile uint32_t EXTI_RTSR;
	volatile uint32_t EXTI_FTSR;
	volatile uint32_t EXTI_SWIER;
	volatile uint32_t EXTI_PR;
}EXTI_RegDef_t;

typedef struct
{
	volatile uint32_t USART_SR;
	volatile uint32_t USART_DR;
	volatile uint32_t USART_BRR;
	volatile uint32_t USART_CR1;
	volatile uint32_t USART_CR2;
	volatile uint32_t USART_CR3;
	volatile uint32_t USART_GTPR;
}USART_RegDef_t;

typedef struct
{
  volatile uint32_t SPI_CR1;
  volatile uint32_t SPI_CR2;
  volatile uint32_t SPI_SR;
  volatile uint32_t SPI_DR;
  volatile uint32_t SPI_CRCPR;
  volatile uint32_t SPI_RXCRCR;
  volatile uint32_t SPI_TXCRCR;
  volatile uint32_t SPI_I2SCFGR;
  volatile uint32_t SPI_I2SPR;

}SPI_RegDef_t;
#define GPIOA  ((GPIO_RegDef_t*)  GPIOA_BASEADDR) // when GPIO_RegDef_t* pGPIOA = GPIOA it creates a pointer of type structure and the address it holds is base address of GPIOA peripheral register structure
#define GPIOB  ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define RCC    ((RCC_RegDef_t*) RCC_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define USART1 ((USART_RegDef_t*) USART1_BASEADDR)
#define USART6 ((USART_RegDef_t*) USART6_BASEADDR)
#define USART2 ((USART_RegDef_t*) USART2_BASEADDR)
#define USART3 ((USART_RegDef_t*) USART3_BASEADDR)
#define UART4 ((USART_RegDef_t*) UART4_BASEADDR)
#define UART5 ((USART_RegDef_t*) UART5_BASEADDR)
#define SPI1 ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*) SPI3_BASEADDR)




// clock enable macro for GPIOx peripherals
#define GPIOA_PCLK_EN()  (RCC->RCC_AHB1ENR|= (1<<0))
#define GPIOB_PCLK_EN()  (RCC->RCC_AHB1ENR|= (1<<1))
#define GPIOC_PCLK_EN()  (RCC->RCC_AHB1ENR|= (1<<2))
#define GPIOD_PCLK_EN()  (RCC->RCC_AHB1ENR|= (1<<3))
#define GPIOE_PCLK_EN()  (RCC->RCC_AHB1ENR|= (1<<4))
#define GPIOF_PCLK_EN()  (RCC->RCC_AHB1ENR|= (1<<5))
#define GPIOG_PCLK_EN()  (RCC->RCC_AHB1ENR|= (1<<6))
#define GPIOH_PCLK_EN()  (RCC->RCC_AHB1ENR|= (1<<7))
#define GPIOI_PCLK_EN()  (RCC->RCC_AHB1ENR|= (1<<8))

//clock disable macro for GPIOx peripherals
#define GPIOA_PCLK_DI()  (RCC->RCC_AHB1ENR&= ~(1<<0))
#define GPIOB_PCLK_DI()  (RCC->RCC_AHB1ENR&= ~(1<<1))
#define GPIOC_PCLK_DI()  (RCC->RCC_AHB1ENR&= ~(1<<2))
#define GPIOD_PCLK_DI()  (RCC->RCC_AHB1ENR&= ~(1<<3))
#define GPIOE_PCLK_DI()  (RCC->RCC_AHB1ENR&= ~(1<<4))
#define GPIOF_PCLK_DI()  (RCC->RCC_AHB1ENR&= ~(1<<5))
#define GPIOG_PCLK_DI()  (RCC->RCC_AHB1ENR&= ~(1<<6))
#define GPIOH_PCLK_DI()  (RCC->RCC_AHB1ENR&= ~(1<<7))
#define GPIOI_PCLK_DI()  (RCC->RCC_AHB1ENR&= ~(1<<8))


//enable clock for SYSCFG block
#define SYSCFG_PCLK_EN() (RCC->RCC_APB2ENR|=(1<<14)) // enable clock for SYSCFG

// clock enable macro for I2C PERIPHERALS hanging on APB1 bus

#define I2C1_PCLK_EN() (RCC->RCC_APB1ENR|= (1<<21))
#define I2C2_PCLK_EN() (RCC->RCC_APB1ENR|= (1<<22))
#define I2C3_PCLK_EN() (RCC->RCC_APB1ENR|= (1<<23))

//clock enable macro for SPI peripheral hanging on APB2 and APB1 bus
#define SPI1_PCLK_EN() ((RCC->RCC_APB2ENR)|=(1<<12))
#define SPI2_PCLK_EN() ((RCC->RCC_APB1ENR)|=(1<<14))
#define SPI3_PCLK_EN() ((RCC->RCC_APB1ENR)|=(1<<15))

//clock enable for USART UART
#define USART1_PCLK_EN ((RCC->RCC_APB2ENR)|=(1<<4))
#define USART6_PCLK_EN ((RCC->RCC_APB2ENR)|=(1<<5))
#define USART2_PCLK_EN ((RCC->RCC_APB1ENR)|=(1<<17))
#define USART3_PCLK_EN ((RCC->RCC_APB1ENR)|=(1<<18))
#define UART4_PCLK_EN ((RCC->RCC_APB1ENR)|=(1<<19))
#define UART5_PCLK_EN ((RCC->RCC_APB1ENR)|=(1<<20))

//clock disable macro for USART UART
#define USART1_PCLK_DI ((RCC->RCC_APB2ENR)&=~(1<<4))
#define USART6_PCLK_DI ((RCC->RCC_APB2ENR)&=~(1<<5))
#define USART2_PCLK_DI ((RCC->RCC_APB1ENR)&=~(1<<17))
#define USART3_PCLK_DI ((RCC->RCC_APB1ENR)&=~(1<<18))
#define UART4_PCLK_DI ((RCC->RCC_APB1ENR)&=~(1<<19))
#define UART5_PCLK_DI ((RCC->RCC_APB1ENR)&=~(1<<20))

//CLOCK DISABLE MACRO FOR SPI
#define SPI1_PCLK_DI() ((RCC->RCC_APB2ENR)&=~(1<<12))
#define SPI2_PCLK_DI() ((RCC->RCC_APB1ENR)&=~(1<<14))
#define SPI3_PCLK_DI() ((RCC->RCC_APB1ENR)&=~(1<<15))

//CLOCK DISABLE MACRO FOR ALL PERIPHALS


/*
uint32_t GPIO_baseaddr_to_code(GPIO_RegDef_t* m)
{
	uint32_t k;

	if( m== GPIOA)
				k=PA;
			else if(m == GPIOB)
				k=PB;
			else if(m == GPIOC)
				k=PC;
			else if(m == GPIOD)
				k=PD;
			else if(m == GPIOE)
				k=PE;
			else if(m == GPIOF)
				k=PF;
			else if(m == GPIOG)
				k=PG;
			else if(m == GPIOH)
				k=PH;
			else if(m == GPIOI)
				k=PI;
	return(k);
}

*/

#endif /* INC_STM32F407XX_H_ */
