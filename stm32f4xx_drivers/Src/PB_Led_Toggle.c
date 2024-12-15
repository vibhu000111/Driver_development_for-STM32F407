/*
 * PB_Led_Toggle.c
 *
 *  Created on: 25-Nov-2024
 *      Author: vibhu
 */
#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>

void EXTI0_IRQHandler()
{
	GPIO_ToggleOutputPin(GPIOD ,12);
	//uint32_t* pEXTI_PR=(uint32_t*)(EXTI_BASEADDR + 0x14);
		//	*pEXTI_PR|=(1<<0);


	GPIO_IRQHandling(0);
	uint32_t* pNVIC_ICPR0=(uint32_t*)(0XE000E280);
			*pNVIC_ICPR0|=(1<<6);
}
//PA0 for input
//PD12 for toggle
int main()
{

	//configure PD12 for output
	GPIO_Handle_t pGPIOD ;
	pGPIOD.pGPIOx=GPIOD;
	pGPIOD.GPIO_pin.GPIO_PinNumber=12;
	pGPIOD.GPIO_pin.GPIO_PinMode=OUTPUT;
	pGPIOD.GPIO_pin.GPIO_PinOPType=0;
	pGPIOD.GPIO_pin.GPIO_PinPUPD= GPIO_NO_PUPD;
	pGPIOD.GPIO_pin.GPIO_PinSpeed=GPIO_SPEED_LOW;

	//configure PA0 for input
	GPIO_Handle_t pGPIOA ;
	pGPIOA.pGPIOx=GPIOA;
		pGPIOA.GPIO_pin.GPIO_PinNumber=0;
		pGPIOA.GPIO_pin.GPIO_PinMode=GPIO_MODE_RT;
		pGPIOA.GPIO_pin.GPIO_PinOPType=0;
		pGPIOA.GPIO_pin.GPIO_PinPUPD= GPIO_NO_PUPD;
		pGPIOA.GPIO_pin.GPIO_PinSpeed=GPIO_SPEED_HIGH;

	GPIO_Peripehral_clk_control(GPIOA,ENABLE);
	GPIO_Peripehral_clk_control(GPIOD,ENABLE);

	GPIO_init(&pGPIOD);
	GPIO_init(&pGPIOA);


	//configuring EXTI0
	/*RCC->RCC_APB2ENR|=(1<<14); //enabling clock for RCC for SYSCFG peripheral
	uint32_t* pSYSCFG_EXTICR1=(uint32_t*)(SYSCFG + 0x08);
	*pSYSCFG_EXTICR1&=~(15<<0); //now PA0 can trigger interrupt on EXTIO interrupt line that is IRQ number
	uint32_t* pEXTI_RT = (uint32_t*)(EXTI_BASEADDR + 0x08);
	*pEXTI_RT|=(1<<0);
	uint32_t* pEXTI_IMR = (uint32_t*)(EXTI_BASEADDR);
	*pEXTI_IMR|=(1<<0);*/
	//uint32_t* pNVIC_ISER0 = (uint32_t*) 0xE000E100;
	//*pNVIC_ISER0|=(1<<6); //enable Interrput on IRQ6 for EXTI0
	GPIO_IRQConfig(6,ENABLE);

	while(1)
	{

	}


}

