/*
 * 001_led_toggle_check.c
 *
 *  Created on: 24-Nov-2024
 *      Author: vibhu
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

volatile int m = 500000;
void delay()
{
	for(int i=0;i<m;i++)
	{



	}
}
int main(void)
{
	GPIO_Handle_t GPIOHandle;
	GPIOHandle.pGPIOx = GPIOD;
	GPIOHandle.GPIO_pin.GPIO_PinNumber = 12;
	GPIOHandle.GPIO_pin.GPIO_PinMode = OUTPUT;
	GPIOHandle.GPIO_pin.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	GPIOHandle.GPIO_pin.GPIO_PinPUPD= GPIO_NO_PUPD;
	GPIOHandle.GPIO_pin.GPIO_PinSpeed =GPIO_SPEED_LOW;



	GPIO_Peripehral_clk_control(GPIOHandle.pGPIOx,ENABLE);
	GPIO_init(&GPIOHandle);
  while(1)
  {
	  m=m/4;
	  delay();

	  GPIO_ToggleOutputPin(GPIOD,12);
	  if(m<=10000)
		  m=500000;

  }
}
