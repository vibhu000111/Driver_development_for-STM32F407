/*
 * 002_LED_toggling_Open_Drain.c
 *
 *  Created on: 24-Nov-2024
 *      Author: vibhu
 */

/*
 * Sample application checks driver for GPIO_Open_Drain_Configuration_toggle
 */
#include <stdint.h>
#include <stm32f407xx.h>
#include <stm32f407xx_gpio_driver.h>

void delay ()
{
	for(int i=0;i<250000;i++)
	{

	}
}
int main()
{
	GPIO_Handle_t GPIOpd;
	GPIOpd.pGPIOx = GPIOD;
	GPIOpd.GPIO_pin.GPIO_PinNumber = 12;
	GPIOpd.GPIO_pin.GPIO_PinMode = OUTPUT;
	GPIOpd.GPIO_pin.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOpd.GPIO_pin.GPIO_PinPUPD = GPIO_PU ; //as to PD12 already a resistor is connected to LED
	GPIOpd.GPIO_pin.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Peripehral_clk_control(GPIOD,ENABLE);
	GPIO_init(&GPIOpd);

	while(1)
	{
		delay();
		GPIO_ToggleOutputPin(GPIOD,12);
	}

}
