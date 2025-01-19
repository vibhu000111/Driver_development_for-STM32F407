/*
 * I2C_master_tx_testing.c
 *
 *  Created on: 19-Jan-2025
 *      Author: vibhu
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f4xx_I2C_driver.h"
#include "string.h"

/*
 * SCL PB6
 * SDA PB9
 */
I2C_handle_t I2C1_init;
void delay()
{
	for(int i=0;i<=50000;i++)
	{

	}
}

void EXTI0_IRQHandler()
{
	delay();
	char user_data[] = "Hi my name is Vibhu";
	uint8_t len = strlen(user_data);
	I2C_MasterSendData(&I2C1_init, (uint8_t*)user_data ,len, 0x68);
	GPIO_IRQHandling(0);
	uint32_t* pNVIC_ICPR0=(uint32_t*)(0XE000E280);
			*pNVIC_ICPR0|=(1<<6);


}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2Cpins;

	I2Cpins.pGPIOx = GPIOB;
	I2Cpins.GPIO_pin.GPIO_PinMode = ALTFN;
	I2Cpins.GPIO_pin.GPIO_PinAltFunMode = 4 ;
	I2Cpins.GPIO_pin.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2Cpins.GPIO_pin.GPIO_PinPUPD = GPIO_NO_PUPD;
	I2Cpins.GPIO_pin.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCL
	I2Cpins.GPIO_pin.GPIO_PinNumber=6;
	GPIO_init(&I2Cpins);

	//SDA
	I2Cpins.GPIO_pin.GPIO_PinNumber=9;
	GPIO_init(&I2Cpins);
	GPIO_Handle_t pGPIOA ;
	pGPIOA.pGPIOx=GPIOA;
	pGPIOA.GPIO_pin.GPIO_PinNumber=0;
	pGPIOA.GPIO_pin.GPIO_PinMode=GPIO_MODE_RT;
	pGPIOA.GPIO_pin.GPIO_PinOPType=0;
	pGPIOA.GPIO_pin.GPIO_PinPUPD= GPIO_NO_PUPD;
	pGPIOA.GPIO_pin.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	GPIO_init(&pGPIOA);


}

void I2C_Inits()
{


	I2C1_init.pI2Cx = I2C1;
	I2C1_init.I2C_config_params.I2C_ACKControl = ENABLE;
	I2C1_init.I2C_config_params.DeviceAddress = 0x61;
	I2C1_init.I2C_config_params.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_init.I2C_config_params.I2C_SCLSpeed = I2C_SCL_SPEED_SM;


	I2C_init(&(I2C1_init));

}
int main()
{
	I2C1_GPIOInits();
	I2C_Inits();
	GPIO_IRQConfig(6,ENABLE);
	I2C_EnDi(I2C1 , ENABLE);







	while(1)
	{


	}
return(0);


}
