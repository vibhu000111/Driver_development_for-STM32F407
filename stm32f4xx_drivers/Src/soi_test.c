/*
 * soi_test.c
 *
 *  Created on: 12-Jan-2025
 *      Author: vibhu
 */
/*
 * for SPI2
 * ALT function mode 5
 * PB14 --> MISO
 * PB15 --> MOSI
 * PB13 --> SCLK
 * PB12 --> NSS
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_SPI_driver.h"
#include "string.h"
void delay()
{
	for(int i=0;i<=50000;i++)
	{

	}
}
void EXTI0_IRQHandler()
{
	delay();
	char user_data[] = "Hi Linkedin";
	uint8_t len = strlen(user_data);
	SPI_EnDi(SPI2 , ENABLE);
	// fist send length info
	SPI_SendData(SPI2,&len,1);
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));


	while(SPI_GetStatusFLAG(SPI2)==1);



	GPIO_IRQHandling(0);
	uint32_t* pNVIC_ICPR0=(uint32_t*)(0XE000E280);
			*pNVIC_ICPR0|=(1<<6);
			SPI_EnDi(SPI2 , DISABLE);

}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIpins;

	SPIpins.pGPIOx = GPIOB;
	SPIpins.GPIO_pin.GPIO_PinMode = ALTFN;
	SPIpins.GPIO_pin.GPIO_PinAltFunMode = 5;
	SPIpins.GPIO_pin.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIpins.GPIO_pin.GPIO_PinPUPD = GPIO_NO_PUPD;
	SPIpins.GPIO_pin.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCLK
	SPIpins.GPIO_pin.GPIO_PinNumber=13;
	GPIO_init(&SPIpins);

	//SPIpins.GPIO_pin.GPIO_PinNumber=14;
	//GPIO_init(&SPIpins);

	SPIpins.GPIO_pin.GPIO_PinNumber=15;
	GPIO_init(&SPIpins);
	SPIpins.GPIO_pin.GPIO_PinNumber=12;
	GPIO_init(&SPIpins);
	GPIO_Handle_t pGPIOA ;
	pGPIOA.pGPIOx=GPIOA;
	pGPIOA.GPIO_pin.GPIO_PinNumber=0;
	pGPIOA.GPIO_pin.GPIO_PinMode=GPIO_MODE_RT;
	pGPIOA.GPIO_pin.GPIO_PinOPType=0;
	pGPIOA.GPIO_pin.GPIO_PinPUPD= GPIO_NO_PUPD;
	pGPIOA.GPIO_pin.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	GPIO_init(&pGPIOA);


}

void SPI2_Inits()
{
	SPI_Handle_t SPI2_init;
	SPI2_init.pSPIx = SPI2;
	SPI2_init.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_init.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_init.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2_init.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2_init.SPIConfig.SPI_CPHA = SPI_CPHA_LEADING_EDGE;
	SPI2_init.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_init.SPIConfig.SPI_SSM = SPI_SSM_DI;//Hardware slave mgmt

	SPI_init(&(SPI2_init));

}
int main()
{
	SPI2_GPIOInits();
	SPI2_Inits();
	GPIO_IRQConfig(6,ENABLE);



	while(1)
	{


	}
return(0);


}
