#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include <stm32f407xx.h> //the driver header file should contain the MCU specific header file






typedef struct
{
	uint8_t SPI_DeviceMode; // SPI master or Slave
	uint8_t SPI_BusConfig; // half duplex , simplex or full duplex
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF; // 8 bit or 16 bit of shift register
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t *pSPIx; // this will hold the base address of SPIx(x = 1,2,3) peripheral
	SPI_Config_t SPIConfig; // structure holding all the configurable items for SPI

}SPI_Handle_t;




























#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
