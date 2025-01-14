


#include <stm32f407xx_SPI_driver.h>


void SPI_Peripehral_clk_control(SPI_RegDef_t *pSPIx,uint8_t EnDi)
{
	if (EnDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();

		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();

		}

	}
	else
	{
		if(EnDi == DISABLE)
		{
			if(pSPIx == SPI1)
					{
						SPI1_PCLK_DI();
					}
					else if(pSPIx == SPI2)
					{
						SPI2_PCLK_DI();

					}
					else if(pSPIx == SPI3)
					{
						SPI3_PCLK_DI();

					}
		}
	}
}

//Init or deinit the peripheral

void SPI_init(SPI_Handle_t *pSPIHandle)
{
	SPI_Peripehral_clk_control(pSPIHandle->pSPIx,ENABLE);
	uint32_t tempreg=0;

	//1. configure the device mode
	tempreg|=(pSPIHandle->SPIConfig.SPI_DeviceMode<<2);
	//2. Configure the busconfig mode
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//2-line unidirectional data mode has to be selected
		tempreg&=~(1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//1-line bidirectional data mode has to be selected
		tempreg|=(1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//2-line unidirectional mode is selected
		tempreg&=~(1<<15);
		//RXONLY bit must be set
		tempreg|=(1<<10);
	}

	//3. CONFIGURE CLOCK SPEED
	tempreg|=(pSPIHandle->SPIConfig.SPI_SclkSpeed << 3);
	// 4. Configure Data frame format
	tempreg|=(pSPIHandle->SPIConfig.SPI_DFF << 11);
	// 5. Configure clock polarity
	tempreg|=(pSPIHandle->SPIConfig.SPI_CPOL << 1);
	// configure clock phase
	tempreg|=(pSPIHandle->SPIConfig.SPI_CPHA << 0);
	// configure SSM
	tempreg|=(pSPIHandle->SPIConfig.SPI_SSM << 9);

	if(pSPIHandle->SPIConfig.SPI_SSM == 1) // is SSM is selected SSI bit has to be set to 1 otherwise MODF fault occurs
	{
		tempreg|=(1<<8);
	}

	pSPIHandle->pSPIx->SPI_CR1 = tempreg; // pSPIX->SPI_CR1 means derefrencing the base address of SPI + offset to access specific register
    if(pSPIHandle->SPIConfig.SPI_SSM == 0)
    {
    	pSPIHandle->pSPIx->SPI_CR2|=(1<<2);

    }

}
void SPI_deinit(SPI_Handle_t *pSPIHandle);

//Data send or receive
// in communication peripherals
// 3 types of data transmission methods
// blocking , interrupt , DMA based

void SPI_SendData(SPI_RegDef_t* pSPIx , uint8_t *pTxBuffer , uint32_t length) // length dhould always be uint32 , this code is polling based
{
	uint32_t temp = length;
	uint16_t *pTx_Buffer;
	uint32_t DFF =( (pSPIx->SPI_CR1) & 1<<11);
	if (DFF==1)
	{
		pTx_Buffer = (uint16_t*)pTxBuffer;

	}
	while (temp>0)
	{
		//uint32_t txe_status= (pSPIx->SPI_SR & (1<<1));
		while((pSPIx->SPI_SR & (1<<1))==0)
		{
		}
		if(DFF == 1)
		{
			pSPIx->SPI_DR = *(pTx_Buffer);// accesses two locations at a time
			temp--;
			temp--;
			pTx_Buffer++; // points to location ahead as 16 bit pointer
		}
		else
		{
			pSPIx->SPI_DR = *(pTxBuffer);
			temp--;
			pTxBuffer++;
		}
		}
	}
void SPI_ReceiveData(SPI_RegDef_t* pSPIx,uint8_t *pRxBuffer , uint32_t length)
{
	uint32_t temp= length;
	uint16_t *pRx_Buffer = (uint16_t*)pRxBuffer;
	uint32_t DFF =( (pSPIx->SPI_CR1) & 1<<11);
	while(temp>0)
	{

		while( (pSPIx->SPI_SR & (1<<0)) == 0);

		if(DFF == 1)
		{
			*pRx_Buffer = pSPIx->SPI_DR; // read data register
			temp--;
			temp--;
			pRx_Buffer++;
		}
		else
		{
			*pRxBuffer = pSPIx->SPI_DR;
			temp--;
			pRxBuffer++;
		}


	}
}

void SPI_EnDi(SPI_RegDef_t *pSPIx , uint8_t EnDi)
{
	if(EnDi == ENABLE)
  pSPIx->SPI_CR1|=(1<<6);

	else if(EnDi ==DISABLE)
	{
		pSPIx->SPI_CR1&=~(1<<6);
	}

}
uint8_t SPI_GetStatusFLAG(SPI_RegDef_t* pSPIx)
{
	return((pSPIx->SPI_SR & (1<<7)));
}





void SPI_IRQConfig(uint8_t IRQNumber,uint8_t EnDi);
void SPI_IRQ_PriorityConfig(uint8_t IRQNumber , uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
