#include "spi.h"

void SPI5_Init(void)
{
	GPIO_InitTypeDef    gpio;
  SPI_InitTypeDef     spi5;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_OUT;	
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_8;
	gpio.GPIO_Mode = GPIO_Mode_AF;	
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &gpio);
	
	GPIO_PinAFConfig(GPIOF , GPIO_PinSource7 , GPIO_AF_SPI5);
	GPIO_PinAFConfig(GPIOF , GPIO_PinSource9 , GPIO_AF_SPI5);
	GPIO_PinAFConfig(GPIOF , GPIO_PinSource8 , GPIO_AF_SPI5);
	
	/////////
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, DISABLE);
	
	spi5.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi5.SPI_Mode = SPI_Mode_Master;
	spi5.SPI_DataSize = SPI_DataSize_8b;
	spi5.SPI_CPOL = SPI_CPOL_High;
	spi5.SPI_CPHA = SPI_CPHA_2Edge;
	spi5.SPI_NSS  = SPI_NSS_Soft;
	spi5.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	spi5.SPI_FirstBit = SPI_FirstBit_MSB;
	spi5.SPI_CRCPolynomial = 10;
	SPI_Init(SPI5, &spi5);
	SPI_Cmd(SPI5 , ENABLE);
}

u8 SPI5_ReadWriteByte(u8 TxData)
{
	while(SPI_I2S_GetFlagStatus(SPI5 , SPI_I2S_FLAG_TXE) == RESET){}

	SPI_I2S_SendData(SPI5 , TxData);
		
	while(SPI_I2S_GetFlagStatus(SPI5 , SPI_I2S_FLAG_RXNE) == RESET){}

	return SPI_I2S_ReceiveData(SPI5);
}

void ImuSPI5_ReadData(uint8_t address, uint8_t* pdat , uint16_t dataLength)
{
  GPIO_ResetBits(GPIOF , GPIO_Pin_6); 

	address = address | 0x80;
	
	SPI5_ReadWriteByte(address);
	while(dataLength--)
	{
		*pdat = SPI5_ReadWriteByte(0xff);
		pdat++;
	}
  GPIO_SetBits(GPIOF , GPIO_Pin_6); 
}









