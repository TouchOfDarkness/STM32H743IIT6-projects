#include "Bsp_Gt9xx.h"

volatile TouchStructure touchInfo;
I2C_HandleTypeDef * i2cPort;

void Touch_INT_Out(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin   = TP_IRQ_Pin;

	HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);
}

void Touch_INT_In(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin   = TP_IRQ_Pin;

	HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);
}

void GT9XX_Reset(void)
{
	Touch_INT_Out();
	HAL_GPIO_WritePin(TP_IRQ_GPIO_Port, TP_IRQ_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(TP_CS_GPIO_Port,TP_CS_Pin,GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(TP_CS_GPIO_Port,TP_CS_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	Touch_INT_In();
	HAL_Delay(100);
}

uint8_t GT9XX_WriteData (uint16_t addr,uint8_t value)
{
	uint8_t status;

	status = HAL_I2C_Mem_Write(i2cPort, GT9XX_IIC_WADDR, addr, I2C_MEMADD_SIZE_16BIT, &value, 1, 100);

	return status;
}

uint8_t GT9XX_WriteReg (uint16_t addr, uint8_t cnt, uint8_t *value)
{
	uint8_t status;

	status = HAL_I2C_Mem_Write(i2cPort, GT9XX_IIC_WADDR, addr, I2C_MEMADD_SIZE_16BIT, value, cnt, 100);

	return status;
}

uint8_t GT9XX_ReadReg (uint16_t addr, uint8_t cnt, uint8_t *value)
{
	uint8_t status;

	status = HAL_I2C_Mem_Read(i2cPort, GT9XX_IIC_RADDR, addr, I2C_MEMADD_SIZE_16BIT, value, cnt, 100);

	return status;
}

uint8_t Touch_Init(I2C_HandleTypeDef * SetI2cPort)
{
	GT9XX_Reset();
	i2cPort = SetI2cPort;

	uint8_t GT9XX_Info[11];
	uint8_t cfgVersion = 0;

	GT9XX_Reset();

	GT9XX_ReadReg (GT9XX_ID_ADDR,11,GT9XX_Info);
	GT9XX_ReadReg (GT9XX_CFG_ADDR,1,&cfgVersion);

	if( GT9XX_Info[0] == '9' )
	{

		return SUCCESS;
	}
	else
	{
		return ERROR;
	}

}

void Touch_Scan(void)
{
 	uint8_t  touchData[2 + 8 * TOUCH_MAX ];
	uint8_t  i = 0;

	GT9XX_ReadReg (GT9XX_READ_ADDR,2 + 8 * TOUCH_MAX ,touchData);
	GT9XX_WriteData (GT9XX_READ_ADDR,0);
	touchInfo.num = touchData[0] & 0x0f;

	if ( (touchInfo.num >= 1) && (touchInfo.num <=5) )
	{
		for(i=0;i<touchInfo.num;i++)
		{
			touchInfo.y[i] = (touchData[5+8*i]<<8) | touchData[4+8*i];
			touchInfo.x[i] = (touchData[3+8*i]<<8) | touchData[2+8*i];
		}
		touchInfo.flag = 1;
	}
	else
	{
		touchInfo.flag = 0;
	}
}
