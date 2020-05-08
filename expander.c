/*
 * expander.c
 *
 *  Created on: Mar 31, 2020
 *      Author: Arek
 */

#include "expander.h"

static void mcp_write_reg(uint8_t addr,uint8_t value, SPI_HandleTypeDef * spi)
{
 uint8_t tx_buf[] = { ADDRESS, addr, value };

 HAL_GPIO_WritePin(CS_PORT, CS_Pin, GPIO_PIN_RESET);
 HAL_SPI_Transmit(spi, tx_buf, 3, HAL_MAX_DELAY);
 HAL_GPIO_WritePin(CS_PORT, CS_Pin, GPIO_PIN_SET);
}

static uint8_t mpc_read_reg(SPI_HandleTypeDef * spi, uint8_t reg)
{
	uint8_t tx_buf[] = { 0x41, reg, 0xff };
	uint8_t rx_buf[3];

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi, tx_buf, rx_buf, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

	return rx_buf[2];
}

//------------------------------------------------------------------------------

void MCP_Init(SPI_HandleTypeDef * spi, MCP_HandleTypeDef * mcp)
{

	mcp_write_reg(IODIR, mcp->Direction, spi);
	mcp_write_reg(GPPU, mcp->GPIO_PU_resistors, spi);
	mcp_write_reg(OLAT, mcp->Output_latch, spi);
}


void MCP_SetPinAsOutput(SPI_HandleTypeDef * spi, uint8_t pin)
{
	uint8_t RegisterDirection;	//value from MCP
	RegisterDirection = mpc_read_reg(spi, IODIR);
	uint8_t buff = 0xff;
	buff &= ~pin;

	RegisterDirection &= buff;

	mcp_write_reg(IODIR, RegisterDirection, spi);

}

//do not finished
void MCP_SetPinAsInput(SPI_HandleTypeDef * spi, uint8_t pin)
{
	//change register direction
	uint8_t DirectionReg;	//value from MCP
	DirectionReg = mpc_read_reg(spi, IODIR);

	DirectionReg |= pin;

	mcp_write_reg(IODIR, DirectionReg, spi);


	//change register GPIO PullUp resistors
	uint8_t PullUpReg;	//value from MCP
	PullUpReg = mpc_read_reg(spi, GPPU);
	PullUpReg |= pin;
	mcp_write_reg(GPPU, PullUpReg, spi);
}


void MCP_WritePort(SPI_HandleTypeDef * spi, uint8_t port)
{
	mcp_write_reg(OLAT, port, spi);
}

uint8_t MCP_ReadPort(SPI_HandleTypeDef * spi)
{
 uint8_t tx_buf[] = { 0x41, GPIO, 0xff };
 uint8_t rx_buf[3];

 HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
 HAL_SPI_TransmitReceive(spi, tx_buf, rx_buf, 3, HAL_MAX_DELAY);
 HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

 return rx_buf[2];
}

void MCP_WritePin(SPI_HandleTypeDef * spi, uint8_t pin, bool state)
{
	uint8_t PortState = MCP_ReadPort(spi);
	if(state == 1){
		PortState |= pin;
	}
	else{
		PortState &= ~pin;
	}

	mcp_write_reg(OLAT, PortState, spi);
}

bool MCP_ReadPin(SPI_HandleTypeDef * spi, uint8_t pin)
{
	uint8_t PortState = MCP_ReadPort(spi);
	uint8_t buff = PortState;
	if((buff &= ~pin) == PortState)
		return 0;
	else
		return 1;

}






