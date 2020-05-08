/*
 * Expander.h
 *
 * MCP230083
 *  Created on: 29 Mar 2020
 *      Author: Arek
 *
 *  More information about MCP230083 can be found in datasheet
 */

#ifndef EXPANDER_H
#define EXPANDER_H


#include <stdbool.h>
#include "main.h"



//register addresses

#define IODIR 	0x00
#define IPOL 		0x01
#define GPINTEN 0x02
#define DEFVAL 	0x03
#define INTCON 	0x04
#define IOCON 	0x05
#define GPPU	 	0x06
#define INTF 		0x07	//read only
#define INTCAP 	0x08
#define GPIO 		0x09
#define OLAT 		0x0a

#define CS_PIN 	CS_Pin
#define CS_PORT CS_GPIO_Port


 //MCP address
#define ADDRESS 0x40

//Expander GPIO Pins
#define GPIO_0 0x01
#define GPIO_1 0x02
#define GPIO_2 0x04
#define GPIO_3 0x08
#define GPIO_4 0x10
#define GPIO_5 0x20
#define GPIO_6 0x40
#define GPIO_7 0x80

typedef struct __MCP_HandleTypeDef {

	/* set 1 for input pin
	 * set 0 for output pin
	 * Example:
	 * GPIO0...GPIO7 as OUTPUT -- > Direction = 0x00
	 * GPIO0 as INPUT, GPIO1...GPIO7 as OUTPUT   -- > Direction = 0x01
	*/
	uint8_t Direction;

	// this will be developed
	//uint8_t Input_polarity;
	//uint8_t Interrupt_on_change_pins;
	//uint8_t Default_value_register;
	//uint8_t Interrupt_on_change_control;
	//uint8_t Expander_configuration;

	/* Register controls the pull-up resistors for theport  pins.
	 * If  a  bit  is  set  and  the  corresponding  pin
	 * is configured  as  an  input,  the  corresponding  port  pin
	 * is internally pulled up with a resistor.
	 */
	uint8_t GPIO_PU_resistors;
	//uint8_t Interrupt_captured;

	/* GPIO output register
	 * Register Direction must be set LOW
	 * Example:
	 * GPIO0 == HIGH -- > Output_latch = 0x01
	 */
	uint8_t Output_latch;

} MCP_HandleTypeDef;


void MCP_Init(SPI_HandleTypeDef * spi, MCP_HandleTypeDef * mcp);

void MCP_SetPinAsOutput(SPI_HandleTypeDef * spi, uint8_t pin);
void MCP_SetPinAsInput(SPI_HandleTypeDef * spi, uint8_t pin);

/* Register Direction must be set LOW on bits you want to use as OUTPUTS
 * Example:
 * MCP_WritePort(&spi, 0x01);  - > GPIO0 is HIGH, GPIO1...GPIO7 is LOW
 * MCP_WritePort(&spi, ~0x01); - > All pins are HIGH except GPIO0
 * (GPIO0 - GPIO7)
 */
void MCP_WritePort(SPI_HandleTypeDef * spi, uint8_t port);


/* Register GPIO_PU_resistors must be set HIGH on the read pin
 * Example:
 * For GPIO0 GPIO_PU_resistors = 0x01
 * MCP_ReadPort(&spi) return state for all GPIO pins
 */
uint8_t MCP_ReadPort(SPI_HandleTypeDef * spi);

/* Bit of the register Direction must be set HIGH */
void MCP_WritePin(SPI_HandleTypeDef * spi, uint8_t pin, bool state);

bool MCP_ReadPin(SPI_HandleTypeDef * spi, uint8_t pin);




#endif /* EXPANDER_H_ */
