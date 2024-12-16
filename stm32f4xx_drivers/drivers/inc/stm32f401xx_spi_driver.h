/*
 * stm32f401xx_spi_driver.h
 *
 *  Created on: Dec 14, 2024
 *      Author: Nirvan
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include <stdint.h>
#include "stm32f401xx.h"

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_config_t;

typedef struct{

	SPI_RegDef_t *pSPIx; // pointer to SPI peripheral on the APBx bus, like SPI1, SPI2, SPI3, SPI4
	SPI_config_t SPIConfig; // from user
}SPI_Handle_t;


#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
