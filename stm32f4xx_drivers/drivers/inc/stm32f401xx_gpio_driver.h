/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: Nov 23, 2024
 *      Author: Nirvan
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_
#include <stdint.h>
#include "stm32f401xx.h"

/*
 * GPIO Pin configuration structure
 * */
typedef struct{

	uint8_t GPIO_PinNumber;		/*@GPIO_PinNumber*/
	uint8_t GPIO_PinMode;		/*<@GPIO_PinMode>*/
	uint8_t GPIO_PinSpeed;		/*<@GPIO_PinSpeed>*/
	uint8_t GPIO_PuPdControl;	/*<@GPIO_PuPdControl>*/
	uint8_t GPIO_PinOPType;		/*<@GPIO_PinOPType>*/
	uint8_t	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 * */

typedef struct{
	//1. pointer to hold the base address of the gpio peripherals
	GPIO_RegDef_t 		*pGPIOx; // pointer to the registers in the GPIO_RegDef_t
	GPIO_PinConfig_t 	GPIO_PinConfig; // from the user
}GPIO_Handle_t;

// Possible Values of @GPIO_PinNumber
#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15

// Possible Values of @GPIO_PinMode
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

// Possible values of @GPIO_PinSpeed
#define GPIO_LOW_SPEED 			0
#define GPIO_MEDIUM_SPEED 		1
#define GPIO_HIGH_SPEED 		2
#define GPIO_VERY_HIGH_SPEED 	3

// Possible values of @GPIO_PuPdControl
#define GPIO_NOPUPD 0
#define GPIO_PU		1
#define GPIO_PD		2

// Possible values of @GPIO_PinOPType
#define GPIO_OP_TYPE_PP	0
#define GPIO_OP_TYPE_OD	1


/*
 * Peripheral Clock setup
 * */
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO init and deinit
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read write
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Interrupt handling
 * */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
