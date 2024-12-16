/*
 * 001led_toggle.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Nirvan
 */

#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"


void delay(){
	for(uint32_t i = 0; i < 500000; i++);
}
int main(void){
	GPIO_Handle_t Gpio_Led;

	Gpio_Led.pGPIOx = GPIOB;
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	Gpio_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//	Gpio_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	Gpio_Led.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NOPUPD;

	GPIO_PCLKControl(GPIOB, ENABLE);
	GPIO_Init(&Gpio_Led);

	while(1){
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_9);
		delay();
	}




	return 0;
}
