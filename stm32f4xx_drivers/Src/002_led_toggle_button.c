/*
 * 002_led_toggle_button.c
 *
 *  Created on: Nov 24, 2024
 *      Author: Nirvan
 */


#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

#define LOW 		1
#define BTN_PRESSED LOW
void delay(){
	for(uint32_t i = 0; i < 5000000/2; i++);
}

int main(void){

	GPIO_Handle_t Gpio_button, Gpio_Led;

	// this for led configuration
	Gpio_Led.pGPIOx = GPIOB;
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	Gpio_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpio_Led.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NOPUPD;

	GPIO_PCLKControl(GPIOB, ENABLE);
	GPIO_Init(&Gpio_Led);

	//this for button configuration
	Gpio_button.pGPIOx = GPIOA;
	Gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	Gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	Gpio_button.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PU;


	GPIO_PCLKControl(GPIOA, ENABLE);
	GPIO_Init(&Gpio_button);


	/*we need to read from the input pin PC13 i.e., where the button is connected, if yes
	 the toggle the led*/
	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_6) == BTN_PRESSED){
			delay();
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_9);
		}
	}
	return 0;




}
