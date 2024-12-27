/*
 * main.c
 *
 *  Created on: Dec 27, 2024
 *      Author: Nirvan
 */



#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

int main(void){
	return 0;
}

void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(0);
}
