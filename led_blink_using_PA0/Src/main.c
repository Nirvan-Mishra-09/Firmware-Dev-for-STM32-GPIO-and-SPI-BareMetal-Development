
#include <stdint.h>
int main(void)
{

	uint32_t volatile *pClkCtrlReg = 		(uint32_t*)0x40023830;
	uint32_t volatile *pPortAModeReg_PA5 = 	(uint32_t*)0x40020000;
	uint32_t volatile *pPortAModeReg_PA0 = 	(uint32_t*)0x40020000;
	uint32_t volatile *pGPIOAoutReg = 		(uint32_t*)0x40020014;
	uint32_t volatile *pGPIOAInReg = 		(uint32_t*)0x40020010;
	//1. enabling the GPIOA clock, to set GPIOAEN as 1 in RCC_AHB1ENR

	*pClkCtrlReg = *pClkCtrlReg | 0x01;

	//2. COnfigure Mode
	//a. clear the 11th and 10th bit for 5th pin
//	*pPortAModeReg &= 0xFFFFF3FF; // CLEAR
	*pPortAModeReg_PA5 &= ~(3<<10);
	//b. make 10th bit as 1
//	*pPortAModeReg |= 0x00000400;	//SET
	*pPortAModeReg_PA5 |= (1<<10);


	// Configure mode for PA0
	*pPortAModeReg_PA0 &= ~(3<<0);
	// Now reading from pin PA0
	while(1){
		uint8_t status = (uint8_t)(*pGPIOAInReg & 0x1);

		if(status){
			*pGPIOAoutReg |= (1 << 5); //Turn ON
		}
		else{
			*pGPIOAoutReg &= ~(1<<5); //Turn OFF
		}

	}




}
