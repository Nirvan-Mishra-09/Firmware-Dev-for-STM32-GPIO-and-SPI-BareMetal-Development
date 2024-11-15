# Firmware-Dev-for-STM32-GPIO-aand-SPI-bareMetal-Programming


## STM32F4 GPIO LED Toggle Project

A bare metal implementation of GPIO control for STM32F4 microcontroller that toggles an onboard LED based on button input. This project demonstrates direct register manipulation without using any HAL or external libraries.

## Overview

The project implements a simple LED toggle mechanism where:
- The onboard LED (connected to PA5) is toggled based on
- The state of a button (connected to PA0)
- Uses direct register manipulation for GPIO control

## Hardware Setup

- **Board**: STM32F4 Discovery Board
- **LED**: Connected to PA5
- **Button**: Connected to PA0
- **Clock**: GPIOA enabled via RCC_AHB1ENR

## Register Details

The project uses the following memory-mapped registers:

```c
Clock Control: 0x40023830 (RCC_AHB1ENR)
GPIO Mode:     0x40020000 (GPIOA_MODER)
GPIO Output:   0x40020014 (GPIOA_ODR)
GPIO Input:    0x40020010 (GPIOA_IDR)
```

## Implementation Details

1. **Clock Configuration**
   ```c
   // Enable GPIOA clock
   *pClkCtrlReg = *pClkCtrlReg | 0x01;
   ```

2. **GPIO Configuration**
   ```c
   // Configure PA5 as output
   *pPortAModeReg_PA5 &= ~(3<<10);  // Clear mode bits
   *pPortAModeReg_PA5 |= (1<<10);   // Set as output

   // Configure PA0 as input
   *pPortAModeReg_PA0 &= ~(3<<0);   // Set as input
   ```

3. **LED Control Logic**
   ```c
   // Read button state
   uint8_t status = (uint8_t)(*pGPIOAInReg & 0x1);

   // Toggle LED based on button state
   if(status){
       *pGPIOAoutReg |= (1 << 5);  // Turn ON
   }
   else{
       *pGPIOAoutReg &= ~(1<<5);   // Turn OFF
   }
   ```

## Flow Diagram

```
Initialize Clock → Configure GPIO → Enter Loop → Read Button → Toggle LED → Repeat
```

## Features

- Bare metal implementation
- Direct register access
- No external dependencies
- Simple and efficient code
- Real-time LED response to button press

