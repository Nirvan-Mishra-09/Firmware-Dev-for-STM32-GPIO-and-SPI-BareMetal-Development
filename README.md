# Firmware-Dev-for-STM32-GPIO-aand-SPI-bareMetal-Programming


# STM32F4 LED Blink - Bare Metal Implementation

A bare metal implementation of LED blinking for STM32F4 microcontroller using bit-field structures. This project demonstrates register-level programming without using HAL or any external frameworks.

## Project Overview

This project implements a simple LED blinking mechanism using structured bit-field approach for register access. The implementation focuses on clean, maintainable code using proper C structures for register manipulation.

## Hardware Requirements

- STM32F4 Discovery Board
- Onboard LED connected to PA5

## Code Structure

### Register Definitions
The project uses structured bit-fields for clear register access:

```c
typedef struct{
    uint32_t gpioa_en:1;
    uint32_t gpiob_en:1;
    uint32_t gpioc_en:1;
    // ... other bits
} RCC_AHB1ENR_t;

typedef struct{
    uint32_t pin_0:2;
    uint32_t pin_1:2;
    // ... other pins
    uint32_t pin_5:2;
    // ... remaining pins
} GPIOx_MODER_t;

typedef struct{
    uint32_t pin_0:1;
    uint32_t pin_1:1;
    // ... other pins
    uint32_t pin_5:1;
    // ... remaining pins
    uint32_t reserved:16;
} GPIOx_ODR_t;
```

### Memory Map
```c
RCC_AHB1ENR:    0x40023830
GPIOA_MODER:    0x40020000
GPIOA_ODR:      0x40020014
```

## Implementation Details

1. **Clock Configuration**
   ```c
   RCC_AHB1ENR_t volatile *const pClkCtrlReg = (RCC_AHB1ENR_t*)0x40023830;
   pClkCtrlReg->gpioa_en = 1;
   ```

2. **GPIO Configuration**
   ```c
   GPIOx_MODER_t volatile *const pPortAModeReg = (GPIOx_MODER_t*)0x40020000;
   pPortAModeReg->pin_5 = 1;  // Set as output
   ```

3. **LED Control**
   ```c
   GPIOx_ODR_t volatile *const pGPIOAoutReg = (GPIOx_ODR_t*)0x40020014;
   pGPIOAoutReg->pin_5 = 1;  // LED ON
   pGPIOAoutReg->pin_5 = 0;  // LED OFF
   ```

## Key Features

1. **Structured Register Access**
   - Uses bit-fields for clear register manipulation
   - Type-safe register access
   - Self-documenting code structure

2. **Direct Register Manipulation**
   - No HAL dependencies
   - Bare metal implementation
   - Efficient code execution



3. **Const Correctness**
   - Uses const pointers for register addresses
   - Prevents accidental address modification

4. **Volatile Correctness**
   - Proper volatile declarations for hardware registers
   - Prevents compiler optimization issues

https://github.com/user-attachments/assets/725d7c5c-bd8a-4f6f-87c3-cb042c14c8c8

https://github.com/user-attachments/assets/f3411308-0a1c-41a3-b81f-989eb9ccb46a

