# Firmware Dev for STM32 GPIO and SPI bareMetal Programming (In Progress)


# STM32 GPIO Driver Development - Bare Metal Programming

A custom GPIO driver implementation for the STM32F401RE Nucleo Board using **bare-metal programming** techniques. This project demonstrates a low-level approach to peripheral control by directly manipulating hardware registers, without relying on external frameworks like HAL or CMSIS.

## Project Overview

This project implements a GPIO driver for the STM32F401RE microcontroller, offering APIs for configuring, reading, and writing to GPIO pins. The implementation emphasizes a clear structure, portability, and efficient hardware interaction.

## Features

- **Pin Configuration**:
  - Supports Input, Output, Alternate Function, and Analog modes.
  - Configurable Pull-Up/Pull-Down resistors.
  - Push-Pull and Open-Drain output types.
  - Slew rate control for pin speed: Low, Medium, High, or Very High speed.

- **APIs**:
  - Enable or disable peripheral clock.
  - Initialize and deinitialize GPIO pins.
  - Read input pin or port state.
  - Write to output pin or port.
  - Toggle output pins.
  - Configure interrupts for GPIO pins and handle ISRs.

- **Bare-Metal Programming**:
  - Direct register manipulation for maximum control and efficiency.
  - Minimal reliance on pre-defined libraries or HAL.
    
## Directory Structure
```
stm32f4xx_drivers\drivers\inc  // Contains all driver header files
stm32f4xx_drivers\drivers\src  // Contains test and implementation files
```

## Hardware Requirements

- STM32F401RE Nucleo Board
- STM32 cube IDE

## Code Structure

### Peripheral Registers

The project uses detailed bit-field structures for register manipulation:

```c
typedef struct {
    uint32_t MODER;    // GPIO mode register
    uint32_t OTYPER;   // GPIO output type register
    uint32_t OSPEEDR;  // GPIO output speed register
    uint32_t PUPDR;    // GPIO pull-up/pull-down register
    uint32_t IDR;      // GPIO input data register
    uint32_t ODR;      // GPIO output data register
    uint32_t BSRR;     // GPIO bit set/reset register
    uint32_t LCKR;     // GPIO configuration lock register
    uint32_t AFR[2];   // GPIO alternate function registers
} GPIO_RegDef_t;
```

### GPIO Pin Modes

| Mode               | Configuration Code |
|--------------------|--------------------|
| Input              | `00`              |
| General Output     | `01`              |
| Alternate Function | `10`              |
| Analog Mode        | `11`              |

### GPIO Output Types

| Output Type | Description            |
|-------------|------------------------|
| Push-Pull   | Standard configuration |
| Open-Drain  | Output pulls to ground |

### APIs Implemented

1. **Clock Control**:
   ```c
       void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
   ```

2. **GPIO Initialization**:
   ```c
      void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
      void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
   ```

3. **Data Read/Write**:
   ```c
        uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
        uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
        void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
        void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
        void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
   ```

4. **Interrupt Configuration**:
   ```c
        void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
        void GPIO_IRQHandling(uint8_t PinNumber);
   ```

## How to Run

1. **Setup**:
   - Use the STM32F401RE Nucleo Board.
   - Connect an LED to GPIO pin PA5 for testing.

2. **Build and Flash**:
   - Use a toolchain like **Keil**, **STM32CubeIDE**, or **Makefile + ARM GCC**.
   - Flash the generated binary to the Nucleo board.

3. **Test**:
   - Test basic GPIO functionality like toggling the onboard LED using PA5.
   - Connecting an External LED at PB9 pin -> _LED_toggle.mp4_
   - Connecting an External LED with Extenal button, and toggling when this button is pressed -> _LED_toggle_with_ext_button.mp4_
   - Extend with interrupt-driven GPIO to explore more advanced use cases.

## Key Concepts

- **Peripheral Clock Control**:
  Ensures that the GPIO peripheral is enabled via the RCC registers before configuration.
  
- **Push-Pull vs. Open-Drain**:
  Push-pull allows output to both high and low states, while open-drain only pulls to low.

- **Pull-Up/Pull-Down**:
  Configures pins to default to a high or low state to avoid floating conditions.

- **Interrupt Handling**:
  Configures EXTI lines to trigger ISRs for edge-detection on GPIO pins.

## Highlights

1. **Efficient Register Access**:
   - Direct manipulation for low overhead.
   - Bitwise operations for precise control.

2. **Robust Driver Design**:
   - Modular and reusable APIs.
   - Comprehensive initialization and deinitialization routines.

3. **Interrupt Support**:
   - Seamlessly handle external interrupts for GPIO pins.



https://github.com/user-attachments/assets/cbc84dc1-37a7-4353-8210-86a114f5a6ce



https://github.com/user-attachments/assets/29efc48c-a821-4da1-baab-65b479ec1b56



