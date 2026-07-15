# Project Context
We are implementing Field Oriented Control (FOC) for a BLDC motor on an STM32F446RE using C/C++ and VS Code. 

# Hardware 
* **MCU:** STM32F446RE (ARM Cortex-M4 with FPU)
* **Inverter:** Semikron educational inverter
* **Sensors:** Hall effect sensors (providing position every 60 electrical degrees)
* **MOTOR:** 3-phase bruchless motor with unknown parameters
* **Power supply:** 24Vdc lab dc power supply with 6A limit

# Coding Rules
1. Prioritize STM32 HAL and LL (Low-Layer) drivers. 
2. FOC math must be optimized for the hardware FPU.
3. ADC measurements must be strictly synchronized to the exact center of the PWM cycles.
4. When editing FOC math, account for the fact that Hall sensors require an angle-interpolation algorithm (like a PLL) to estimate continuous angles between ticks.