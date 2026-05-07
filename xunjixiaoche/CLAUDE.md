# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F407VGT6 line-following car (循迹小车) using STM32CubeMX-generated HAL code. Built with Keil MDK-ARM IDE (`xunjixiaoche.uvprojx`).

## Build & Flash

- Open `MDK-ARM/xunjixiaoche.uvprojx` in Keil MDK-ARM (v5)
- Compile: F7 or Project → Build Target
- Flash/debug: F8 or use ST-LINK via SWD
- MCU: STM32F407VGT6, 168 MHz HSE, LQFP100 package

## Architecture

### Directory Layout

```
Core/           STM32CubeMX-generated HAL init (main.c, tim.c, usart.c, spi.c, gpio.c, dma.c + headers)
Drivers/        STM32 HAL libraries (CMSIS + STM32F4xx_HAL_Driver) — do not modify
code/           User application modules (all custom logic lives here)
0.96spi/        OLED display driver (SPI, 128x64)
MDK-ARM/        Keil project files + startup assembly
xunjixiaoche.ioc  STM32CubeMX project config
```

### Peripheral Assignment

| Peripheral | Purpose |
|---|---|
| TIM1, TIM2 | Encoder mode — left/right motor speed feedback |
| TIM3 CH4 | Left motor PWM (PC7/PC8 direction) |
| TIM4 CH3 | Right motor PWM (PC6/PD15 direction) |
| TIM5 | Periodic interrupt for speed control loop |
| TIM12 | (reserved) |
| USART1 + DMA2 Stream2/7 | JY901S IMU (baud 9600 assumed) |
| USART2 | Debug serial / Bluetooth parameter tuning |
| SPI1 | Grayscale sensor (software I2C via PC1/PC2) |
| SPI2 | OLED display (PC10/PC11/PC12 + PD2/PD3) |

### Control Loop (cascade PID)

1. **Grayscale sensor** (`ganweixunji.c` → `gw_get_value()`) reads 8-channel digital value via software I2C into global `Digtal`
2. **Line detection** (`xunji.c` → `track_line()`) converts 8-bit sensor data to a weighted position, determines line offset and turn state (straight/left/right/sharp-turn)
3. **Outer position PID** (`pid.c` → `PositionPID_Calculate()`) uses Yaw angle from JY901S IMU vs `target_position` (set by line detection). Position-correction is added/subtracted from base speed targets
4. **Inner speed PID** (`pid.c` → `control_speed()`, called from TIM5 ISR) uses incremental PID to track speed targets, reading encoder counts from TIM1/TIM2
5. **Motor output** (`motor.c` → `set_speed()`) applies PWM+direction signals, clamped to ±1000

Key globals: `target_position` (desired yaw from line tracking), `star_car` (start/stop via button), `Yaw`/`GyrZ` (IMU angle/angular velocity)

### Parameter Tuning via UART

`chuankou.c` parses UART commands of format `=PL12!` / `=PR05!` / `=IL12!` / `=IR05!` to adjust PID gains (`Kp_l`, `Kp_r`, `Ki_l`, `Ki_r`) at runtime. Two-digit values are divided by 10 (e.g., `12` → `1.2`).

### Button

Single button on PD8 toggles `star_car` (start/stop car movement). In `pid.c`, when `star_car == 0`, motors stop and PID state resets.

### Pin Definitions

- Grayscale I2C: SDA = PC1, SCL = PC2
- Left motor: PC8 (dir), PC7 (dir), TIM3_CH4 (PWM)
- Right motor: PD15 (dir), PC6 (dir), TIM4_CH3 (PWM)
- OLED SPI: SCK = PC10, CS = PC11, MOSI = PC12, DC = PD2, RES = PD3
- Button: PD8
- Soft I2C: PC1 (SDA), PC2 (SCL) — bit-banged, not hardware I2C
