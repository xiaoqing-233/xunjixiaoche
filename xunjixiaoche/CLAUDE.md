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
Core/           STM32CubeMX-generated HAL init (main.c, tim.c, usart.c, gpio.c, dma.c + headers)
Drivers/        STM32 HAL libraries (CMSIS + STM32F4xx_HAL_Driver) — do not modify
code/           User application modules (all custom logic lives here)
OLED_I2C/       SSD1306 OLED display driver (software I2C, 128x64)
MDK-ARM/        Keil project files + startup assembly
xunjixiaoche.ioc  STM32CubeMX project config
```

### Peripheral Assignment

| Peripheral | Purpose |
|---|---|
| TIM3 | Right encoder interface (PA6/PA7) |
| TIM4 | Left encoder interface (PD12/PD13) |
| TIM5 | Periodic interrupt for speed control loop |
| TIM12 | DRV8701 EN/PWM outputs: CH1 right, CH2 left |
| USART1 + DMA2 Stream7 | VOFA command RX and JustFloat DMA TX |
| USART2 | JY901S IMU on PA2/PA3 (baud 115200) |
| Software I2C | Grayscale sensor (PE1 SDA, PE2 SCL) |
| Software I2C | SSD1306 OLED display (PB8 SDA, PB9 SCL) |

### Control Loop (cascade PID)

1. **Grayscale sensor** (`ganweixunji.c` → `gw_get_value()`) reads 8-channel digital value via software I2C into global `Digtal`
2. **Line detection** (`xunji.c` → `track_line()`) converts 8-bit sensor data to a weighted position, determines line offset and turn state (straight/left/right/sharp-turn)
3. **Outer grayscale/position PID** (`pid.c` → `control_speed()`) runs at 100 Hz on every TIM5 interrupt and updates `target_gyro_z`.
4. **Middle angular-rate PID** (`pid.c` → `control_speed()`) runs at 100 Hz and tracks `target_gyro_z` against JY901S `GyrZ`.
5. **Inner speed PID** (`pid.c` → `control_speed()`, called from the 100 Hz TIM5 ISR) runs at 100 Hz, uses incremental PID to track speed targets, and reads encoder counts from TIM4/TIM3.
6. **Motor output** (`motor.c` → `set_speed()`) applies PWM+direction signals, clamped to ±10000. TIM12 uses ARR=4199 with Prescaler=0: command magnitudes 0..10000 map to CCR 0..4200 using ARR + 1, so 10000 is 100% duty cycle at 20 kHz (84 MHz timer clock).

Key globals: `target_position` (desired yaw from line tracking), `star_car` (start/stop via button), `Yaw`/`GyrZ` (IMU angle/angular velocity)

### Parameter Tuning via UART

`VOFA/vofa.c` receives newline- or `!`-terminated `name=value` commands through USART1. `V` changes base speed, `SW` starts/stops, and `P1`/`I1`/`D1`, `P2`/`I2`/`D2`, `P3`/`I3`/`D3` tune the speed, gyro, and position PID loops respectively. `VOFA/justfloat.c` sends JustFloat frames with USART1 DMA. The main loop schedules best-effort JustFloat telemetry every 10 ms as `speed_left`, then `speed_right`; a busy DMA transfer skips that frame. JY901S uses USART2 on PA2/PA3 with single-byte interrupt reception.

### Buttons

PC1, PC2, and PC3 are active-low user buttons with internal pull-ups and 20 ms software debouncing. `key_control()` runs in the main loop. A confirmed PC1 press toggles `star_car`, resets PID history on either transition, and immediately stops motor output when disabling. PC2 and PC3 record their debounced press events through `key_get_last_event()` but have no assigned business action.

### Pin Definitions

- Grayscale I2C: SDA = PE1, SCL = PE2
- Left motor: PE0 (PH), PB15 (EN/PWM, TIM12_CH2)
- Right motor: PB5 (PH), PB14 (EN/PWM, TIM12_CH1)
- Left encoder: PD12/PD13 (TIM4); right encoder: PA6/PA7 (TIM3)
- OLED software I2C: SDA = PB8, SCL = PB9
- OLED PB8/PB9 require external pull-up resistors for the open-drain bus.
- User buttons: PC1 (key1), PC2 (key2), PC3 (key3), active-low with internal pull-ups
- Soft I2C: PE1 (SDA), PE2 (SCL) — bit-banged, not hardware I2C
