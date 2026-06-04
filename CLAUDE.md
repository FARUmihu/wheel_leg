# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

`wheel_leg` is a bare-metal firmware project for a **STM32F407VETx** (Cortex-M4, 168 MHz) robot controller. It targets a wheel-legged robot platform and is built with STM32CubeIDE / arm-none-eabi-gcc.

## Build System

This is a **STM32CubeIDE managed-build project** (Eclipse CDT). There is no standalone Makefile at the root — builds are driven by the IDE or its generated Makefile under `Debug/`.

**Build via command line (from project root):**
```sh
# Requires arm-none-eabi-gcc toolchain on PATH
make -C Debug -j$(nproc)
```

**Flash via ST-Link:**
```sh
st-flash write Debug/wheel_leg.bin 0x08000000
# or with OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program Debug/wheel_leg.elf verify reset exit"
```

**Regenerate peripheral init code:** Open `wheel_leg.ioc` in STM32CubeMX and click *Generate Code*. Only edit within `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks — everything outside is overwritten by CubeMX.

## Architecture

### Layer structure

```
user/          ← application logic (owned by developer)
  bsp/         ← board-support: hardware abstraction above HAL
  algorithm/   ← control algorithms (e.g. balance, kinematics)
  app/         ← top-level task/state-machine code
  controller/  ← PID / state controllers
  devices/     ← device drivers (motors, sensors)

Core/Src/      ← CubeMX-generated peripheral init (CAN, TIM, GPIO)
Core/Inc/      ← CubeMX-generated headers
Drivers/       ← ST HAL + CMSIS (do not edit)
```

### Key peripherals

| Peripheral | Purpose | Notes |
|---|---|---|
| CAN1 | Primary motor/sensor bus | PA11/PA12, 1 Mbit/s, RX interrupt on `CAN1_RX0_IRQn` |
| CAN2 | Secondary bus | PB12/PB13, 1 Mbit/s |
| TIM6 | Control-loop timer | 84 MHz APB1 timer, prescaler=83, period=499 → **2 kHz** IRQ |

### Clock

HSI 16 MHz → PLL → **168 MHz** SYSCLK (AHB), APB1 = 42 MHz (timers ×2 = 84 MHz), APB2 = 84 MHz.

### Code placement rule

All user code goes under `user/`. The four sub-directories map to concerns:
- `bsp/` — thin wrappers over HAL (e.g. `bsp_can` sends/receives CAN frames)
- `devices/` — stateful drivers for physical devices (motors, IMU, etc.)
- `controller/` — control-law implementations
- `app/` — application tasks; `algorithm/` — math/kinematics utilities

The main loop (`Core/Src/main.c`) initialises peripherals then enters a bare `while(1)`. Real work happens in the TIM6 IRQ and CAN RX IRQ — add hooks there inside `USER CODE` blocks, or call user-layer init from `main()`.

## Linker Scripts

Two scripts are provided:
- `STM32F407VETX_FLASH.ld` — normal flash execution (used for production)
- `STM32F407VETX_RAM.ld` — execute-in-RAM (used for fast debug cycles)

The debug build configuration uses the flash script by default (see `.cproject`).
