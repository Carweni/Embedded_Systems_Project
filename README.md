# Embedded_Systems_Project
Embedded system project built with Zephyr RTOS that demonstrates multi-threaded control of an LED with real-time monitoring through an ADC, display output, and UART command interface.

## Features

- **Multi-threaded architecture** with prioritized tasks
- **LED control** (ON/OFF/Blinking)
- **ADC monitoring** of voltage with percentage calculation
- **Display output** showing system status and ADC readings
- **UART command interface** for system control
- **Real-time system information** via command interface
- **Thread-safe data sharing** using mutexes and semaphores

## Hardware Requirements

- STM32F429I-DISC1 development board
- ADC input source (potentiometer or voltage source)

## Software Requirements

- Zephyr RTOS
- Zephyr SDK
- Serial terminal program (minicom, screen, Putty, etc.)

## Building and Flashing

Navigate to your Zephyr project directory:
```bash
cd %HOMEPATH%\zephyrproject\zephyr
```

Build the project (with pristine build option):

```bash
west build -p always -b stm32f429i_disc1 Embedded_Systems_Project
```

Flash to the board:

```bash
west flash
```