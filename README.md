# Hoverboard Control

ESP32 receiver for controlling a hoverboard via ESP-NOW wireless communication.

## Overview

This firmware runs on an ESP32 that acts as a receiver. It receives joystick commands via ESP-NOW from a remote controller and translates them into the hoverboard's serial protocol.

## Hardware

- ESP32 development board
- Hoverboard with compatible firmware
- UART connection: TX/RX pins at 115200 baud

## Protocol

The receiver accepts ESP-NOW packets containing:
- Speed percentage (-100 to +100)
- Steer percentage (-100 to +100)
- CRC16 checksum for validation

Commands are sent to the hoverboard at 50 Hz (20ms period) using the standard hoverboard serial protocol. Values are mapped from percentage to firmware units (-1000 to +1000).

## Failsafe

If no ESP-NOW packet is received within 500ms, the receiver automatically sends zero commands to stop the hoverboard.

## Configuration

The receiver MAC address is printed in the code comments. The transmitter must be configured with this MAC address to establish communication.

## Building

Requires PlatformIO with ESP32 platform support. Build and upload using:

```
pio run -t upload
```

