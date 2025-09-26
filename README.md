# Frequency Counter!

A precision frequency counter with **always-on dual-mode operation** using GPS PPS as reference.

## Features

- **Always-On 1 PPS Output**: Continuous precision timing signal generation on pin 16
- **GPS PPS Monitoring**: Automatic frequency measurement when GPS PPS is available on pin 15
- **SiT5501 MEMS Oscillator Support**: I2C control for ±3200 PPM frequency adjustment
- **High Precision**: Uses GPT2 timer with 10 MHz timebase for accurate measurements
- **Robust Operation**: Works with or without GPS PPS input

## Operating Mode

### Dual Mode (Always Active)
The system operates in a single, robust dual mode:

- **Output Compare**: Always generates 1 PPS (or other frequencies) on pin 16
- **Input Capture**: Continuously monitors pin 15 for GPS PPS signals
- **Automatic Reporting**: Displays frequency measurements only when GPS PPS is received
- **Independent Operation**: 1 PPS output works regardless of GPS PPS availability

This design ensures the frequency counter is always useful - providing a precision timing reference even without GPS input, and automatically providing frequency measurements when GPS becomes available.

## Pin Configuration

| Pin | Function | Status |
|-----|----------|--------|
| 14  | External clock input | Optional (for future use) |
| 15  | GPS PPS input | Always monitoring |
| 41  | Precision timing output | Always active (1 PPS default) |
| 18  | SDA (I2C for SiT5501) | Always available |
| 19  | SCL (I2C for SiT5501) | Always available |

## Serial Commands

Key commands available via serial interface:
- `s` - Show system status
- `f<freq>` - Set output frequency (e.g., `f10` for 10 Hz)
- `p<ppm>` - Set oscillator frequency offset (e.g., `p+2.5`)
- `b` - Reboot to bootloader mode for firmware updates (orange LED will blink)
- `h` - Show complete help menu

## Usage Examples

See `examples/` directory for:
- `gpt2_dual_mode_example.ino` - Demonstrates the dual-mode operation
- `sit5501_example.ino` - Shows SiT5501 oscillator control via I2C
