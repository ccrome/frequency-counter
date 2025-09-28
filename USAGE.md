# Frequency Counter Usage Guide

## Overview
This precision frequency counter provides always-on operation with comprehensive serial command interface.

## Operating Modes

### 1. Input Capture Mode (Default)
- **Purpose**: Measures frequency by counting signal edges between PPS intervals
- **Pin Usage**: 
  - Pin 14: External clock input
  - Pin 15: PPS input signal
- **Command**: `i`

### 2. Output Compare Mode
- **Purpose**: Generates precise timing signals (1 PPS or other frequencies)
- **Pin Usage**: Pin 41: Frequency output
- **Command**: `o`

## Serial Commands

### Mode Control
- `i` - Switch to Input Capture mode (frequency measurement)
- `o` - Switch to Output Compare mode (1 PPS generation)
- `s` - Show current system status
- `h` - Show help menu

### Output Compare Mode Commands
- `f<freq>` - Set output frequency in Hz
  - Examples: `f1` (1 Hz), `f10` (10 Hz), `f1000` (1 kHz)
  - Range: 1 Hz to 1,000,000 Hz

### SiT5501 Oscillator Control
- `p<ppm>` - Set frequency offset in PPM
  - Examples: `p+5.2`, `p-3.1`, `p0` (center frequency)
  - Range: ±3200 PPM
- `r` - Read current oscillator settings
- `e` - Enable oscillator output
- `d` - Disable oscillator output

### Other Commands
- `h` - Show help menu
- `b` - Reboot to bootloader mode for firmware updates

## Example Usage Sessions

### Frequency Measurement Session
```
> i                    # Switch to input capture mode
> Connect PPS signal to pin 15
> (Frequency measurements will be displayed automatically)
> s                    # Check status and statistics
```

### Signal Generation Session
```
> o                    # Switch to output compare mode
> 1 PPS signal available on pin 41
> f10                  # Change to 10 Hz output
> Output frequency set to 10 Hz
> f1                   # Back to 1 PPS
```

### Oscillator Calibration Session
```
> r                    # Read current oscillator setting
> SiT5501 current setting: 0.000000 PPM
> p+2.5                # Set +2.5 PPM offset
> Oscillator frequency offset set to 2.500 PPM
> r                    # Verify setting
> SiT5501 current setting: 2.500000 PPM
```

### Firmware Update Session
```
> b                    # Reboot to bootloader mode
> Rebooting to bootloader mode...
> Device will disconnect and enter bootloader for firmware updates.
# Device disconnects and enters Teensy bootloader mode
# Orange LED will blink indicating bootloader is active
# Use Teensy Loader or Arduino IDE to upload new firmware
# No need to press the program button manually
```

## Pin Configuration Summary

| Pin | Function | Mode | Description |
|-----|----------|------|-------------|
| 14  | External clock input | Input Capture | 10 MHz signal input |
| 15  | PPS input | Input Capture | GPS PPS signal input |
| 41  | Frequency output | Output Compare | Generated timing signals |
| 18  | SDA (I2C) | Both | SiT5501 oscillator control |
| 19  | SCL (I2C) | Both | SiT5501 oscillator control |

## Technical Specifications

### Input Capture Mode
- **Timebase**: 10 MHz internal clock
- **Measurement Method**: Count ticks between PPS edges
- **Accuracy**: Limited by PPS signal quality and 10 MHz reference
- **Display**: Real-time frequency, averaging, PPM error

### Output Compare Mode
- **Timebase**: 10 MHz internal clock
- **Output Type**: Square wave (toggle mode)
- **Frequency Range**: 1 Hz to 1 MHz
- **Accuracy**: ±1 count (±0.1 PPM at 1 Hz)

### SiT5501 MEMS Oscillator
- **Control Interface**: I2C
- **Frequency Range**: ±3200 PPM adjustment
- **Resolution**: ~0.000015 PPM (32-bit control)
- **Stability**: ±10 ppb over temperature

## Troubleshooting

### Common Issues
1. **"SiT5501 oscillator not found"**
   - Check I2C connections (pins 18, 19)
   - Verify oscillator I2C address (default: 0x60)
   - System will work without oscillator for basic frequency counting

2. **"Must be in Output Compare mode"**
   - Use `o` command first to switch modes
   - Check current mode with `s` command

3. **No frequency measurements**
   - Ensure in Input Capture mode (`i` command)
   - Check PPS signal connection to pin 15
   - Verify signal levels (3.3V logic)

### Status Checking
Use the `s` command to display:
- Current operating mode
- GPT2 register values
- Sample statistics
- SiT5501 oscillator status
