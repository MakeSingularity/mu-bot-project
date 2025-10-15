# Emu Droid Hardware Schematics

## Block Diagram

```
                    Emu Droid System Architecture
                           
    ┌─────────────────┐    ┌─────────────────┐
    │  Left Camera    │    │  Right Camera   │
    │  Arducam 5MP    │    │  Arducam 5MP    │
    │  OV5647         │    │  OV5647         │
    └─────────┬───────┘    └─────────┬───────┘
              │ CSI-0                │ CSI-1
              │                      │
    ┌─────────▼──────────────────────▼─────────┐
    │           Raspberry Pi 5 (16GB)          │
    │  ┌─────────────────────────────────────┐  │
    │  │            GPIO Header              │  │
    │  └─────────────┬───────────────────────┘  │
    └────────────────┼──────────────────────────┘
                     │
    ┌────────────────▼──────────────────────────┐
    │         Hailo AI HAT+ (26 TOPS)          │
    │  ┌─────────────────────────────────────┐  │
    │  │          GPIO Passthrough           │  │
    │  └─────────────┬───────────────────────┘  │
    └────────────────┼──────────────────────────┘
                     │
    ┌────────────────▼──────────────────────────┐
    │        Waveshare WM8960 Audio HAT        │
    │  ┌─────────────────────────────────────┐  │
    │  │   I2S: GPIO 18-21                  │  │
    │  │   Mic In, Speaker Out, 3.5mm Jack  │  │
    │  │          GPIO Passthrough           │  │
    │  └─────────────┬───────────────────────┘  │
    └────────────────┼──────────────────────────┘
                     │
    ┌────────────────▼──────────────────────────┐
    │       PCA9685 16-Channel PWM HAT         │
    │  ┌─────────────────────────────────────┐  │
    │  │   I2C: GPIO 2,3 (Address 0x40)     │  │
    │  │   PWM Channels 0-15                 │  │
    │  │   External 5V Power Input           │  │
    │  └─────────────────────────────────────┘  │
    └──┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┘
       │   │   │   │   │   │   │   │   │   │
       │   │   │   │   │   │   │   │   │   └─ Ch 11: Neck Servo 6
       │   │   │   │   │   │   │   │   └───── Ch 10: Neck Servo 5
       │   │   │   │   │   │   │   └───────── Ch 9:  Neck Servo 4
       │   │   │   │   │   │   └───────────── Ch 8:  Neck Servo 3
       │   │   │   │   │   └───────────────── Ch 7:  Neck Servo 2
       │   │   │   │   └───────────────────── Ch 6:  Neck Servo 1
       │   │   │   └───────────────────────── Ch 5:  Right Eye Servo 3
       │   │   └───────────────────────────── Ch 4:  Right Eye Servo 2
       │   └───────────────────────────────── Ch 3:  Right Eye Servo 1
       └───────────────────────────────────── Ch 0-2: Left Eye Servos 1-3
```

## Power Distribution Schematic

```
External Power Supplies:
                                    
┌─────────────────┐    ┌─────────────────┐
│   5V/5A USB-C  │    │   5V/10A DC     │
│   Pi Power      │    │   Servo Power   │
└─────────┬───────┘    └─────────┬───────┘
          │                      │
          │                      │
          ▼                      ▼
    ┌─────────┐              ┌─────────┐
    │   Pi    │              │ PCA9685 │
    │  GPIO   │              │ V+ Term │
    │  5V/GND │              │         │
    └─────────┘              └─────────┘
          │                      │
          │ Powers HAT Stack     │ Powers Servos
          ▼                      ▼
    ┌─────────┐              ┌─────────┐
    │ Hailo   │              │ 12x     │
    │ WM8960  │              │ SG90    │  
    │ PCA9685 │              │ Servos  │
    └─────────┘              └─────────┘
```

## GPIO Pinout Reference

```
Raspberry Pi 5 GPIO Header (40-pin)

     3V3  (1) (2)  5V      ← HAT Stack Power
   GPIO2  (3) (4)  5V      ← I2C SDA (PCA9685)
   GPIO3  (5) (6)  GND     ← I2C SCL (PCA9685)
   GPIO4  (7) (8)  GPIO14
     GND  (9) (10) GPIO15
  GPIO17 (11) (12) GPIO18  ← I2S BCK (WM8960)
  GPIO27 (13) (14) GND
  GPIO22 (15) (16) GPIO23
     3V3 (17) (18) GPIO24
  GPIO10 (19) (20) GND
   GPIO9 (21) (22) GPIO25
  GPIO11 (23) (24) GPIO8
     GND (25) (26) GPIO7
   GPIO0 (27) (28) GPIO1
   GPIO5 (29) (30) GND
   GPIO6 (31) (32) GPIO12
  GPIO13 (33) (34) GND
  GPIO19 (35) (36) GPIO16  ← I2S LRCK (WM8960)
  GPIO26 (37) (38) GPIO20  ← I2S DIN (WM8960)
     GND (39) (40) GPIO21  ← I2S DOUT (WM8960)
```

## Camera Interface Diagram

```
Stereo Camera Setup:

    Left Eye Assembly          Right Eye Assembly
    ┌─────────────────┐        ┌─────────────────┐
    │  ┌───────────┐  │        │  ┌───────────┐  │
    │  │ Arducam   │  │        │  │ Arducam   │  │
    │  │ 5MP OV5647│  │        │  │ 5MP OV5647│  │
    │  │ Autofocus │  │        │  │ Autofocus │  │
    │  └─────┬─────┘  │        │  └─────┬─────┘  │
    │        │ 15cm   │        │        │ 20cm   │
    │        │ Flex   │        │        │ Flex   │
    └────────┼────────┘        └────────┼────────┘
             │                          │
             ▼                          ▼
        CSI-0 Port                 CSI-1 Port
    ┌─────────────────────────────────────────────┐
    │           Raspberry Pi 5                    │
    └─────────────────────────────────────────────┘

    Baseline Distance: 12cm (120mm)
    Field of View: 62.2° diagonal
    Resolution: 2592x1944 max, 1920x1080 @30fps
```

## Servo Connection Matrix

```
Eye Stewart Platform Configuration:

Left Eye (Channels 0-2):          Right Eye (Channels 3-5):
                                  
    Top Servo (Ch 2)                   Top Servo (Ch 5)
         │                                   │
         │                                   │
    ┌────▼────┐                         ┌────▼────┐
    │ Camera  │                         │ Camera  │
    │Platform │                         │Platform │
    └────┬────┘                         └────┬────┘
         │                                   │
Base─────┼─────Mid                  Base─────┼─────Mid
   (Ch 0)     (Ch 1)                   (Ch 3)     (Ch 4)


Neck Cable System (Channels 6-11):

    Servo 1 (Ch 6) ────┐
    Servo 2 (Ch 7) ────┼─── Upper Neck Control
    Servo 3 (Ch 8) ────┘
                        
    Servo 4 (Ch 9) ────┐
    Servo 5 (Ch 10)────┼─── Lower Neck Control  
    Servo 6 (Ch 11)────┘

    Cable Routing: Each servo pulls a cable through
    pulleys to create 6-DOF neck movement
```

## I2C Device Addresses

```
I2C Bus 1 (GPIO 2/3) Device Map:

Address | Device              | Function
--------|--------------------|-----------------------
0x1A    | WM8960 Audio Codec | Audio input/output
0x40    | PCA9685 PWM Driver | Servo control (default)
0x41    | PCA9685 (Alt)      | Alternative address  
0x50    | EEPROM (Optional)  | Configuration storage
0x68    | RTC (Optional)     | Real-time clock
0x70-77 | Mux (Future)       | I2C expansion

Note: Addresses 0x40-0x41 configurable via PCA9685 jumpers
```

## Signal Specifications

### PWM Servo Control
- **Frequency**: 50Hz (20ms period)
- **Pulse Width**: 1-2ms (1.5ms center)
- **Voltage**: 3.3V logic level
- **Current**: 20mA max per channel

### I2S Audio Interface  
- **Sample Rate**: 8-48kHz configurable
- **Bit Depth**: 16/24/32-bit
- **Channels**: Stereo (L/R)
- **Clock**: Master mode from Pi

### CSI Camera Interface
- **Lanes**: 2-lane MIPI CSI-2
- **Data Rate**: Up to 1Gbps per lane
- **Voltage**: 1.2V differential
- **Frequency**: 80-1000MHz

## Mechanical Mounting

```
HAT Stack Assembly (Side View):

    ╔═══════════════════════════════════╗  ← PCA9685 PWM HAT
    ║   PWM Channels 0-15              ║    (External power)
    ╠═══════════════════════════════════╣
    ║                                   ║  ← Standoffs (11mm)
    ╔═══════════════════════════════════╗  ← WM8960 Audio HAT  
    ║   Mic/Speaker/3.5mm              ║    (I2S interface)
    ╠═══════════════════════════════════╣
    ║                                   ║  ← Standoffs (11mm)
    ╔═══════════════════════════════════╗  ← Hailo AI HAT+
    ║   26 TOPS AI Accelerator         ║    (AI inference)
    ╠═══════════════════════════════════╣
    ║                                   ║  ← Standoffs (11mm)  
    ╔═══════════════════════════════════╗  ← Raspberry Pi 5
    ║   CPU, RAM, GPIO, CSI Ports      ║    (Main computer)
    ╚═══════════════════════════════════╝

    Total Stack Height: ~65mm
    Cooling: Fan on top HAT recommended
```

This schematic provides the complete hardware reference for building the emu droid companion robot with proper electrical connections and mechanical assembly.