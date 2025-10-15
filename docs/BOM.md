# Bill of Materials (BOM) - Emu Droid Companion Robot

## Overview
Complete parts list for building the emu droid companion robot, organized by subsystem. Prices are estimates as of October 2025 and may vary by supplier.

## Core Computing Stack

| Component | Model/Part Number | Quantity | Unit Price | Total | Supplier | Notes |
|-----------|-------------------|----------|------------|-------|----------|--------|
| Single Board Computer | Raspberry Pi 5 (16GB) | 1 | $120 | $120 | Official Pi retailers | Main brain, 16GB RAM essential |
| AI Accelerator | Hailo AI HAT+ (26 TOPS) | 1 | $299 | $299 | Hailo, distributors | Key component for inference |
| Audio Interface | Waveshare WM8960 Audio HAT | 1 | $35 | $35 | Waveshare, Amazon | Mic input + speaker output |
| Servo Controller | PCA9685 16-Channel PWM HAT | 1 | $25 | $25 | Adafruit, Amazon | I2C servo control |
| Storage | SanDisk Extreme 64GB microSD | 1 | $15 | $15 | Amazon, Best Buy | Class 10, fast read/write |
| Power Supply (Pi) | Official Pi 5 USB-C 5V/5A | 1 | $12 | $12 | Official Pi retailers | Clean power for Pi stack |
| **Subtotal Computing** | | | | **$506** | | |

## Vision System

| Component | Model/Part Number | Quantity | Unit Price | Total | Supplier | Notes |
|-----------|-------------------|----------|------------|-------|----------|--------|
| Left Camera | Arducam 5MP Motorized Focus B0344 | 1 | $45 | $45 | Arducam, Amazon | OV5647, autofocus |
| Right Camera | Arducam 5MP Motorized Focus B0344 | 1 | $45 | $45 | Arducam, Amazon | Stereo pair |
| CSI Cable (Left) | 15cm MIPI CSI-2 Cable | 1 | $8 | $8 | Arducam, Pi retailers | Standard length |
| CSI Cable (Right) | 20cm MIPI CSI-2 Cable | 1 | $10 | $10 | Arducam, Pi retailers | Longer for positioning |
| Camera Mounts | 3D Printed Eyeball Assembly | 2 | $5 | $10 | Self-print or service | PLA/PETG material |
| **Subtotal Vision** | | | | **$118** | | |

## Servo Motors & Actuation

| Component | Model/Part Number | Quantity | Unit Price | Total | Supplier | Notes |
|-----------|-------------------|----------|------------|-------|----------|--------|
| Eye Platform Servos | TowerPro SG90 Micro Servo | 6 | $3 | $18 | Amazon, robotics suppliers | 3 per eye (Stewart platform) |
| Neck Control Servos | TowerPro SG90 Micro Servo | 6 | $3 | $18 | Amazon, robotics suppliers | Cable-driven neck |
| Servo Power Supply | 5V/10A Switching PSU | 1 | $25 | $25 | Amazon, electronics suppliers | External servo power |
| Servo Cables | 20cm Extension Cables | 12 | $2 | $24 | Servo suppliers | PWM signal extension |
| **Subtotal Servos** | | | | **$85** | | |

## Mechanical Structure

| Component | Model/Part Number | Quantity | Unit Price | Total | Supplier | Notes |
|-----------|-------------------|----------|------------|-------|----------|--------|
| Leg Bones | 2020 Aluminum Extrusion 50cm | 4 | $8 | $32 | 80/20, Misumi, Amazon | Black anodized preferred |
| Leg Bones | 2020 Aluminum Extrusion 30cm | 4 | $6 | $24 | 80/20, Misumi, Amazon | Lower leg segments |
| Corner Brackets | 2020 Corner Bracket Set | 8 | $3 | $24 | 80/20, Amazon | Joint connections |
| Linear Bearings | LM8UU Linear Bearing | 8 | $4 | $32 | Robotics suppliers | Smooth leg movement |
| Linear Shafts | 8mm Chrome Steel Rod 40cm | 4 | $6 | $24 | McMaster, Amazon | Bearing guides |
| T-Nuts & Bolts | M5 T-Nut & Bolt Kit | 1 | $15 | $15 | 80/20, fastener suppliers | Assembly hardware |
| **Subtotal Mechanical** | | | | **$151** | | |

## 3D Printed Components

| Component | Material | Estimated Cost | Print Time | Notes |
|-----------|----------|----------------|------------|--------|
| Head Assembly | PETG Black | $8 | 12 hours | Houses Pi stack + cameras |
| Eye Platforms (2x) | PLA Black | $4 | 6 hours | Stewart platform base |
| Joint Brackets (8x) | PETG Black | $12 | 16 hours | Leg joint connections |
| Foot Assemblies (2x) | TPU/PETG | $6 | 8 hours | Impact absorption |
| Cable Guides | PLA | $3 | 4 hours | Neck cable routing |
| **Subtotal 3D Printing** | | **$33** | **46 hours** | PLA/PETG filament |

## Cables & Connectivity

| Component | Specification | Quantity | Unit Price | Total | Supplier | Notes |
|-----------|---------------|----------|------------|-------|----------|--------|
| GPIO Stacking Header | 2x20 40-pin | 3 | $2 | $6 | Pi retailers | HAT stacking |
| I2C Cables | 4-wire 30cm | 2 | $3 | $6 | Electronics suppliers | Sensor connections |
| Power Cables | 18AWG Silicone Wire | 5m | $8 | $8 | Electronics suppliers | Servo power distribution |
| Heat Shrink Tubing | Assorted Kit | 1 | $12 | $12 | Electronics suppliers | Wire protection |
| Cable Management | Spiral Wrap & Ties | 1 | $10 | $10 | Amazon, electronics | Clean cable routing |
| **Subtotal Cables** | | | | **$42** | | |

## Energy Management (Future Upgrade)

| Component | Model/Part Number | Quantity | Unit Price | Total | Supplier | Notes |
|-----------|-------------------|----------|------------|-------|----------|--------|
| Battery Pack | LiPo 4S 5000mAh 25C | 1 | $85 | $85 | RC suppliers | Portable power option |
| Battery Management | BMS 4S 30A Protection | 1 | $25 | $25 | Electronics suppliers | Safety protection |
| DC-DC Converter | 5V/10A Buck Converter | 1 | $15 | $15 | Electronics suppliers | Efficient 5V rail |
| **Subtotal Energy** | | | | **$125** | | *Optional for mobile operation* |

## Tools & Assembly

| Tool | Purpose | Estimated Cost | Notes |
|------|---------|----------------|--------|
| Hex Key Set | M3-M6 assembly | $15 | T-handle preferred |
| Soldering Iron | Wire connections | $25 | Temperature controlled |
| 3D Printer Access | Component printing | $200 | Or print service |
| Multimeter | Electrical testing | $30 | Basic digital meter |
| **Subtotal Tools** | | **$270** | *One-time investment* |

## Cost Summary

| Subsystem | Cost | Percentage |
|-----------|------|------------|
| Core Computing Stack | $506 | 43.8% |
| Vision System | $118 | 10.2% |
| Servo Motors & Actuation | $85 | 7.4% |
| Mechanical Structure | $151 | 13.1% |
| 3D Printed Components | $33 | 2.9% |
| Cables & Connectivity | $42 | 3.6% |
| **Total Base System** | **$935** | **81.0%** |
| Energy Management (Optional) | $125 | 10.8% |
| Tools (One-time) | $270 | 23.4% |
| **Grand Total** | **$1,330** | | |

## Quantity Discounts

Building multiple units (5+ droids for Maker Faire):

| Component | Single Unit | 5-Unit Discount | Savings per Unit |
|-----------|-------------|-----------------|------------------|
| Raspberry Pi 5 | $120 | $110 | $10 |
| Hailo AI HAT+ | $299 | $275 | $24 |
| Servos (12x) | $36 | $30 | $6 |
| Aluminum Extrusion | $56 | $45 | $11 |
| **Total Savings** | | | **$51 per unit** |

**5-Unit Project Cost: $4,420** (vs $4,675 individual)

## Supplier Recommendations

### Primary Suppliers
- **Raspberry Pi Official**: Direct from foundation for Pi/accessories
- **Hailo**: Direct or authorized distributors for AI HAT+
- **Amazon**: Convenient for servos, cables, small components
- **80/20 Inc**: Aluminum extrusion specialists
- **Adafruit**: Quality electronics, good documentation

### Alternative Suppliers
- **DigiKey/Mouser**: Professional electronics components
- **AliExpress**: Cost-effective servos/cables (longer shipping)
- **McMaster-Carr**: Precision mechanical components
- **Local Makerspaces**: 3D printing services

## Lead Times & Availability

| Component | Typical Lead Time | Stock Status |
|-----------|-------------------|--------------|
| Raspberry Pi 5 | 2-4 weeks | Improving availability |
| Hailo AI HAT+ | 4-6 weeks | New product, limited stock |
| Standard components | 1-2 weeks | Good availability |
| 3D Printing | 1-3 days | Local/on-demand |

**Recommended order timing: Start procurement 8 weeks before assembly**

## Cost Optimization Notes

1. **Hailo AI HAT+** is the most expensive single component (32% of total)
2. **3D printing** in-house vs service can save $50-100 per unit
3. **Bulk servo purchases** offer significant savings for multi-unit builds
4. **Aluminum extrusion** can be sourced locally to save shipping costs
5. **Tools** are one-time investment, amortized across multiple builds

## Mare Island Maker Faire Budget (5 Units)

- **5 Complete Droids**: $4,420
- **Shipping/Taxes**: $450
- **Contingency (10%)**: $487
- **Documentation/Signage**: $150
- ****Total Faire Budget**: $5,507**

This BOM provides a realistic cost estimate for the emu droid project with supplier recommendations and scaling considerations for the September 2026 Maker Faire demonstration.