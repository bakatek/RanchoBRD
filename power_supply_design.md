# Power Supply and Component Integration Design

This document outlines the power supply scheme for the automotive dashboard interface project, detailing input protection, voltage regulation, power distribution, grounding, and component integration notes.

## 1. Overview

The system takes a nominal 12V from the car's electrical system and regulates it down to 5V and 3.3V to power the various electronic components. Robust input protection and careful grounding are essential for reliable operation in the automotive environment.

## 2. Input Stage (Car's 12V Supply)

The car's electrical system (nominal 12V) is the primary power source. This input requires protection before feeding into the main voltage regulator.

*   **Input Source:** Car's accessory power line (switched +12V).
*   **Protection Components:**
    1.  **Fuse (F1):**
        *   Type: Automotive blade fuse (or similar).
        *   Rating: **2A** (This should be verified based on the final total current draw of all 5V components. ESP32 ~100-300mA avg, TFT ~150mA. Max current for LM2596 is 3A, but it's wise to stay well below that).
        *   Function: Protects against overcurrent conditions and short circuits.
    2.  **Reverse Polarity Protection Diode (D_prot):**
        *   Type: **1N5822 Schottky Diode** (40V, 3A).
        *   Connection: In series with the +12V input, after the fuse. Anode towards the fuse/car, Cathode towards the DC-DC converter.
        *   Function: Protects the circuit from damage if the input polarity is accidentally reversed. Chosen for its low forward voltage drop (~0.5V).
    3.  **Input Filter Capacitor (C_in_filter):**
        *   Type: Electrolytic Capacitor.
        *   Value: **470µF, 35V**.
        *   Connection: Across the input terminals of the DC-DC buck converter (after D_prot). Positive terminal to the protected +12V line, Negative terminal to Ground.
        *   Function: Smooths the noisy 12V input from the car, absorbs voltage fluctuations, and provides a stable input for the DC-DC converter. Low ESR type is preferred.

**Input Path:**
`Car +12V ACC --> F1 (2A Fuse) --> D_prot (1N5822 Anode) | (1N5822 Cathode) --> C_in_filter (+) & DC-DC_VIN+`
`Car Chassis GND ------------------------------------------------------------> C_in_filter (-) & DC-DC_VIN-`

## 3. Main Voltage Regulation (12V to 5V)

*   **Module:** DC-DC Buck Converter Module (e.g., based on LM2596).
*   **Configuration:** The module's output voltage must be adjusted (via its potentiometer) to provide a stable **+5V DC**.
*   **Input:** Receives protected and filtered 12V from the input stage.
*   **Output:** Provides a regulated +5V rail.

## 4. Power Distribution

### 4.1. +5V Rail

The +5V output from the DC-DC buck converter powers:

1.  **ESP32-S3 Wroom 1 N16R8 Module:**
    *   Connected to the ESP32 module's 5V input pin (often labeled 'VIN' or '5V').
    *   The ESP32 module itself has an onboard LDO to generate its internal 3.3V.
2.  **TFT Display (JC3248W535C):**
    *   Connected to the display's 5V power input.
    *   Nominal current draw: ~150mA.

### 4.2. +3.3V Rail

The ESP32-S3 Wroom module provides a regulated +3.3V output (from its internal LDO). This rail powers:

1.  **PCF8575 I/O Expander Module:**
    *   Connected to the VCC pin of the PCF8575 module.
2.  **Pull-up Resistors for Dashboard Indicator Optocoupler Outputs:**
    *   The 10kOhm pull-up resistors (9 total) for the PC817 optocoupler outputs (interfacing with PCF8575) are connected to this 3.3V rail.
3.  **Pull-up Resistor for RPM Optocoupler Output:**
    *   The 10kOhm pull-up resistor for the PC817 optocoupler output (interfacing with an ESP32-S3 input pin) is connected to this 3.3V rail.
4.  **I2C Pull-up Resistors (R_i2c_sda, R_i2c_scl):**
    *   Value: **4.7kOhm** (typical, can range 2.2k-10k) - 2 resistors.
    *   Connection: One from the SDA line to 3.3V, one from the SCL line to 3.3V.
    *   Function: Required for I2C communication between ESP32-S3 and PCF8575.
    *   *Note:* These may be omitted if the PCF8575 module already includes pull-up resistors to its VCC pin (which is now 3.3V).

## 5. Grounding Strategy

*   **Common Ground:** A unified and robust ground (GND) plane or star grounding point is crucial.
*   **Connections:**
    *   The car's chassis ground is the primary ground reference.
    *   This chassis ground connects to the negative input (VIN-) and negative output (VOUT-) of the DC-DC buck converter.
    *   All other module grounds (ESP32-S3 GND, TFT Display GND, PCF8575 GND) and the emitter pins of the optocoupler output transistors must be connected to this common ground.
*   **Importance:** Prevents ground loops, ensures stable voltage references, and is critical for reliable signal integrity. Use low-resistance connections for ground paths.

## 6. Component Integration and PCB Notes

*   **Modules:** The DC-DC converter and PCF8575 module will likely be separate PCBs connected to a main custom PCB hosting the ESP32-S3, TFT connector, and optocoupler circuits.
*   **Connectors:** Use reliable connectors for all off-board connections (car power, dashboard signals, RPM signal).
*   **Power Traces:** Ensure 5V and 12V power traces on any PCB are sufficiently wide to handle the expected currents.
*   **Decoupling Capacitors:** Besides the main filter capacitors, place smaller ceramic decoupling capacitors (e.g., 0.1µF) close to the power pins of each IC (ESP32, PCF8575, and TFT if applicable) to filter high-frequency noise. These would be on the custom PCB.

## 7. Block Diagram (Power Flow)

```
                                     +------------+
                                     | Car +12V   |
                                     | (Accessory)|
                                     +-----+------+
                                           |
                                     +-----v------+    +-----------------+
                                     | Fuse (2A)  |--->| Car Chassis GND |
                                     +-----+------+    +-------+---------+
                                           |                    |
                                     +-----v------+             |
                                     | 1N5822     |<------------+ (GND Ref for Opto Inputs)
                                     | (ReversePol) |             |
                                     +-----+------+             |
                                           |                    |
      +------------------------------------v--------------------+------+
      |               DC-DC Buck Converter Module (LM2596 based)       |
      |                                                                |
      | Input: ~12V (Protected)      Output: +5V DC                    |
      | VIN+                         VOUT+                             |
      | VIN- (Connected to GND)      VOUT- (Connected to GND)          |
      +-----------------|----------------+-----------------------------+
                        |                |
                        | (Common GND)   +---- (+5V Rail) ----+
                        |                                     |
                        |                               +-----v----------------+     +-----v-------------+
                        |                               | ESP32-S3 Wroom Mod.  |     | TFT Display       |
                        |                               | (VIN=5V, GND)        |     | (VCC=5V, GND)     |
                        |                               +-----+----------------+     +-------------------+
                        |                                     |
                        |                                     | (+3.3V Output from ESP32 LDO)
                        |                                     |
                        |                +--------------------+---- (+3.3V Rail) ----+
                        |                |                                           |
                        |          +-----v---------------+     +-------------------v-+-------------------+
                        |          | PCF8575 Module    |     | Pull-up Resistors:                        |
                        |          | (VCC=3.3V, GND)   |     | - Opto Outputs (Dashboard, RPM) to 3.3V |
                        +----------> (Common GND Ref) <-------+ - I2C Lines (SDA/SCL) to 3.3V         |
                                   +-------------------+     +-----------------------------------------+
```

## 8. Summary of Additional Components

*   **F1:** Automotive Fuse, 2A.
*   **D_prot:** 1N5822 Schottky Diode (or equivalent, >2A, >25V).
*   **C_in_filter:** 470µF, 35V Electrolytic Capacitor.
*   **R_i2c_sda, R_i2c_scl:** 2x 4.7kOhm Resistors, 1/4W (if not on PCF8575 module).
*   *(Implicit)* Decoupling capacitors (e.g., 0.1µF ceramic) for ICs on the custom PCB.

This power supply design aims to provide stable and protected power to all components of the system.
```
