# Connections and PCB Layout Guidance

This document provides detailed connection lists (netlists) for the various subsystems of the automotive dashboard interface project and offers guidance for designing a single-sided Printed Circuit Board (PCB). It draws upon information from `indicator_circuit_design.md`, `rpm_input_circuit_design.md`, `power_supply_design.md`, and `esp32_pin_allocations.md`.

## 1. Connection Lists / Netlists

These lists describe the point-to-point connections for each component. Signal names like `+12V_ACC`, `GND_CAR`, `+5V_LOGIC`, `+3.3V_LOGIC`, and `GND_LOGIC` are used to denote power rails. `CAR_SIGNAL_X` refers to the specific signal wire from the car.

### 1.1. Power Input and DC-DC Converter Module

*   **F1 (2A Fuse):**
    *   `F1_Terminal1` -> `CAR_ACC_IN (+12V from Car Accessory Power)`
    *   `F1_Terminal2` -> `D_prot_Anode (1N5822)`
*   **D_prot (1N5822 Schottky Diode - Reverse Polarity Protection):**
    *   `D_prot_Anode` -> `F1_Terminal2`
    *   `D_prot_Cathode` -> `C_in_filter_Positive`
    *   `D_prot_Cathode` -> `DC_DC_Module_VIN+`
*   **C_in_filter (470µF, 35V Electrolytic Capacitor):**
    *   `C_in_filter_Positive` -> `D_prot_Cathode`
    *   `C_in_filter_Negative` -> `GND_CAR` (Also connected to `DC_DC_Module_VIN-`)
*   **DC-DC_Module (LM2596 based):**
    *   `DC_DC_Module_VIN+` -> `D_prot_Cathode`
    *   `DC_DC_Module_VIN-` -> `C_in_filter_Negative` (Connected to `GND_CAR`)
    *   `DC_DC_Module_VOUT+` -> `+5V_LOGIC` (Power rail for ESP32 and TFT)
    *   `DC_DC_Module_VOUT-` -> `GND_LOGIC` (Common ground with `GND_CAR`)

### 1.2. ESP32-S3 Wroom 1 N16R8 Module

*   **Power:**
    *   `ESP32_5V_IN (VIN)` -> `+5V_LOGIC`
    *   `ESP32_GND` -> `GND_LOGIC`
    *   `ESP32_3V3_OUT` -> `+3.3V_LOGIC` (Power rail for PCF8575 and pull-ups)
*   **RPM Input:**
    *   `ESP32_GPIO4` -> `U_Opto_RPM_Collector` (Pin 4 of RPM PC817)
*   **I2C Communication (to PCF8575):**
    *   `ESP32_GPIO8 (SDA)` -> `PCF8575_SDA`
    *   `ESP32_GPIO9 (SCL)` -> `PCF8575_SCL`
*   **TFT Display (JC3248W535C):**
    *   `ESP32_TFT_CS` -> `TFT_CS` (Chip Select)
    *   `ESP32_TFT_DC (RS)` -> `TFT_DC` (Data/Command)
    *   `ESP32_TFT_WR (SCL/CLK)` -> `TFT_WR` (Write Clock / SPI Clock)
    *   `ESP32_TFT_RD` -> `TFT_RD` (Read - may not be used or tied high)
    *   `ESP32_TFT_DATA_D0-D7/D15 (SPI_MOSI, MISO if needed)` -> `TFT_DATA_D0-D7/D15` (Parallel data lines or SPI MOSI/MISO)
    *   `ESP32_TFT_RESET` -> `TFT_RESET`
    *   `ESP32_TFT_BL_CTRL` (Optional Backlight Control) -> `TFT_BL_CTRL`
    *   *(Note: Specific TFT pin names and the number of data lines depend on the parallel/SPI mode chosen for the ST7796S controller. The user must verify these against their TFT module and ESP32 configuration.)*
*   **Decoupling Capacitor (C_decouple_esp32 - 0.1µF Ceramic):**
    *   `C_decouple_esp32_Pin1` -> `ESP32_3V3_OUT` (or `ESP32_5V_IN` if placed near the input)
    *   `C_decouple_esp32_Pin2` -> `ESP32_GND`

### 1.3. PCF8575 I/O Expander Module

*   **Power:**
    *   `PCF8575_VCC` -> `+3.3V_LOGIC`
    *   `PCF8575_GND` -> `GND_LOGIC`
*   **I2C Communication (to ESP32):**
    *   `PCF8575_SDA` -> `ESP32_GPIO8 (SDA)`
    *   `PCF8575_SCL` -> `ESP32_GPIO9 (SCL)`
*   **I2C Pull-up Resistors (R_i2c_sda, R_i2c_scl - 2x 4.7kOhm, 1/4W - if not on module):**
    *   `R_i2c_sda_Pin1` -> `PCF8575_SDA`
    *   `R_i2c_sda_Pin2` -> `+3.3V_LOGIC`
    *   `R_i2c_scl_Pin1` -> `PCF8575_SCL`
    *   `R_i2c_scl_Pin2` -> `+3.3V_LOGIC`
*   **Indicator Signal Inputs (PCF8575_P0 to PCF8575_P7 for first 8 indicators, PCF8575_P10 for 9th - example, actual pin mapping flexible):**
    *   `PCF8575_P0` -> `U_Opto_Indicator1_Collector`
    *   `PCF8575_P1` -> `U_Opto_Indicator2_Collector`
    *   ...
    *   `PCF8575_P7` -> `U_Opto_Indicator8_Collector`
    *   `PCF8575_P10` (or any other available input) -> `U_Opto_Indicator9_Collector`
    *   *(Note: The PCF8575 has 16 I/O pins, P00-P07 and P10-P17. Choose any 9 for inputs.)*
*   **Address Pins (A0, A1, A2):**
    *   `PCF8575_A0` -> `GND_LOGIC` (Example, can be set to VCC or GND for different I2C addresses)
    *   `PCF8575_A1` -> `GND_LOGIC`
    *   `PCF8575_A2` -> `GND_LOGIC`
*   **Interrupt Pin (Optional - PCF8575_INT):**
    *   `PCF8575_INT` -> (Can be connected to an ESP32 GPIO if interrupt-driven input is desired)
*   **Decoupling Capacitor (C_decouple_pcf8575 - 0.1µF Ceramic):**
    *   `C_decouple_pcf8575_Pin1` -> `PCF8575_VCC`
    *   `C_decouple_pcf8575_Pin2` -> `PCF8575_GND`

### 1.4. Indicator Light Optocoupler Circuits (9 identical circuits)

Let's denote components for Indicator `N` (where N=1 to 9) as `U_Opto_IndicatorN`, `R_limit_IndicatorN`, `R_pullup_IndicatorN_Output`.

**For Each Indicator Circuit N:**

*   **R_limit_IndicatorN (1.2kOhm, 1/4W - Input Current Limiter):**
    *   Connection depends on Active-Low or Active-High signal type:
        *   **Active-Low Signals (e.g., Dégivrage, Huile, Frein, Batterie, Essence):**
            *   `R_limit_IndicatorN_Pin1` -> `+12V_ACC` (Switched Car Accessory Power)
            *   `R_limit_IndicatorN_Pin2` -> `U_Opto_IndicatorN_Anode (Pin 1)`
        *   **Active-High Signals (e.g., Phare, Clignotants, Plein Phare):**
            *   `R_limit_IndicatorN_Pin1` -> `CAR_SIGNAL_IndicatorN` (Specific Car Signal Wire)
            *   `R_limit_IndicatorN_Pin2` -> `U_Opto_IndicatorN_Anode (Pin 1)`
*   **U_Opto_IndicatorN (PC817 Optocoupler):**
    *   `U_Opto_IndicatorN_Anode (Pin 1)` -> `R_limit_IndicatorN_Pin2`
    *   `U_Opto_IndicatorN_Cathode (Pin 2)`:
        *   **Active-Low Signals:** -> `CAR_SIGNAL_IndicatorN`
        *   **Active-High Signals:** -> `GND_CAR`
    *   `U_Opto_IndicatorN_Collector (Pin 4)` -> `PCF8575_Px` (Assigned input pin on PCF8575)
    *   `U_Opto_IndicatorN_Collector (Pin 4)` -> `R_pullup_IndicatorN_Output_Pin1`
    *   `U_Opto_IndicatorN_Emitter (Pin 3)` -> `GND_LOGIC`
*   **R_pullup_IndicatorN_Output (10kOhm, 1/4W - Output Pull-up):**
    *   `R_pullup_IndicatorN_Output_Pin1` -> `U_Opto_IndicatorN_Collector (Pin 4)`
    *   `R_pullup_IndicatorN_Output_Pin2` -> `+3.3V_LOGIC`

### 1.5. RPM Signal Input Conditioning Circuit

*   **R_series_prot (470 Ohms, 0.5W - Series Protection Resistor):**
    *   `R_series_prot_Pin1` -> `CAR_SIGNAL_RPM (from Ignition Coil Command)`
    *   `R_series_prot_Pin2` -> `D_zener_Cathode (1N4744A)`
    *   `R_series_prot_Pin2` -> `R_limit_led_rpm_Pin1`
*   **D_zener (1N4744A Zener Diode - 15V, 1W):**
    *   `D_zener_Cathode` -> `R_series_prot_Pin2`
    *   `D_zener_Anode` -> `GND_CAR`
*   **R_limit_led_rpm (620 Ohms, 1/4W - LED Current Limiter):**
    *   `R_limit_led_rpm_Pin1` -> `R_series_prot_Pin2`
    *   `R_limit_led_rpm_Pin2` -> `U_Opto_RPM_Anode (Pin 1)`
    *   `R_limit_led_rpm_Pin2` -> `C_filter_rpm_Pin1`
*   **C_filter_rpm (0.1µF, 50V Ceramic Capacitor - Noise Filter):**
    *   `C_filter_rpm_Pin1` -> `U_Opto_RPM_Anode (Pin 1)` (or `R_limit_led_rpm_Pin2`)
    *   `C_filter_rpm_Pin2` -> `U_Opto_RPM_Cathode (Pin 2)` (Connected to `GND_CAR`)
*   **U_Opto_RPM (PC817 Optocoupler):**
    *   `U_Opto_RPM_Anode (Pin 1)` -> `R_limit_led_rpm_Pin2`
    *   `U_Opto_RPM_Cathode (Pin 2)` -> `C_filter_rpm_Pin2` (Connected to `GND_CAR`)
    *   `U_Opto_RPM_Collector (Pin 4)` -> `ESP32_GPIO4`
    *   `U_Opto_RPM_Collector (Pin 4)` -> `R_pullup_rpm_esp_Pin1`
    *   `U_Opto_RPM_Emitter (Pin 3)` -> `GND_LOGIC`
*   **R_pullup_rpm_esp (10kOhm, 1/4W - Output Pull-up):**
    *   `R_pullup_rpm_esp_Pin1` -> `U_Opto_RPM_Collector (Pin 4)`
    *   `R_pullup_rpm_esp_Pin2` -> `+3.3V_LOGIC`

### 1.6. TFT Display (JC3248W535C)

*   **Power:**
    *   `TFT_VCC` -> `+5V_LOGIC`
    *   `TFT_GND` -> `GND_LOGIC`
    *   `TFT_LED_A (or BL_VCC)` -> `+5V_LOGIC` (or via a transistor if current is high/PWM control needed)
    *   `TFT_LED_K (or BL_GND)` -> `GND_LOGIC`
*   **Control & Data Signals:** (Refer to ESP32 connections above, ensure consistency)
    *   `TFT_CS` -> `ESP32_TFT_CS`
    *   `TFT_DC (RS)` -> `ESP32_TFT_DC`
    *   `TFT_WR (SCL/CLK)` -> `ESP32_TFT_WR`
    *   `TFT_RD` -> `ESP32_TFT_RD` (or tied high)
    *   `TFT_DATA_D0-D7/D15` -> `ESP32_TFT_DATA_D0-D7/D15`
    *   `TFT_RESET` -> `ESP32_TFT_RESET`
*   **Decoupling Capacitor (C_decouple_tft - 0.1µF Ceramic, if not on display module itself):**
    *   `C_decouple_tft_Pin1` -> `TFT_VCC`
    *   `C_decouple_tft_Pin2` -> `TFT_GND`

## 2. PCB Layout Guidance (Single-Sided PCB)

Designing a single-sided PCB for a mixed-signal automotive project can be challenging due to noise and routing density. Careful planning is key.

### 2.1. Component Grouping

*   **Power Input Section:** Group `F1 (Fuse Holder)`, `D_prot (1N5822)`, and `C_in_filter (470µF)` closely together near the 12V input connector. The `DC-DC_Module` should be placed adjacent to this group.
*   **Indicator Optocoupler Banks:** Group the 9 indicator optocoupler circuits. Each `U_Opto_IndicatorN` should be placed with its corresponding `R_limit_IndicatorN`. The output pull-up resistors (`R_pullup_IndicatorN_Output`) can be grouped near the PCF8575 or near their respective optocouplers.
*   **RPM Optocoupler Circuit:** Group `U_Opto_RPM` with its input components (`R_series_prot`, `D_zener`, `R_limit_led_rpm`, `C_filter_rpm`) and its output pull-up (`R_pullup_rpm_esp`).
*   **ESP32-S3 Section:** Place the ESP32-S3 module centrally or in a way that minimizes trace lengths to high-speed signals (TFT) and I2C. Place its decoupling capacitor very close to its power pins.
*   **PCF8575 Section:** Place the PCF8575 module near the bank of indicator optocoupler outputs. If I2C pull-ups are external, place them near the PCF8575 or ESP32. Place its decoupling capacitor very close to its power pins.
*   **TFT Connector Area:** Position the ESP32 and TFT connector to allow for short, direct data lines if possible.

### 2.2. Module Placement

*   **DC-DC Converter Module:** This module can be bulky and may generate heat. Place it with adequate space, possibly near an edge for good ventilation if needed. Connect its VIN/VOUT and GND pins to the main PCB using short, thick wires or headers if it's a separate daughterboard.
*   **PCF8575 I/O Expander Module:** Typically smaller. Can be mounted via headers directly onto the main PCB. Ensure its orientation allows for clean routing of the 9 indicator signals to it and I2C lines to the ESP32.
*   **ESP32-S3 Wroom Module:** This is an SMT module and will be soldered directly to the custom PCB. Follow its datasheet for recommended footprint and keep-out areas.

### 2.3. Signal Routing

*   **Isolation:**
    *   **Physical Separation:** Keep car-side (12V, noisy signals like `CAR_SIGNAL_X`, `CAR_SIGNAL_RPM`) traces and components physically as far as practical from the logic-side (3.3V/5V, ESP32, PCF8575, TFT) traces and components. The optocouplers are the bridge; aim for clear separation across them.
    *   **No Overlapping Traces (if possible):** Avoid running logic traces directly under or parallel and close to high-voltage car-side traces for extended lengths on a single-sided board.
*   **Trace Widths:**
    *   **Power & Ground:** Use significantly wider traces for `+12V_ACC`, `GND_CAR`, `+5V_LOGIC`, `+3.3V_LOGIC`, and `GND_LOGIC`. Aim for at least 50-100 mils (1.27-2.54mm) if space allows, especially for main supply lines.
    *   **Signal Traces:** Standard signal traces (e.g., 10-15 mils / 0.25-0.38mm) are usually fine for digital logic, I2C, and optocoupler inputs/outputs.
*   **Trace Lengths:**
    *   **RPM Signal:** Keep the trace from `CAR_SIGNAL_RPM` to `R_series_prot` and the subsequent path to `U_Opto_RPM` as short as reasonably possible to minimize noise pickup.
    *   **I2C Lines:** Keep SDA and SCL traces between ESP32 and PCF8575 relatively short. Avoid routing near noisy lines.
    *   **TFT Lines:** If using a parallel interface, these can be numerous. Try to keep them of similar lengths if possible, though this is less critical than for very high-speed memory buses. SPI lines for TFT should also be kept reasonably short.
*   **Optocoupler Layout:**
    *   Place the input resistor and input-side connections on one "side" of the optocoupler footprint and the output components (pull-up, connection to MCU/expander) on the other "side" to maintain good isolation.

### 2.4. Ground Plane (Single-Sided)

*   **Crucial for Noise Immunity:** On a single-sided PCB, a true ground plane is not possible. However, strive to make the ground network as robust as possible.
*   **Extensive Ground Traces:** Use wide, interconnected ground traces. Try to create a "star" or "meshed" ground pattern that connects all `GND_LOGIC` and `GND_CAR` points robustly.
*   **Fill Empty Areas:** Use copper fill (polygon pour) connected to `GND_LOGIC` to cover as much unused board area as possible. This will act as a pseudo-ground plane. Ensure the fill respects clearance rules around non-ground traces and pads.
*   **Connect GND_CAR and GND_LOGIC:** These should be connected at a single, solid point, typically at the output ground of the DC-DC converter module.

### 2.5. Connectors

*   **Car Power Input (`+12V_ACC`, `GND_CAR`):**
    *   **Screw Terminals:** Robust and good for direct wire connections from the car harness.
    *   **Locking Connectors (e.g., Molex Minifit Jr. or similar):** More secure if vibrations are a concern.
*   **Car Signal Inputs (Indicator signals, RPM signal):**
    *   **Screw Terminals:** Can be used if signals come as individual wires.
    *   **Multi-pin Headers (e.g., Dupont 0.1" pitch) with Housings:** Good for a bundle of signals if a mating connector is made for the car harness. Consider latching types for security.
    *   **JST XH (2.5mm pitch) or PH (2.0mm pitch):** Good board-to-wire connectors, often used for internal connections.
*   **TFT Display:**
    *   This often uses a specific FPC/FFC connector. Ensure you have the correct footprint and part for your display module.
    *   If it's a module with header pins, use female headers on the PCB.
*   **Module Connections (DC-DC, PCF8575 if on headers):**
    *   Standard 0.1" pitch male headers on the modules, female headers on the main PCB.

### 2.6. General Tips for Single-Sided Layout

*   **Minimize Jumper Wires:** Plan routing carefully. If jumpers are unavoidable, try to keep them short and for non-critical signals.
*   **Component Orientation:** Orient components to simplify routing. For example, align resistors so their pads lead naturally to the next component in the signal path.
*   **Clearances:** Maintain adequate clearance between traces and pads to avoid solder bridges, especially with wider power traces.
*   **Thermal Relief:** Use thermal relief pads for connections to large copper areas (like ground fills) to make soldering easier. Most PCB software does this automatically.
*   **Silkscreen:** Clear silkscreen labels for all components, connectors, and important signals (e.g., voltage test points, LED indicators if any) will be invaluable for assembly and debugging.

This guidance should provide a solid starting point for the PCB design process. Iteration and careful checking against the schematic (derived from the netlists) will be essential.
