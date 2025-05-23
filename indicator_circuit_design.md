# Dashboard Indicator Input Conditioning Circuit Design

This document details the design of the input conditioning circuits for the 9 dashboard indicator signals, interfacing them with a PCF8575 I/O expander, which in turn connects to an ESP32.

## 1. General Design Parameters

*   **Opto-isolation:** PC817 optocouplers are used for each signal to electrically isolate the car's electrical system from the microcontroller system.
*   **Car Input Voltage:** Nominal 12V. The circuit is designed to operate reliably within a range of 11V to 14.5V.
*   **PCF8575 Power Supply:** The PCF8575 I/O expander is assumed to be powered at 3.3V (VCC_PCF).
*   **Optocoupler LED Forward Voltage (Vf):** Assumed to be 1.2V at the target operating current.
*   **Target Optocoupler LED Forward Current (If):** Aiming for approximately 8-11mA for good Current Transfer Ratio (CTR) and longevity.

## 2. Current Limiting Resistor (R_limit) for Optocoupler LED

The current limiting resistor is crucial for protecting the optocoupler's internal LED.

*   **Calculation Formula:** `R_limit = (V_source - Vf) / If`
*   **Chosen Nominal If for calculation:** ~9mA to allow for voltage variations.
    *   Using V_source = 12V (nominal car voltage):
        `R_limit = (12V - 1.2V) / 0.009A = 10.8V / 0.009A = 1200 Ohms (1.2 kOhms)`

*   **Verification across voltage range (11V to 14.5V) with R_limit = 1.2 kOhms:**
    *   At 11V: `If = (11V - 1.2V) / 1200 Ohms = 9.8V / 1200 Ohms = 8.17mA`
    *   At 12V: `If = (12V - 1.2V) / 1200 Ohms = 10.8V / 1200 Ohms = 9.00mA`
    *   At 14.5V: `If = (14.5V - 1.2V) / 1200 Ohms = 13.3V / 1200 Ohms = 11.08mA`
    *   This current range (8.17mA to 11.08mA) is well within typical safe operating limits for a PC817 and ensures reliable switching.

*   **Power Dissipation for R_limit:**
    *   Maximum current is 11.08mA.
    *   `P = I^2 * R = (0.01108A)^2 * 1200 Ohms = 0.00012277 * 1200 = 0.147 Watts`
    *   A standard 1/4W (0.25W) rated resistor is suitable.

**Selected R_limit: 1.2 kOhms, 1/4W** (for all 9 inputs)

## 3. Output Side Pull-up Resistor (R_pull_up) for PCF8575

The output of the optocoupler (collector-emitter) interfaces with the PCF8575 input pins. Each input pin requires a pull-up resistor to the PCF8575's VCC (3.3V).

*   When the optocoupler LED is ON, its transistor conducts, pulling the PCF8575 input LOW.
*   When the optocoupler LED is OFF, its transistor is non-conducting, and R_pull_up pulls the PCF8575 input HIGH.
*   A common value providing a good balance between noise immunity and switching speed, while keeping current consumption low, is 10kOhm.

**Selected R_pull_up: 10 kOhms** (connected from PCF8575 input pin to 3.3V, for all 9 outputs)
*   Power Dissipation for R_pull_up (when input is LOW):
    *   `P = V^2 / R = (3.3V)^2 / 10000 Ohms = 10.89V / 10000 Ohms = 0.001089W`
    *   A 1/8W or 1/4W resistor is suitable.

## 4. Circuit Configuration Details

### 4.1. Active-Low Car Signals

*   **Signals:**
    1.  Dégivrage lunette arrière (Rear defroster)
    2.  Voyant orange Frein (Amber Brake)
    3.  Voyant rouge Huile (Red Oil)
    4.  Voyant batterie rouge (Red Battery)
    5.  Voyant niveau essence orange (Amber Fuel)
    6.  Voyant rouge Frein (Red Brake)

*   **Circuit Diagram Description:**
    *   The **anode** of the PC817's internal LED is connected to the car's switched +12V accessory power line.
    *   The **1.2kOhm current-limiting resistor (R_limit)** is placed in series between the +12V accessory power and the optocoupler LED's anode.
    *   The **cathode** of the PC817 LED is connected directly to the car's specific indicator signal wire.
    *   The **collector** of the PC817's output NPN transistor is connected to an input pin on the PCF8575.
    *   The **10kOhm pull-up resistor (R_pull_up)** is connected between this PCF8575 input pin and the PCF8575's VCC (3.3V).
    *   The **emitter** of the PC817's output NPN transistor is connected to the system ground (GND), common with the PCF8575's ground.

*   **Operation:**
    *   **Indicator Light OFF:**
        *   Car signal wire is at +12V or floating.
        *   No significant current flows through the optocoupler LED (it's reverse biased or has insufficient forward voltage). LED is OFF.
        *   Optocoupler transistor is OFF (open circuit).
        *   R_pull_up pulls the PCF8575 input pin **HIGH (3.3V)**.
    *   **Indicator Light ON:**
        *   Car signal wire goes to 0V (ground).
        *   Current flows from +12V accessory power, through R_limit, through the optocoupler LED, to the 0V signal wire. LED is ON.
        *   Optocoupler transistor is ON (saturated).
        *   PCF8575 input pin is pulled **LOW (approx. 0V)** by the conducting transistor.

*   **Signal Logic at PCF8575:**
    *   Car signal Active (Light ON): 0V
    *   PCF8575 Input (Light ON): **LOW**
    *   This means the PCF8575 reads a LOW when the indicator light is ON.

### 4.2. Active-High Car Signals

*   **Signals:**
    1.  Voyant phare/éclairage tableau de bord (Headlights/Dash illumination)
    2.  Voyant clignotants (Turn signals - single indicator)
    3.  Voyant plein Phare (High beams)

*   **Circuit Diagram Description:**
    *   The **anode** of the PC817's internal LED is connected to the car's specific indicator signal wire.
    *   The **1.2kOhm current-limiting resistor (R_limit)** is placed in series between the car's signal wire and the optocoupler LED's anode.
    *   The **cathode** of the PC817 LED is connected to the car's chassis ground (0V).
    *   The **collector** of the PC817's output NPN transistor is connected to an input pin on the PCF8575.
    *   The **10kOhm pull-up resistor (R_pull_up)** is connected between this PCF8575 input pin and the PCF8575's VCC (3.3V).
    *   The **emitter** of the PC817's output NPN transistor is connected to the system ground (GND), common with the PCF8575's ground.

*   **Operation:**
    *   **Indicator Light OFF:**
        *   Car signal wire is at 0V.
        *   No current flows through the optocoupler LED. LED is OFF.
        *   Optocoupler transistor is OFF.
        *   R_pull_up pulls the PCF8575 input pin **HIGH (3.3V)**.
    *   **Indicator Light ON:**
        *   Car signal wire goes to +12V.
        *   Current flows from the +12V signal wire, through R_limit, through the optocoupler LED, to the car's chassis ground. LED is ON.
        *   Optocoupler transistor is ON.
        *   PCF8575 input pin is pulled **LOW (approx. 0V)**.

*   **Signal Logic at PCF8575:**
    *   Car signal Active (Light ON): +12V
    *   PCF8575 Input (Light ON): **LOW**
    *   This means the PCF8575 reads a LOW when the indicator light is ON. This is an inversion of the car's signal voltage level (HIGH on car -> LOW on PCF).

## 5. Summary of Resistor Values

*   **Optocoupler LED Current Limiting Resistor (R_limit):**
    *   Value: **1.2 kOhms**
    *   Power Rating: 1/4W or greater
    *   Quantity: 9 (one for each indicator)

*   **PCF8575 Input Pull-up Resistor (R_pull_up):**
    *   Value: **10 kOhms**
    *   Power Rating: 1/8W or greater
    *   Quantity: 9 (one for each indicator input to PCF8575)
    *   Connection: Between PCF8575 input pin and 3.3V (PCF8575 VCC)

This design ensures that all active indicator lights, whether active-low or active-high from the car's perspective, will result in a **LOW** signal at the corresponding PCF8575 input pin. This provides a consistent logic level for the ESP32 to interpret.
