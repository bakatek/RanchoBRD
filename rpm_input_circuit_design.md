# RPM Signal Input Conditioning Circuit Design

This document details the design of the input conditioning circuit for the car's RPM signal, intended for connection to an ESP32-S3 microcontroller. The RPM signal is assumed to be a 0V to +12V pulse derived from the ignition coil's low-voltage command line.

## 1. Design Goals

*   Protect the circuit and microcontroller from voltage spikes.
*   Provide electrical isolation using an optocoupler.
*   Filter high-frequency noise from the RPM signal.
*   Deliver a clean 0V to 3.3V logic-level pulse to the ESP32-S3.
*   Ensure the PC817 optocoupler is suitable for typical automotive RPM frequencies.

## 2. Optocoupler Selection and Speed Assessment

*   **Optocoupler:** PC817.
*   **Speed Assessment:**
    *   Typical automotive RPMs (e.g., up to 8000 RPM for 4-cyl, 15000 RPM for 8-cyl) result in pulse frequencies up to around 1 kHz.
    *   1 kHz corresponds to a pulse period of 1000µs. If the duty cycle is around 50%, the pulse width is 500µs.
    *   The PC817's typical rise and fall times (tr+tf) are in the range of 8µs to 36µs (sum of typicals often < 20µs).
    *   Since 500µs >> 20µs, the PC817 is deemed sufficiently fast for this application.

## 3. Circuit Components and Description

### 3.1. Input Stage (Car Side)

1.  **RPM Signal Source:** Ignition coil command signal (0V to +12V nominal pulses).
2.  **Series Protection Resistor (R_series_prot):**
    *   Value: **470 Ohms**
    *   Power Rating: **0.5W** (chosen for robustness against spikes, though 1/4W is sufficient for normal operation current).
    *   Function: Limits current into the Zener diode during voltage clamping events and forms part of the current limiting path for the optocoupler LED.
3.  **Zener Diode (D_zener):**
    *   Part Number: **1N4744A**
    *   Zener Voltage: 15V
    *   Power Rating: 1W
    *   Connection: Parallel to the input, after R_series_prot. Cathode to the signal line, Anode to Ground.
    *   Function: Clamps voltage spikes above ~15V, protecting the downstream components, especially the optocoupler LED.
4.  **LED Current Limiting Resistor (R_limit_led):**
    *   Value: **620 Ohms**
    *   Power Rating: **1/4W**
    *   Function: Works in conjunction with R_series_prot to set the forward current for the optocoupler LED to a safe and effective level (approx. 9.9mA with a 12V input pulse).
    *   Total series resistance for LED: R_series_prot + R_limit_led = 470 + 620 = 1090 Ohms.
5.  **Optocoupler (U_opto_rpm):**
    *   Part Number: **PC817**
    *   Connection:
        *   Anode (Pin 1): Connected to the junction of R_series_prot and R_limit_led, after R_limit_led.
        *   Cathode (Pin 2): Connected to Ground.
    *   Function: Provides electrical isolation between the car's electrical system and the microcontroller.
6.  **Noise Filter Capacitor (C_filter):**
    *   Value: **0.1 µF (100nF)**
    *   Voltage Rating: 50V (or higher) ceramic capacitor.
    *   Connection: In parallel with the optocoupler's input LED (between Anode, after R_limit_led, and Cathode).
    *   Function: Shunts high-frequency noise present on the RPM signal, preventing spurious optocoupler switching.

### 3.2. Output Stage (ESP32-S3 Side)

1.  **Optocoupler Output Transistor:**
    *   Collector (Pin 4 of PC817): Connected to the ESP32-S3 input pin.
    *   Emitter (Pin 3 of PC817): Connected to the ESP32-S3 system Ground.
2.  **Pull-up Resistor (R_pull_up_esp):**
    *   Value: **10 kOhms**
    *   Power Rating: 1/8W or 1/4W.
    *   Connection: Between the ESP32-S3 input pin and the ESP32-S3's 3.3V supply.
    *   Function: Pulls the ESP32-S3 input pin HIGH (to 3.3V) when the optocoupler transistor is OFF.

## 4. Circuit Operation

1.  **RPM Pulse LOW (0V from coil command):**
    *   No (or negligible) current flows through R_series_prot and R_limit_led.
    *   The optocoupler LED (PC817 Anode-Cathode) is OFF.
    *   The optocoupler's output transistor (Collector-Emitter) is OFF (high impedance).
    *   The pull-up resistor R_pull_up_esp (10kOhm) pulls the ESP32-S3 input pin **HIGH (3.3V)**.

2.  **RPM Pulse HIGH (+12V from coil command):**
    *   Current flows from the RPM signal line, through R_series_prot (470 Ohms) and R_limit_led (620 Ohms), into the PC817's LED anode. The LED's cathode is connected to ground.
    *   The LED forward current is approximately (12V - 1.2V_LED_Vf) / (470 + 620 Ohms) = 10.8V / 1090 Ohms ≈ 9.9mA.
    *   The optocoupler LED turns ON.
    *   The optocoupler's output transistor (Collector-Emitter) turns ON, effectively connecting its collector to its emitter (Ground).
    *   The ESP32-S3 input pin is pulled **LOW (approx. 0V)** by the conducting transistor, overriding the pull-up resistor.

3.  **Voltage Spike Event (e.g., > 15V on RPM line):**
    *   If a voltage spike exceeding ~15V occurs on the RPM signal line, the Zener diode (1N4744A) starts to conduct.
    *   The Zener clamps the voltage at its terminals to approximately 15V.
    *   R_series_prot (470 Ohms) limits the current flowing into the Zener diode, protecting it.
    *   The voltage seen by R_limit_led and the optocoupler LED will not exceed ~15V, protecting the LED from over-voltage and over-current.

## 5. Signal Logic at ESP32-S3

*   Car RPM Pulse Active (Ignition Coil Command ON): +12V
*   ESP32-S3 Input: **LOW (0V)**

This means the ESP32-S3 will detect a falling edge when an RPM pulse begins and a rising edge when it ends. The duration of the LOW state on the ESP32-S3 pin corresponds to the duration of the +12V portion of the RPM pulse.

## 6. Summary of Components

*   **Resistors:**
    *   R_series_prot: 470 Ohms, 0.5W
    *   R_limit_led: 620 Ohms, 1/4W
    *   R_pull_up_esp: 10 kOhms, 1/4W
*   **Diodes:**
    *   D_zener: 1N4744A (15V, 1W Zener Diode)
*   **Optocoupler:**
    *   U_opto_rpm: PC817
*   **Capacitor:**
    *   C_filter: 0.1 µF (100nF), 50V Ceramic Capacitor

This design provides a robust and isolated interface for the RPM signal to the ESP32-S3.
