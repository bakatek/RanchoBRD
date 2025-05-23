# ESP32-S3 Tentative Pin Allocations

This document provides suggested pin allocations for I2C communication with the PCF8575 I/O expander and for the conditioned RPM pulse input on an ESP32-S3 Wroom 1 N16R8 module.

## !! CRITICAL DISCLAIMER !!

**These are suggested pin allocations only. The user MUST verify these against their specific ESP32-S3 development board/module datasheet, their existing project codebase, and hardware connections.**

*   **TFT Display & Other Peripherals:** Ensure these pins do not conflict with pins already in use by the TFT display interface or any other peripherals in your project. TFT displays often use a significant number of GPIOs for parallel or SPI communication.
*   **Strapping Pins:** Some GPIO pins on the ESP32-S3 have special functions during boot (strapping pins) that determine boot mode, JTAG enablement, etc. While many can be used as general-purpose IOs after boot, their state at boot must be considered. Consult the ESP32-S3 datasheet for details on strapping pins (e.g., GPIO0, GPIO3, GPIO45, GPIO46).
*   **JTAG Pins:** Default JTAG pins (e.g., GPIO11-GPIO14 for SPI-based JTAG, or others for different JTAG configurations) might be configurable as GPIOs if JTAG is not needed, but this requires careful configuration.
*   **Module Specifics:** Pin availability and numbering can sometimes vary slightly between different ESP32-S3 modules or development boards. Always refer to the documentation for your specific hardware.

**Failure to verify and resolve conflicts can lead to non-functional hardware, programming issues, or unexpected behavior.**

## 1. I2C Pins for PCF8575 Communication

The PCF8575 I/O expander communicates with the ESP32-S3 via the I2C protocol. The ESP32-S3 has multiple I2C controllers (typically I2C0 and I2C1). The following pins are commonly used for I2C0:

*   **PCF8575 SDA (Serial Data) line   -> ESP32-S3 GPIO8**
*   **PCF8575 SCL (Serial Clock) line  -> ESP32-S3 GPIO9**

**Note on I2C Pull-up Resistors:**
*   The I2C protocol requires pull-up resistors on both the SDA and SCL lines.
*   Typical values are **2.2kOhm to 10kOhm (e.g., 4.7kOhm)**, connected to the **3.3V** supply rail.
*   Many PCF8575 modules come with these pull-up resistors already installed. Verify if your module includes them.
*   If not present on the module, you must add them externally. The ESP32-S3's internal pull-up resistors can sometimes be used, but external pull-ups are generally more robust and recommended for I2C.

## 2. RPM Pulse Input Pin

The conditioned RPM signal (a 0V to 3.3V logic-level pulse) requires a digital input pin on the ESP32-S3. This pin should ideally support interrupts or be usable with the ESP32-S3's Pulse Counter (PCNT) peripheral for accurate RPM measurement.

*   **Conditioned RPM Signal Output -> ESP32-S3 GPIO4**

**Considerations for RPM Input Pin (GPIO4):**
*   **General Purpose:** GPIO4 is generally available as a general-purpose input/output pin.
*   **Interrupts:** It supports external interrupts, which can be used to detect each RPM pulse.
*   **Pulse Counter (PCNT):** GPIO4 can typically be routed to one of the PCNT unit inputs, which is highly recommended for frequency/RPM counting as it offloads the CPU.
*   **Strapping Pin Check:** GPIO4 is not typically a critical strapping pin that would prevent normal boot if pulled low or high by the RPM input circuit's resting state. However, always verify with the specific ESP32-S3 module datasheet. The RPM conditioning circuit designed previously outputs HIGH (3.3V) when the RPM signal is inactive, and LOW when active.

## 3. Summary of Proposed Pin Allocations

| Signal                      | Proposed ESP32-S3 Pin | Notes                                       |
| :-------------------------- | :-------------------- | :------------------------------------------ |
| PCF8575 SDA                 | GPIO8                 | Common I2C0 SDA. Verify availability.     |
| PCF8575 SCL                 | GPIO9                 | Common I2C0 SCL. Verify availability.     |
| Conditioned RPM Input       | GPIO4                 | General purpose, PCNT capable. Verify.    |

**Again, thorough verification against your project's specific hardware and software configuration is essential before committing to these pin assignments.**
