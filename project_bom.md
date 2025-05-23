# Bill of Materials (BOM)

This Bill of Materials lists the components required for the automotive dashboard interface project, based on the designs in `indicator_circuit_design.md`, `rpm_input_circuit_design.md`, and `power_supply_design.md`.

## Resistors

| Qty | Value        | Type/Description      | Power Rating | Reference/Purpose                                       |
|-----|--------------|-----------------------|--------------|---------------------------------------------------------|
| 9   | 1.2 kOhms    | Resistor              | 1/4W         | Indicator light optocoupler LED current limiting          |
| 10  | 10 kOhms     | Resistor              | 1/4W         | 9x PCF8575 input pull-ups, 1x RPM ESP32 input pull-up   |
| 1   | 470 Ohms     | Resistor              | **0.5W**     | RPM input Zener diode series protection resistor        |
| 1   | 620 Ohms     | Resistor              | 1/4W         | RPM optocoupler LED current limiting (additional)       |
| 2   | 4.7 kOhms    | Resistor              | 1/4W         | I2C pull-up resistors (SDA, SCL) - if not on module   |

## Capacitors

| Qty | Value        | Type/Description      | Voltage Rating | Reference/Purpose                                       |
|-----|--------------|-----------------------|----------------|---------------------------------------------------------|
| 1   | 0.1 µF (100nF) | Ceramic Capacitor     | 50V            | RPM optocoupler LED noise filter                        |
| 1   | 470 µF       | Electrolytic Capacitor| 35V            | DC-DC buck converter input filter                       |
| ~3  | 0.1 µF (100nF) | Ceramic Capacitor     | 16V or higher  | IC decoupling (ESP32, PCF8575, other logic if needed) |

## Diodes

| Qty | Value    | Type/Description | Rating         | Reference/Purpose                                       |
|-----|----------|------------------|----------------|---------------------------------------------------------|
| 1   | 1N4744A  | Zener Diode      | 15V, 1W        | RPM input voltage spike clamping                        |
| 1   | 1N5822   | Schottky Diode   | 40V, 3A        | Main input reverse polarity protection                  |

## Optocouplers

| Qty | Value | Type/Description | Package | Reference/Purpose                                       |
|-----|-------|------------------|---------|---------------------------------------------------------|
| 10  | PC817 | Optocoupler      | DIP-4   | 9x Indicator signal isolation, 1x RPM signal isolation  |

## Modules & Main Components

| Qty | Component                       | Description                                         | Reference/Purpose                                       |
|-----|---------------------------------|-----------------------------------------------------|---------------------------------------------------------|
| 1   | ESP32-S3 Wroom 1 N16R8 Module   | Microcontroller unit                                | Main processor                                          |
| 1   | PCF8575 I/O Expander Module     | I2C I/O expander for indicator lights               | Reading dashboard indicator signals                     |
| 1   | DC-DC Buck Converter Module     | LM2596-based or similar, 12V to 5V step-down        | Main 5V power supply regulation                       |
| 1   | TFT Display                     | JC3248W535C (3.5 inch, 320x480, ST7796S SPI)        | Visual output                                           |

## Miscellaneous & Connectors

| Qty | Component                       | Description                                         | Reference/Purpose                                       |
|-----|---------------------------------|-----------------------------------------------------|---------------------------------------------------------|
| 1   | Automotive Fuse                 | Blade type, 2A                                      | Main input overcurrent protection                     |
| 1+  | Fuse Holder                     | Appropriate for automotive blade fuse               | To hold the main fuse                                   |
| TBD | Connectors                      | Various types (e.g., JST, Molex, Screw Terminals)   | For car power input, dashboard signals, RPM signal      |
| 1   | Custom PCB                      | Main board to host ESP32, optos, connectors, etc. | Integrating all components                              |

**Notes:**
*   "Qty ~3" for decoupling capacitors is an estimate; actual number depends on final PCB layout and specific needs of ICs.
*   Resistor power ratings are 1/4W unless otherwise specified (e.g., 0.5W for R_series_prot in RPM circuit).
*   Voltage ratings for capacitors are minimum suggestions; higher voltage ratings are generally acceptable.
*   "TBD" (To Be Determined) for connectors indicates that specific types and quantities will depend on final enclosure and wiring choices.
*   This BOM assumes through-hole components for resistors, diodes, and optocouplers unless a surface-mount version is explicitly chosen for the custom PCB design. Modules are assumed to be pre-built.
