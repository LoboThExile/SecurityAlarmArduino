# Security System Wiring

## Prerequisites

- Arduino Uno (or compatible)
- PN532 NFC Reader (configured for **I²C mode**)
- Breadboard + jumper wires
- Resistors: 220Ω (for LEDs), 10kΩ (for buttons, photoresistor divider)

## Wiring

| Component                   | Arduino Pin | Notes / Function                                   |
|-----------------------------|-------------|---------------------------------------------------|
 | Silent Indicator LED (Yellow) | 2           | Shows "silent mode" active, 220Ω resistor         |
| Red LED                     | 3           | Alarm triggered indicator, 220Ω resistor          |
| Buzzer                      | 5           | Audible alarm, can pulse/beep                     |
| White LED / Laser           | 6           | Extra alert / deterrent light, 220Ω resistor      |
| Green LED (Armed Status)    | 9           | Shows system armed, 220Ω resistor                 |
 | Silent Mode Button          | 7           | Toggles silent alarm, 10kΩ pull-down resistor     |
| Reset Button                | 8           | Resets alarm state, 10kΩ pull-down resistor       |
| Power Off Button            | A1          | Turns system off, 10kΩ pull-down resistor         |
 | Photoresistor (LDR)        | A0          | Light detection, wired as voltage divider (10kΩ)  |
| SDA                         | A4          | Data line, requires PN532 set to I²C mode         |
| SCL                         | A5          | Clock line, requires PN532 set to I²C mode        |
| VCC                         | 5V          | Powers PN532 module                               |
| GND                         | GND         | Ground                                            |
| RSTO / RSTPDN               | —           | Leave unconnected unless reset is required        |
| IRQ                         | —           | Not needed in I²C mode                            |
 | Breadboard Rail (+)        | 5V          | Common supply rail                                |
| Breadboard Rail (–)         | GND         | Common ground rail                                |

## Notes

- **Resistors**  
  - LEDs: 220Ω series resistor each  
  - Buttons: 10kΩ pull-down resistors  
  - LDR: 10kΩ for voltage divider  

- **PN532**  
  - Must be switched/jumpered to **I²C mode**  
  > Only **SDA (A4)** and **SCL (A5)** are needed for communication  
