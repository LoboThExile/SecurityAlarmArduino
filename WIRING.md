# Security System Wiring Guide

## Microcontroller: Arduino Uno

---

## LEDs and Buzzer

- **Silent Indicator LED (Yellow)** → Arduino Pin **2** (with 220Ω resistor)  
  *Shows if silent mode is active.*
- **Red LED** → Arduino Pin **3** (with 220Ω resistor)  
  *Indicates alarm/tripped state.*
- **White LED / Laser** → Arduino Pin **6** (with 220Ω resistor)  
  *Used as a light source for the photoresistor or as a laser tripwire.*
- **Buzzer** → Arduino Pin **5**  
  *Sounds an alarm when the system is tripped.*

---

## Buttons
- **Silent Mode Toggle Button** → Arduino Pin **7** (use 10kΩ pull-down resistor)  
  *Toggles silent mode on or off.*
- **Reset Button** → Arduino Pin **8** (use 10kΩ pull-down resistor)  
  *Resets the alarm after it is tripped.*

---

## Armed Indicator
- **Green LED** → Arduino Pin **9** (with 220Ω resistor)  
  *Shows when the system is armed and ready.*

---

## Sensors
- **Photoresistor** → Arduino Pin **A0** (use 10kΩ resistor as voltage divider)  
  *Detects changes in light (e.g., if a beam is broken), triggering the alarm.*

---

## Power
- **5V** and **GND** rails on breadboard connected to Arduino **5V** and **GND**  
  *Distributes power to all components.*

---

> **Note:**  
> - Use appropriate resistors for all LEDs (typically 220Ω).
> - Use pull-down resistors (10kΩ) for buttons to ensure stable readings.
> - Double-check all connections before powering up.