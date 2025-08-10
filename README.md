# SecurityAlarmArduino (Experimental)

A Security Alarm system using an Arduino with lasers or an LED shining light onto a photoresistor to detect intrusions.

## Has camera support (Experimental)
This code **HAS camera support** however it is **untested** and **unstable**

### [Wiring](WIRING.md)

### Made for PBL.

---

## Components

### Microcontroller
- Arduino Nano, Arduino Uno, ESP32, or any microcontroller supporting C++  
- (This project uses Arduino Uno for example)
- Later, We might make an android app to connect to the wifi so it could connect to the servers (Using ESP32)

### LEDs and Sound
- Buzzer (x1)  
- LEDs: Red, Yellow, Green, White (or Laser module instead of White LED)  
- Extra LEDs recommended for testing

### Resistors
- 220 Ω resistors (x4/x3)  
- 10 KΩ resistor (x1)  
- You should bring more just in case.

### Other Hardware
- Breadboard (very important for wiring)  
- Photoresistor (Light dependent resistor, x1)  <--- Bring a couple.
- Buttons (x2)  
- Jumper wires for connections <-- Bring a bunch.

### Optional
- Laser module (if you want to replace the white LED with a laser) <-- Later in this project, Will use laser.  
- Camera module for recording footage **Only supported in Expermental branch** 

---

## Camera Recommendations (Optional) **Only supported in Experimental branch**

If you want to add camera support, consider these options: 

- **Adafruit TTL Serial JPEG Camera**  **Supported in Expermental branch**
  - Low resolution but uses fewer pins (4 pins: TX, RX, VCC, GND)  
  - Recommended for simpler wiring  

- **ArduCAM Mini 2MP**  **Unsupported for now.**
  - Higher resolution and more features  
  - Uses SPI interface and more pins (6-7 pins: SDA, SCL, VCC, GND, CS, MOSI, MISO)

---

## Notes

- This code **HAS camera support** however it is **untested** and **unstable**
- Make sure to select a camera compatible with Arduino and your wiring capabilities.

---


## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
