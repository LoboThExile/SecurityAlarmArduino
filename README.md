# SecurityAlarmArduino

A Security Alarm system using an Arduino with lasers or an LED shining light onto a photoresistor to detect intrusions.

## The [experimental](https://github.com/LoboThExile/SecurityAlarmArduino/tree/experimental) build has camera support.  
**WARNING**: [Experimental](https://github.com/LoboThExile/SecurityAlarmArduino/tree/experimental) branch has not been tested yet.

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
- Camera module for recording footage **(not supported in this code)**  

---

## Camera Recommendations (Optional) (**ONLY IN [EXPERIMENTAL](https://github.com/LoboThExile/SecurityAlarmArduino/tree/experimental) BRANCH**) 

If you want to add camera support **ONLY IN [EXPERIMENTAL](https://github.com/LoboThExile/SecurityAlarmArduino/tree/experimental) BRANCH**, consider these options: 

- **Adafruit TTL Serial JPEG Camera**  (**Supported in [experimental](https://github.com/LoboThExile/SecurityAlarmArduino/tree/experimental) branch**)
  - Low resolution but uses fewer pins (4 pins: TX, RX, VCC, GND)  
  - Recommended for simpler wiring  

- **ArduCAM Mini 2MP**  (**Not supported in [experimental](https://github.com/LoboThExile/SecurityAlarmArduino/tree/experimental) branch**)
  - Higher resolution and more features  
  - Uses SPI interface and more pins (6-7 pins: SDA, SCL, VCC, GND, CS, MOSI, MISO)

---

## Notes

- This code currently does **not** support camera integration. Only in the **[Experimental](https://github.com/LoboThExile/SecurityAlarmArduino/tree/experimental) Branch** has camera integration.  
- Make sure to select a camera compatible with Arduino and your wiring capabilities.

---

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
