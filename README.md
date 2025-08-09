# SecurityAlarmArduino
A Security Alarm using an arduino by using lasers or an led to shine light towards a PhotoResistor.


// [ COMPONENTS ]

// Microcontroller:
// Arduino Nano, Arduino Uno, or ESP32 (Good for with related stuff.) any other microcontroller that supports C++. Using Arduino Uno for this case.
// Buzzer, LED (Red, Yellow, Green, White/Laser(Lazer?)). (Bring extra LEDs just in case.)

// Resistors: 
// (NOTE: Pack extra resistors just in case. Or bring the whole set at this point.)
// 220 ohm resistors (x4), 10K ohm resistor (x1)(Still bring extra.)

// Other components:
// Breadboard (Duhh..) <---------------- VERYY IMPORTANTT!!!!!!!!!!!!!!!!!!!!!!!!!!
// Photoresistor (x1), Button (x2), Buzzer (x1)
// Tons and tons of wires. Jumper wires also needed to transfer data / power into the breadboard.

// Optional: 
// Lazer (x1) (if you want to use a laser instead of a LED.),
// Camera (Please look down below, Very important) (For recording the security footage if you want to.)
//( However, this code does not have camera support.) <--------------------- [ ALSO IMPORTANT ]

// ^^ Please buy or use a camera that supports C++ and Arduino. <--------------------- [ ALSO ALSO IMPORTANT ]
// Like Adafruit TTL serial JPEG camera (Low resolution however uses less pins) (4 Pins, TX, RX, VCC, GND) (Recommended as it uses less pins)
// Or Arducam Mini 2MP (Higher resolution and more features but uses more pins (6-7) (SDA, SCL, VCC, GND, CS, MOSI, MISO) (SPI interface))

