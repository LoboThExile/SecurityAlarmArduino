#include <Arduino.h>
// ^^ Required for coding in VISUAL STUDIO CODE with PlatformIO. cuz yes
// However, if you use the Arduino IDE, you don't need to include this header file.

// M.Danish Hakimi
// 3 Cerdik
// For Project Based Learning
// Security alarm using light or lasers(lazers?).
// Using a photoresistor or light sensor to detect if the light is blocked. (used photoresistor for this one)

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


// [ CODE ]
// muhahaha anyways heres the code for the security system n such
//                                                                                    LED LIGHTS
const int silentIndicatorPin = 2; // LED to show silent mode status                   (yellow)
const int ledPin = 3; // red led.                                                     (red)
const int lightPin = 6; // LED OR Lazer for light sensor indication                   (white)
const int buzzerPin = 5; // buzzer
const int silenttogglePin = 7; // button for toggling silent mode
const int resetPin = 8; // reset button (reset system when tripped) 
const int armedPin = 9; // armed indicator LED (green when armed, off when tripped)   (green)

const int lightSensorPin = A0; // PhotoResistor connected to A0


const unsigned long blinkInterval = 150; // interval for blinking LED when tripped
const unsigned long debounceDelay = 50; // debounce time in ms

unsigned long previousMillis = 0; // for blink timing
unsigned long lastResetDebounceTime = 0; // for reset button debouncing
unsigned long lastSilentDebounceTime = 0; // for silent toggle button debouncing

bool blinkState = false; // LED blink state
bool tripped = false; // system tripped state

// debouncer reset button variables n such
bool lastResetState = HIGH; // last state of reset button
bool resetState = HIGH; // current state of reset button

// debouncer silent toggle button variables n such
bool lastSilentState = HIGH; // last state of silent toggle button
bool silentButtonState = HIGH; // current state of silent toggle button

bool silentMode = false; // silent mode state

void bootupsequence() { // bootup sound n such (call bootupsequence() to use)
  if (silentMode) {
    
    // on silent mode (no sound)
    digitalWrite(armedPin, HIGH);
    digitalWrite(lightPin, LOW);
    delay(100);
    digitalWrite(silentIndicatorPin, HIGH);
    delay(1250);
    digitalWrite(silentIndicatorPin, LOW);
    digitalWrite(armedPin, LOW);
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    digitalWrite(lightPin, HIGH);
    digitalWrite(armedPin, HIGH);
    return;
  }
  
  // normal (with sound)
  tone(buzzerPin, 1000);
  digitalWrite(armedPin, HIGH); 
  digitalWrite(lightPin, LOW);
  delay(50);
  noTone(buzzerPin);
  delay(50);
  digitalWrite(silentIndicatorPin, HIGH);
  delay(1250);
  digitalWrite(silentIndicatorPin, LOW);
  tone(buzzerPin, 500);
  delay(50);
  noTone(buzzerPin);
  digitalWrite(armedPin, LOW);
  digitalWrite(ledPin, HIGH);
  delay(50);
  tone(buzzerPin, 1500);
  delay(50);
  noTone(buzzerPin);
  digitalWrite(ledPin, LOW);
  digitalWrite(lightPin, HIGH);
  digitalWrite(armedPin, HIGH);
}

void setup() {
  Serial.begin(9600); // initialize serial communication for debugging if connected to a computer 

  // sets up pins
  pinMode(silenttogglePin, INPUT_PULLUP); 
  pinMode(ledPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(armedPin, OUTPUT);
  pinMode(silentIndicatorPin, OUTPUT); // silent mode indicator LED
  
  // check initial silent mode state
  silentMode = (digitalRead(silenttogglePin) == LOW);
  
  // set initial silent mode indicator
  digitalWrite(silentIndicatorPin, silentMode ? HIGH : LOW);
  
  // startupp!Q!!
  bootupsequence();
  
  Serial.println("Security Ready");
  Serial.print("Silent Mode: ");
  Serial.println(silentMode ? "ON" : "OFF");
}

void loop() {
  // read light sensor value
  int lightValue = analogRead(lightSensorPin);
  Serial.println(lightValue);
  unsigned long currentMillis = millis();

  // get tripped noob
  if (lightValue < 60 && !tripped) {
    tripped = true;
    Serial.println("gocha");
    previousMillis = currentMillis; // reset blink timer
  }

  // blink n beep when tripped
  if (tripped) {
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState ? HIGH : LOW);
      Serial.println("im tweakin out");
      digitalWrite(lightPin, LOW);
      
      // only sound if not in silent mode
      if (blinkState && !silentMode) {
        tone(buzzerPin, 500);
      } else {
        noTone(buzzerPin);
      }
    }
    delay(150); // stop it, im optimising it.
  } else {
    digitalWrite(ledPin, LOW);
    digitalWrite(armedPin, HIGH);
    noTone(buzzerPin);
  }

  // debouncer for reset button
  // resets the system if tripped
  // and reset button is pressed
  // also stops buzzer and turns off LED when reset is pressed
  // and resets the armed indicator LED
  // and runs bootup sequence again

  int resetReading = digitalRead(resetPin);
  
  if (resetReading != lastResetState) {
    lastResetDebounceTime = currentMillis;
  }
  
  if ((currentMillis - lastResetDebounceTime) > debounceDelay) {
    if (resetReading == LOW && resetState == HIGH && tripped) {
      // reset button only if tripped
      tripped = false;
      blinkState = false;
      digitalWrite(ledPin, LOW);
      digitalWrite(armedPin, LOW);
      noTone(buzzerPin);
      delay(250);
      digitalWrite(armedPin, HIGH);
      bootupsequence(); // run bootup sequence again to immerse user!!!! !Q!
      Serial.println("Resetting Complete."); // Might replace with a Display message later
    }
    resetState = resetReading;
  }
  lastResetState = resetReading;

  // debouncer for silent toggle button
  // toggles silent mode on/off
  // and updates the silent mode indicator LED
  // also stops buzzer if tripped and in silent mode

  int silentReading = digitalRead(silenttogglePin);
  
  if (silentReading != lastSilentState) {
    lastSilentDebounceTime = currentMillis;
  }
  
  if ((currentMillis - lastSilentDebounceTime) > debounceDelay) {
    if (silentReading == LOW && silentButtonState == HIGH) {
      // silent toggle button pressed
      silentMode = !silentMode;
      digitalWrite(silentIndicatorPin, silentMode ? HIGH : LOW); // update the silent mode indicator!!! (led)
      Serial.print("Silent Mode: ");
      Serial.println(silentMode ? "ON" : "OFF");
      
      // stop buzzer if tripped and in silent mode
      if (tripped && silentMode) {
        noTone(buzzerPin);
      }
    }
    silentButtonState = silentReading;
  }
  lastSilentState = silentReading;

  delay(100);
}
// if wood chuck could chuck wood, how much wood would a wood chuck chuck if a wood chuck could chuck wood?
// a wood chuck would chuck as much wood as a wood chuck could chuck if a wood chuck could chuck wood.