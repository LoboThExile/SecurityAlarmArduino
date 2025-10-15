#include <Arduino.h>
// ^^ Required for PlatformIO

#include <IRremote.h>

// 3 Cerdik
// For Project Based Learning
// Security alarm using light or lasers(lazers?).
// Using a photoresistor or light sensor to detect if the light is blocked. (used photoresistor for this one)

// [ COMPONENTS ]

// Microcontroller:
// Arduino Nano, Arduino Uno, ESP32 (Good for future-proofing.), or any other microcontroller that supports C++. Using Arduino Uno for this case.
// Buzzer, LED (Red, Yellow, Green, White/Laser(Lazer?)). (Bring extra LEDs just in case.)

// Resistors: 
// (NOTE: Pack extra resistors just in case. Or bring the whole set at this point.)
// 220 ohm resistors (x4), 10K ohm resistor (x1)(Still bring extra for both.)

// Other components:
// Breadboard (Duhh..) <---------------- VERYY IMPORTANTT!!!!!!!!!!!!!!!!!!!!!!!!!!
// Photoresistor (x1), Button (x3), Buzzer (x1)  // Updated: now 3 buttons (reset, silent, power off)
// Tons and tons of wires. Jumper wires also needed to transfer data / power into the breadboard.

// Optional: 
// Lazer (x1) (if you want to use a laser instead of a LED.)

// [ CODE ]
const int silentIndicatorPin = 2; // LED to show silent mode status                   (yellow)
const int ledPin = 3; // red led.                                                     (red)
const int lightPin = 6; // LED OR Lazer for light sensor indication                   (white)
const int buzzerPin = 5; // buzzer
const int silenttogglePin = 7; // button for toggling silent mode
const int resetPin = 8; // reset button (reset system when tripped) 
const int armedPin = 9; // armed indicator LED (green when armed, off when tripped)   (green)
const int powerOffPin = A1; // power off button (acts like first boot)
const int lightSensorPin = A0; // PhotoResistor connected to A0
const int irReceiverPin = 11; // IR receiver connected to digital pin 4

const unsigned long blinkInterval = 150; // interval for blinking LED when tripped
const unsigned long debounceDelay = 50; // debounce time in ms

unsigned long previousMillis = 0; // for blink timing
unsigned long lastResetDebounceTime = 0; // for reset button debouncing
unsigned long lastSilentDebounceTime = 0; // for silent toggle button debouncing
unsigned long lastPowerOffDebounceTime = 0; // for power off button debouncing

bool blinkState = false; // LED blink state
bool tripped = false; // system tripped state
bool systemOn = true; // system power state
bool firstBoot = true; // flag for first boot initialization

// debouncer reset button variables n such
bool lastResetState = HIGH;
bool lastSilentState = HIGH;
bool silentButtonState = HIGH;

// debouncer power off button variables
bool lastPowerOffState = HIGH;
bool powerOffButtonState = HIGH;

bool silentMode = false; // silent mode state
bool resetState = HIGH; // current state of reset button

IRrecv irrecv(irReceiverPin);
decode_results results;

void bootupsequence() {
  if (silentMode) {
    digitalWrite(armedPin, HIGH);
    digitalWrite(lightPin, LOW);
    delay(500);
    digitalWrite(silentIndicatorPin, HIGH);

    if (firstBoot) {
      firstBoot = false;
    }

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

  tone(buzzerPin, 1000);
  digitalWrite(armedPin, HIGH); 
  digitalWrite(lightPin, LOW);
  delay(50);
  noTone(buzzerPin);
  delay(500);
  digitalWrite(silentIndicatorPin, HIGH);

  if (firstBoot) {
    firstBoot = false;
  }

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

void powerOffSequence() {
  Serial.println("Powering off system...");
  digitalWrite(lightPin, LOW);
  digitalWrite(armedPin, LOW);
  digitalWrite(silentIndicatorPin, LOW);
  noTone(buzzerPin);

  if (!silentMode) {
    tone(buzzerPin, 1500, 100);
    delay(150);
    tone(buzzerPin, 1000, 100);
    delay(150);
    tone(buzzerPin, 500, 200);
    delay(250);
  }

  tripped = false;
  blinkState = false;
  systemOn = false;
  firstBoot = true;

  digitalWrite(ledPin, HIGH);

  Serial.println("System powered off. Press power button to restart.");
}

void powerOnSequence() {
  Serial.println("Powering on system...");
  systemOn = true;

  silentMode = (digitalRead(silenttogglePin) == LOW);
  digitalWrite(silentIndicatorPin, silentMode ? HIGH : LOW);
  digitalWrite(ledPin, LOW);

  bootupsequence();

  Serial.println("Security Ready");
  Serial.print("Silent Mode: ");
  Serial.println(silentMode ? "ON" : "OFF");
}

void setup() {
  Serial.begin(9600);
  Serial.println("Security System Initializing...");

  pinMode(silenttogglePin, INPUT_PULLUP); 
  pinMode(ledPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(armedPin, OUTPUT);
  pinMode(silentIndicatorPin, OUTPUT);
  pinMode(powerOffPin, INPUT_PULLUP);

  IrReceiver.begin(irReceiverPin, ENABLE_LED_FEEDBACK); // IRremote v4.x

  powerOnSequence();
}

void loop() {
  unsigned long currentMillis = millis();
  int powerOffReading = digitalRead(powerOffPin);

  // Always check IR remote, even if system is off
  if (IrReceiver.decode()) {
    Serial.print("IR code received: ");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    // Replace 0xFFA25D with your remote's ON/OFF button code
    if (IrReceiver.decodedIRData.decodedRawData) { // Example: Power button
      systemOn = !systemOn;
      if (systemOn) {
        powerOnSequence();
      } else {
        powerOffSequence();
      }
      delay(500); // Debounce IR remote
    }
    IrReceiver.resume(); // Receive the next value
  }

  if (powerOffReading != lastPowerOffState) {
    lastPowerOffDebounceTime = currentMillis;
  }

  if ((currentMillis - lastPowerOffDebounceTime) > debounceDelay) {
    if (powerOffReading == LOW && powerOffButtonState == HIGH) {
      if (systemOn) {
        powerOffSequence();
      } else {
        powerOnSequence();
      }
    }
    powerOffButtonState = powerOffReading;
  }
  lastPowerOffState = powerOffReading;

  if (!systemOn) {
    delay(100);
    return;
  }

  int lightValue = analogRead(lightSensorPin);
  Serial.println(lightValue);

  if (lightValue < 150 && !tripped) {
    tripped = true;
    Serial.println("gocha");
    previousMillis = currentMillis;
  }

  if (tripped) {
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState ? HIGH : LOW);
      Serial.println("im tweakin out");
      digitalWrite(lightPin, LOW);
      digitalWrite(armedPin, LOW);

      if (blinkState && !silentMode) {
        tone(buzzerPin, 500);
      } else {
        noTone(buzzerPin);
      }
    }
    delay(150);
  } else {
    digitalWrite(ledPin, LOW);
    digitalWrite(armedPin, HIGH);
    noTone(buzzerPin);
  }

  int resetReading = digitalRead(resetPin);

  if (resetReading != lastResetState) {
    lastResetDebounceTime = currentMillis;
  }

  if ((currentMillis - lastResetDebounceTime) > debounceDelay) {
    if (resetReading == LOW && resetState == HIGH && tripped) {
      tripped = false;
      blinkState = false;
      digitalWrite(ledPin, LOW);
      digitalWrite(armedPin, LOW);
      noTone(buzzerPin);
      delay(250);
      digitalWrite(armedPin, HIGH);
      bootupsequence();
      Serial.println("Resetting Complete.");
    }
    resetState = resetReading;
  }
  lastResetState = resetReading;

  int silentReading = digitalRead(silenttogglePin);

  if (silentReading != lastSilentState) {
    lastSilentDebounceTime = currentMillis;
  }

  if ((currentMillis - lastSilentDebounceTime) > debounceDelay) {
    if (silentReading == LOW && silentButtonState == HIGH) {
      silentMode = !silentMode;
      digitalWrite(silentIndicatorPin, silentMode ? HIGH : LOW);
      Serial.print("Silent Mode: ");
      Serial.println(silentMode ? "ON" : "OFF");

      if (tripped && silentMode) {
        noTone(buzzerPin);
      }
    }
    silentButtonState = silentReading;
  }
  lastSilentState = silentReading;

  delay(50);
}
// if wood chuck could chuck wood, how much wood would a wood chuck chuck if a wood chuck could chuck wood?
// a wood chuck would chuck as much wood as a wood chuck could chuck if a wood chuck could chuck wood.
// why would a wood chuck chuck wood if a wood chuck could chuck wood?
// because wood chuck could chuck wood if a wood chuck could chuck wood.
// if they could chuck wood, they would chuck wood.11:13 AM
