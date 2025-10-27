#include <Arduino.h>
// ^^ Required for PlatformIO

#include <IRremote.h>

// Security alarm using light or lasers.
// Using a photoresistor (LDR) to detect if the light is blocked.
// Includes using a remote to turn off or on the system. (main_nano.cpp)

// [ COMPONENTS ]

// Microcontroller:
// Currently using Arduino Uno

// Resistors: 
// 220 ohm resistors (x4), 10K ohm resistor (x1)

// Other components:
// Breadboard (x1)
// Buzzer, LED(s) (Red, Yellow, Green, White Or laser)
// Photoresistor (x1), Button (x3), Buzzer (x1)

// Optional: 
// Lazer (x1) (if you want to use a laser instead of a LED.)

// [ CODE ]
const int silentIndicatorPin = 2; // LED to show silent mode status                   (yellow)
const int ledPin = 3; // red led.                                                    (red)
const int statusled = 4; // Status led.
const int lightPin = 6; // Shine light at the LDR                   (white)
const int buzzerPin = 5; // The buzzer
const int silenttogglePin = 7; // Button for toggling Silent mode.
const int resetPin = 8; // Button for resetting.
const int armedPin = 9; // LED for showing the user that the system is armed.   (green)
const int powerOffPin = A1; // Power off button. Picked A1 because we were running out of pins.
const int lightSensorPin = A0; // PhotoResistor (LDR) connected to A0
const int irReceiverPin = 11; // IR receiver connected to digital pin 11

const int TRIGTRESHOLD = 600; // Trigger Treshold for alarm. If LDR reports below treshold, Trigger alarm.

const unsigned long blinkInterval = 150; // Interval for blinking LED when tripped
const unsigned long debounceDelay = 50; // Debounce time in ms

unsigned long previousMillis = 0; // For blink timing
unsigned long lastResetDebounceTime = 0; // For reset button debouncing
unsigned long lastSilentDebounceTime = 0; // For silent toggle button debouncing
unsigned long lastPowerOffDebounceTime = 0; // For power off button debouncing

bool blinkState = false; // LED blink state
bool tripped = false; // System tripped state
bool systemOn = true; // System power state
bool firstBoot = true; // Flag for first boot initialization

// Debouncer for reset button variables
bool lastResetState = HIGH;
bool lastSilentState = HIGH;
bool silentButtonState = HIGH;

// Debouncer power off button variables
bool lastPowerOffState = HIGH;
bool powerOffButtonState = HIGH;

bool silentMode = false; // The state of silent mode
bool resetState = HIGH; // The state of reset
IRrecv irrecv(irReceiverPin);
decode_results results;

void bootupsequence() { // Boot up sequence.
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
    digitalWrite(statusled, LOW);
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
  digitalWrite(statusled, LOW);
  delay(100); // Extra delay here because LDR for some reason takes old reading??
}

void powerOffSequence() {
  Serial.println("Powering off system...");
  digitalWrite(lightPin, LOW);
  digitalWrite(armedPin, LOW);
  digitalWrite(silentIndicatorPin, LOW);
  digitalWrite(statusled, HIGH);
  noTone(buzzerPin);
  if (!silentMode) {
    tone(buzzerPin, 3000, 100);
    delay(150);
    tone(buzzerPin, 2000, 100);
    delay(150);
    tone(buzzerPin, 500, 200);
    delay(250);
  }

  tripped = false;
  blinkState = false;
  systemOn = false;
  firstBoot = true;

  //digitalWrite(ledPin, HIGH);
  digitalWrite(ledPin, LOW);

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
  unsigned long currentMillis = millis(); // Get current ms
  int powerOffReading = digitalRead(powerOffPin);

  // Always check IR remote, even if system is off
  if (IrReceiver.decode()) {
    Serial.print("IR code received: ");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    // Info received is always garbage... No idea how to fix this except adding a security risk.
    // Currently, allows ALL data even if its garbage or totally random.
    if (IrReceiver.decodedIRData.decodedRawData) { 
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

  if (lightValue < TRIGTRESHOLD && !tripped) {
    tripped = true;
    Serial.println("Tripped!");
    previousMillis = currentMillis;
  }

  if (tripped) {
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState ? HIGH : LOW);
      Serial.println("Alarm!!");
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
// Agh..
// No idea what the others did to my wiring.. And I have no idea what they done to the LED part. 
// Why can't they just leave electronics to electronics people?