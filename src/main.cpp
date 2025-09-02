#include <Arduino.h>
// ^^ Required for PlatformIO

#include <Adafruit_PN532.h>
// ^^ Required for NFC
#include <Wire.h>
// ^^ Depenency for NFCs

#include <EEPROM.h>
// ^^ For saving values.

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

Adafruit_PN532 nfc;

// Data struct to save
struct DeviceData {
  uint32_t deviceID;
  uint32_t password;
  byte cardUID[7];
  byte uidLength;
  bool registered;
};

// EEPROM location
const int EEPROM_ADDR = 0;
DeviceData device;

void saveDeviceData() {
  EEPROM.put(EEPROM_ADDR, device);
}

void loadDeviceData() {
  EEPROM.get(EEPROM_ADDR, device);
  if (device.deviceID == 0xFFFFFFFF || device.deviceID == 0) {
    device.registered = false;
  }
}

uint32_t generateRandomID() {
  return (uint32_t)random(100000, 999999); // basic 6-digit ID
}

uint32_t makePasswordFromID(uint32_t id) {
  return id ^ 0xABCDEF; // simple transformation
}

void waitForCardAndRegister() {
  Serial.println("Please tap an NFC card...");

  boolean success;
  uint8_t uid[7];
  uint8_t uidLength;

  while (true) {
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
    if (success) {
      device.uidLength = uidLength;
      for (byte i = 0; i < uidLength; i++) {
        device.cardUID[i] = uid[i];
      }
      device.registered = true;
      saveDeviceData();

      Serial.print("Card registered! UID: ");
      for (byte i = 0; i < uidLength; i++) {
        Serial.print(uid[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      break;
    }
  }
}

bool checkCard() {
  uint8_t uid[7];
  uint8_t uidLength;

  // Look for a card
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) {
    Serial.print("Card detected: ");
    for (uint8_t i = 0; i < uidLength; i++) {
      Serial.print(uid[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Compare length first
    if (uidLength != device.uidLength) {
      Serial.println("Card length mismatch");
      return false;
    }

    // Compare each byte of UID
    for (uint8_t i = 0; i < uidLength; i++) {
      if (uid[i] != device.cardUID[i]) {
        Serial.println("Card UID mismatch");
        return false;
      }
    }

    Serial.println("Card matched");
    return true;
  }

  return false; // No card detected
}


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

// [ INIT ]

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);
  Serial.println("Security System Initializing...");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("Didn't find PN53x board");
  }

    // Configure the board to read RFID tags
  nfc.SAMConfig();
  loadDeviceData();

  if (!device.registered) {
    Serial.println("No device data found. Generating new one...");
    device.deviceID = generateRandomID();
    device.password = makePasswordFromID(device.deviceID);
    device.registered = false;
    saveDeviceData();

    Serial.print("Generated ID: "); Serial.println(device.deviceID);
    Serial.print("Generated Password: "); Serial.println(device.password);

    waitForCardAndRegister();
  } else {
    Serial.println("Loaded device data:");
    Serial.print("Device ID: "); Serial.println(device.deviceID);
    Serial.print("Password : "); Serial.println(device.password);

    Serial.print("Card UID : ");
    for (int i = 0; i < device.uidLength; i++) {
      Serial.print(device.cardUID[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }


  pinMode(silenttogglePin, INPUT_PULLUP); 
  pinMode(ledPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(armedPin, OUTPUT);
  pinMode(silentIndicatorPin, OUTPUT);
  pinMode(powerOffPin, INPUT_PULLUP);

  powerOnSequence();
}

// [ LOOP ]

void loop() {
  unsigned long currentMillis = millis();
  int powerOffReading = digitalRead(powerOffPin);

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
  
  if (checkCard()) {
    if (systemOn) {
      powerOffSequence();
    } else {
      powerOnSequence();
    }
    delay(1000);
  }

  delay(50);
}
// .. im dying. 
