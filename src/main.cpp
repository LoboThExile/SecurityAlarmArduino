#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

// ---------- NFC SETUP ----------
#define PN532_IRQ   2
#define PN532_RESET 3
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

// Your known authorized UID (update with your card’s UID)
// List of authorized UIDs (add more if needed)
uint8_t authorizedUIDs[][4] = {
  {0xE4, 0x98, 0xB0, 0x05},  // Card 1
  {0xFE, 0xBB, 0xBC, 0x02}   // Card 2 (  )
};

const uint8_t numAuthorizedCards = sizeof(authorizedUIDs) / sizeof(authorizedUIDs[0]);
const uint8_t authorizedLength = 4;  // Each card has 4-byte UID

bool nfcAuthorized = false;
unsigned long nfcAuthTimeout = 0;
const unsigned long authDuration = 10000; // 10 seconds of authorization

// ---------- PIN SETUP ----------
const int silentIndicatorPin = 4;
const int ledPin = 5;
const int lightPin = 6;
const int buzzerPin = 7;
const int silenttogglePin = 8;
const int resetPin = 9;
const int armedPin = 10;
const int powerOffPin = A1;
const int lightSensorPin = A0;

const unsigned long blinkInterval = 150;
const unsigned long debounceDelay = 50;

unsigned long previousMillis = 0;
unsigned long lastResetDebounceTime = 0;
unsigned long lastSilentDebounceTime = 0;
unsigned long lastPowerOffDebounceTime = 0;

bool blinkState = false;
bool tripped = false;
bool systemOn = true;
bool firstBoot = true;

bool lastResetState = HIGH;
bool lastSilentState = HIGH;
bool silentButtonState = HIGH;

bool lastPowerOffState = HIGH;
bool powerOffButtonState = HIGH;

bool silentMode = false;
bool resetState = HIGH;

// ---------- FUNCTIONS ----------

bool checkNFC() {
  uint8_t uid[7];
  uint8_t uidLength;

  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) return false;

  if (uidLength != authorizedLength) return false;

  // Compare against each authorized card
  for (uint8_t card = 0; card < numAuthorizedCards; card++) {
    bool match = true;
    for (uint8_t i = 0; i < authorizedLength; i++) {
      if (uid[i] != authorizedUIDs[card][i]) {
        match = false;
        break;
      }
    }
    if (match) return true;  // Found a match!
  }

  // If we got here, card was not recognized
  Serial.print("Unknown card UID: ");
  for (uint8_t i = 0; i < uidLength; i++) {
    Serial.print("0x"); Serial.print(uid[i], HEX);
    if (i < uidLength - 1) Serial.print(", ");
  }
  Serial.println();
  return false;
}


void bootupsequence() {
  if (silentMode) {
    digitalWrite(armedPin, HIGH);
    digitalWrite(lightPin, LOW);
    delay(500);
    digitalWrite(silentIndicatorPin, HIGH);
    firstBoot = false;
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
  firstBoot = false;
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
  if (!nfcAuthorized) return; // 🔒 Block if not authorized

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
  Serial.println("System powered off. Scan NFC to restart.");
}

void powerOnSequence() {
  if (!nfcAuthorized) return; // 🔒 Block if not authorized

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

  // NFC init
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("Didn't find PN532!");
    while (1);
  }
  nfc.SAMConfig();
  Serial.println("Waiting for NFC card...");
}

void loop() {
  unsigned long currentMillis = millis();

  // ---------- NFC AUTH ----------
  if (checkNFC()) {
    nfcAuthorized = true;
    nfcAuthTimeout = currentMillis + authDuration;
    Serial.println("NFC Authorized! System unlocked.");
  }

  if (nfcAuthorized && currentMillis > nfcAuthTimeout) {
    nfcAuthorized = false;
    Serial.println("Authorization expired. System locked.");
  }

  if (!systemOn) {
    delay(100);
    return;
  }

  // ---------- IF NOT AUTHORIZED, BLOCK INPUTS ----------
  if (!nfcAuthorized) {
    digitalWrite(ledPin, LOW);
    digitalWrite(armedPin, LOW);
    noTone(buzzerPin);
    return; // Skip actions until NFC scanned
  }

  // ---------- NORMAL LOGIC ----------
  int powerOffReading = digitalRead(powerOffPin);
  if (powerOffReading != lastPowerOffState) lastPowerOffDebounceTime = currentMillis;
  if ((currentMillis - lastPowerOffDebounceTime) > debounceDelay) {
    if (powerOffReading == LOW && powerOffButtonState == HIGH) {
      if (systemOn) powerOffSequence();
      else powerOnSequence();
    }
    powerOffButtonState = powerOffReading;
  }
  lastPowerOffState = powerOffReading;

  int lightValue = analogRead(lightSensorPin);
  if (lightValue < 150 && !tripped) {
    tripped = true;
    previousMillis = currentMillis;
  }

  if (tripped) {
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState ? HIGH : LOW);
      digitalWrite(lightPin, LOW);
      digitalWrite(armedPin, LOW);

      if (blinkState && !silentMode) tone(buzzerPin, 500);
      else noTone(buzzerPin);
    }
    delay(150);
  } else {
    digitalWrite(ledPin, LOW);
    digitalWrite(armedPin, HIGH);
    noTone(buzzerPin);
  }

  int resetReading = digitalRead(resetPin);
  if (resetReading != lastResetState) lastResetDebounceTime = currentMillis;
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
    }
    resetState = resetReading;
  }
  lastResetState = resetReading;

  int silentReading = digitalRead(silenttogglePin);
  if (silentReading != lastSilentState) lastSilentDebounceTime = currentMillis;
  if ((currentMillis - lastSilentDebounceTime) > debounceDelay) {
    if (silentReading == LOW && silentButtonState == HIGH) {
      silentMode = !silentMode;
      digitalWrite(silentIndicatorPin, silentMode ? HIGH : LOW);
      if (tripped && silentMode) noTone(buzzerPin);
    }
    silentButtonState = silentReading;
  }
  lastSilentState = silentReading;

  delay(50);
}
