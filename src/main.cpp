#include <SoftwareSerial.h>
#include <SdFat.h>
#include <SPI.h>
// ^^ Required for camera and SD card functionality.

#include <Arduino.h>
// ^^ Required for coding in VISUAL STUDIO CODE with PlatformIO. cuz yes
// However, if you use the Arduino IDE, you don't need to include this header file.

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
// Photoresistor (x1), Button (x2), Buzzer (x1)
// Tons and tons of wires. Jumper wires also needed to transfer data / power into the breadboard.

// Optional: 
// Lazer (x1) (if you want to use a laser instead of a LED.),
// Camera (For recording the security footage if you want to.)

// Camera Components (Optional):
// Note: The camera is optional, However if you want to use it, it will take most of the pins on the board.

// Adafruit TTL serial JPEG camera (4 Pins, TX, RX, VCC, GND) 
// SD Card Module (CS, MOSI, MISO, SCK, VCC, GND)
// MicroSD Card (formatted as FAT16 or FAT32) <-- Important for the SD card to work properly.

// Photos saved to SD card in /PHOTOS/ folder as SEC1.JPG, SEC2.JPG, etc.
// System will automatically detect if camera and SD card are available and disable features if not found.

// Camera Connections:
// Camera TX -> Arduino Pin 10 (RX via SoftwareSerial)
// Camera RX -> Arduino Pin 11 (TX via SoftwareSerial)  
// Camera VCC -> Arduino 5V
// Camera GND -> Arduino GND
//
// SD Card Connections:
// CS -> Arduino Pin 4
// MOSI -> Arduino Pin 11 (shared with camera TX)
// MISO -> Arduino Pin 12
// SCK -> Arduino Pin 13
// VCC -> Arduino 5V
// GND -> Arduino GND


SdFat SD; // Create an instance of the SdFat class for SD card

// [ CODE ]
// muhahaha anyways heres the code for the security system n such and with CAMERA SUPPORT!
//                                                                                    LED LIGHTS
const int silentIndicatorPin = 2; // LED to show silent mode status                   (yellow)
const int ledPin = 3; // red led.                                                     (red)
const int lightPin = 6; // LED OR Lazer for light sensor indication                   (white)
const int buzzerPin = 5; // buzzer
const int silenttogglePin = 7; // button for toggling silent mode
const int resetPin = 8; // reset button (reset system when tripped) 
const int armedPin = 9; // armed indicator LED (green when armed, off when tripped)   (green)

// Camera and SD card pins
const int cameraRX = 10; // Camera TX connects here
const int cameraTX = 11; // Camera RX connects here  
const int sdCS = 4; // SD card chip select pin

const int lightSensorPin = A0; // PhotoResistor connected to A0

// Camera setup
SoftwareSerial cameraSerial(cameraRX, cameraTX);
bool cameraAvailable = false;
bool sdAvailable = false;
int photoCount = 0;

const unsigned long blinkInterval = 150; // interval for blinking LED when tripped
const unsigned long debounceDelay = 50; // debounce time in ms

unsigned long previousMillis = 0; // for blink timing
unsigned long lastResetDebounceTime = 0; // for reset button debouncing
unsigned long lastSilentDebounceTime = 0; // for silent toggle button debouncing

bool blinkState = false; // LED blink state
bool tripped = false; // system tripped state
bool photoTaken = false; // flag to prevent multiple photos per trip

// debouncer reset button variables n such
bool lastResetState = HIGH; // last state of reset button
bool resetState = HIGH; // current state of reset button

// debouncer silent toggle button variables n such
bool lastSilentState = HIGH; // last state of silent toggle button
bool silentButtonState = HIGH; // current state of silent toggle button

bool silentMode = false; // silent mode state

// No idea what this is but it said i needed it so i left it here.
// Camera commands for Adafruit TTL Serial JPEG Camera
const byte cameraReset[] = {0x56, 0x00, 0x26, 0x00};
const byte takePicture[] = {0x56, 0x00, 0x36, 0x01, 0x00};
const byte readPictureSize[] = {0x56, 0x00, 0x34, 0x01, 0x00};
const byte readPicture[] = {0x56, 0x00, 0x32, 0x0C, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF};

bool initializeCamera() {
  // Initialize camera communication
  cameraSerial.begin(38400); // Default baud rate for Adafruit camera
  delay(1000);
  
  // Try to reset camera
  cameraSerial.write(cameraReset, sizeof(cameraReset));
  delay(500);
  
  // Check for response
  if (cameraSerial.available() > 0) {
    while(cameraSerial.available()) {
      cameraSerial.read(); // clear buffer
    }
    Serial.println("Camera detected and initialized!");
    tone(buzzerPin, 1000, 50); // Beep to indicate success
    delay(50);
    noTone(buzzerPin);
    delay(25);
    tone(buzzerPin, 1000, 50); // Beep again
    return true;

  } else {
    Serial.println("Camera not detected - camera features disabled");
    tone(buzzerPin, 1000, 50); // Beep to indicate failure
    delay(50);
    noTone(buzzerPin);
    delay(25);
    tone(buzzerPin, 500, 50); // Beep again
    delay(50);
    noTone(buzzerPin);
    return false;
  }
}

bool initializeSD() {
  // Initialize SD card
  if (!SD.begin(sdCS)) {
    Serial.println("SD card initialization failed - photo storage disabled");
    tone(buzzerPin, 1000, 50); // Beep to indicate failure
    delay(50);
    noTone(buzzerPin);
    delay(25);
    tone(buzzerPin, 500, 50); // Beep again
    delay(50);
    noTone(buzzerPin);
    return false;
  }
  Serial.println("SD card initialized successfully!");
  tone(buzzerPin, 1000, 50); // Beep to indicate success
  delay(50);
  noTone(buzzerPin);
  delay(25);
  tone(buzzerPin, 1000, 50); // Beep again
  
  // Create photos directory if it doesn't exist
  if (!SD.exists("/PHOTOS")) { // If /Photos directory not exist,
    SD.mkdir("/PHOTOS"); // Create it.
  }
  
  return true;
}

void takeSecurityPhoto() {
  if (!sdAvailable || photoTaken) {
    Serial.println("Camera or SD card not available, or photo already taken.");
    return; // Skip if camera/SD not available or photo already taken
  }
  if (!cameraAvailable) {
    Serial.println("Camera not available, skipping photo capture.");
    return; // Skip if camera not available
  }
  
  Serial.println("Taking security photo...");
  
  // Take picture command
  cameraSerial.write(takePicture, sizeof(takePicture));
  delay(1000); // Wait for camera to capture
  
  // Get picture size
  cameraSerial.write(readPictureSize, sizeof(readPictureSize));
  delay(100);
  
  if (cameraSerial.available() < 9) {
    Serial.println("Failed to get picture size");
    return;
  }
  
  // Read size response
  byte sizeResponse[9];
  for (int i = 0; i < 9; i++) {
    sizeResponse[i] = cameraSerial.read();
  }
  
  // Extract picture size (bytes 7-8 contain size)
  uint16_t pictureSize = (sizeResponse[7] << 8) | sizeResponse[8];
  Serial.print("Picture size: ");
  Serial.println(pictureSize);
  
  if (pictureSize == 0) {
    Serial.println("Invalid picture size");
    return;
  }
  
  // Create filename
  photoCount++;
  String filename = "/PHOTOS/SEC" + String(photoCount) + ".JPG";
  
  // Create file on SD card
  File photoFile = SD.open(filename, FILE_WRITE);
  if (!photoFile) {
    Serial.println("Failed to create photo file");
    return;
  }
  
  // Read picture data
  byte readCmd[16];
  memcpy(readCmd, readPicture, sizeof(readPicture));
  // Set start address to 0 and size
  readCmd[12] = (pictureSize >> 8) & 0xFF;
  readCmd[13] = pictureSize & 0xFF;
  
  cameraSerial.write(readCmd, sizeof(readCmd));
  delay(100);
  
  // Read and save image data
  int bytesRead = 0;
  unsigned long timeout = millis() + 5000; // 5 second timeout
  
  while (bytesRead < pictureSize && millis() < timeout) { // Okay to ignore warning about sign and unsigned comparison here
    if (cameraSerial.available()) {
      byte data = cameraSerial.read();
      photoFile.write(data);
      bytesRead++;
      
      if (bytesRead % 100 == 0) {
        Serial.print(".");
      }
    }
  }
  
  photoFile.close();
  
  if (bytesRead == pictureSize) { // Okay to ignore warning about sign and unsigned comparison here too.
    Serial.println();
    Serial.print("Photo saved: ");
    Serial.println(filename);
    photoTaken = true; // Mark photo as taken for this trip
  } else {
    Serial.println("Photo transfer incomplete");
    SD.remove(filename); // Remove incomplete file
  }
}

void bootupsequence() {
  static bool firstBoot = true;  // Only initialize on very first boot
  
  if (silentMode) {
    // Silent mode
    digitalWrite(armedPin, HIGH);
    digitalWrite(lightPin, LOW);
    delay(500);
    digitalWrite(silentIndicatorPin, HIGH);
    
    if (firstBoot) {
      firstBoot = false;
      cameraAvailable = initializeCamera();
      sdAvailable = initializeSD();
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
  
  // Normal with sound - first beep
  tone(buzzerPin, 1000);
  digitalWrite(armedPin, HIGH); 
  digitalWrite(lightPin, LOW);
  delay(50);
  noTone(buzzerPin);
  delay(500);
  digitalWrite(silentIndicatorPin, HIGH);
  
  if (firstBoot) {
    firstBoot = false;
    cameraAvailable = initializeCamera();
    sdAvailable = initializeSD();
  }
  
  delay(1250);
  digitalWrite(silentIndicatorPin, LOW);
  
  // Continue with remaining beeps
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
  Serial.begin(9600);
  Serial.println("Security System Initializing...");

  // sets up pins
  pinMode(silenttogglePin, INPUT_PULLUP); 
  pinMode(ledPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(armedPin, OUTPUT);
  pinMode(silentIndicatorPin, OUTPUT);
  
  // check initial silent mode state
  silentMode = (digitalRead(silenttogglePin) == LOW);
  
  // set initial silent mode indicator
  digitalWrite(silentIndicatorPin, silentMode ? HIGH : LOW);
  
  // startupp!Q!! (now includes initialization)
  bootupsequence();
  
  Serial.println("Security Ready");
  Serial.print("Silent Mode: ");
  Serial.println(silentMode ? "ON" : "OFF");
  Serial.print("Camera: ");
  Serial.println(cameraAvailable ? "ENABLED" : "DISABLED");
  Serial.print("SD Storage: ");
  Serial.println(sdAvailable ? "ENABLED" : "DISABLED");
}

void loop() {
  // read light sensor value
  int lightValue = analogRead(lightSensorPin);
  Serial.println(lightValue);
  unsigned long currentMillis = millis();

  // get tripped noob
  if (lightValue < 60 && !tripped) {
    tripped = true;
    photoTaken = false; // reset photo flag for new trip
    Serial.println("gocha");
    previousMillis = currentMillis; // reset blink timer
    
    // take security photo immediately when tripped
    takeSecurityPhoto(); // TAKE IT NOW!!
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
      photoTaken = false; // Reset photo flag
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

  delay(50); // small delay to prevent excessive CPU usage
}
// if wood chuck could chuck wood, how much wood would a wood chuck chuck if a wood chuck could chuck wood?
// a wood chuck would chuck as much wood as a wood chuck could chuck if a wood chuck could chuck wood.
// why would a wood chuck chuck wood if a wood chuck could chuck wood?
// because wood chuck could chuck wood if a wood chuck could chuck wood.
// if they could chuck wood, they would chuck wood.