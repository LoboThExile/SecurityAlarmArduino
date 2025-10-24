#include <IRremote.h> // IR remote library
#include <Arduino.h> // Required for PlatformIO

#define IR_SEND_PIN 3 // Send ir data to pin 3.
#define BUTTON1_PIN A5 // Send to button that send to ground.

void setup() {
  Serial.begin(9600);
  //Serial.println("Nano IR Sender Booted"); //For debugging.
  pinMode(BUTTON1_PIN, INPUT_PULLUP); //Set pin to Pull up.

  IrSender.begin(IR_SEND_PIN); //Start IR line
}

void loop() {
  if (digitalRead(BUTTON1_PIN) == LOW) {
    //Serial.println("Button pressed, sending IR..."); // Debugging
    IrSender.sendNECMSB(0x10AF8877, 32); // Sends IR
    //Serial.println("IR sent!"); // Debugging
    delay(100);
  }
}
