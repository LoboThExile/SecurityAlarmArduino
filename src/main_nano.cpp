#include <IRremote.h> // heres the library
#include <Arduino.h> // heres arduino

#define IR_SEND_PIN 3 // here we define what pin to send it
#define BUTTON1_PIN A5 // button 1 -> ground cause its input pullup

void setup() {
  Serial.begin(9600);
  Serial.println("Nano IR Sender Booted");
  pinMode(BUTTON1_PIN, INPUT_PULLUP); //here

  IrSender.begin(IR_SEND_PIN); //start serial for IR
}

void loop() {
  if (digitalRead(BUTTON1_PIN) == LOW) {
    Serial.println("Button pressed, sending IR...");
    IrSender.sendNECMSB(0x10AF8877, 32); // Example code, use MSB function
    Serial.println("IR sent!");
    delay(100);
  }
}