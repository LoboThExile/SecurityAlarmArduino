#include <IRremote.h> // IR remote library
#include <Arduino.h> // Required for PlatformIO
#define IR_SEND_PIN 3 // Send ir data to pin 3.
void setup() {
  IrSender.begin(IR_SEND_PIN); //Start IR line
  IrSender.sendNECMSB(0x10AF8877, 32);
}
void loop() {
  delay(100);
  IrSender.sendNECMSB(0x10AF8877, 32);
}
