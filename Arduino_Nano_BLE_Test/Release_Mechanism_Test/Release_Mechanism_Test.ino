/*
 * This Arduino Nano code was developed by newbiely.com
 *
 * This Arduino Nano code is made available for public use without any restriction
 *
 * For comprehensive instructions and wiring diagrams, please visit:
 * https://newbiely.com/tutorials/arduino-nano/arduino-nano-relay
 */

#include <Relay.h>
#define RELAY_PIN  2  // The Arduino Nano pin connected to the IN pin of relay module

// The setup function runs once on reset or power-up
void setup() {
  // initialize digital pin as an output.
  pinMode(RELAY_PIN, OUTPUT);
}

Relay relay1;

// The loop function repeats indefinitely
void loop() {
  
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("Relay is ON");
  Serial.println(relay1.getState());
  delay(5000);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("Relay is OFF");
  Serial.println(relay1.getState());
  delay(2000);
}
