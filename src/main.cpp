#include <Arduino.h>

// constants won't change. Used here to set a pin number:
const int ledPin = LED_BUILTIN;  // the number of the LED pin
const int potPin = A0;  // Analog input A0
// Variables will change:
int ledState = LOW;  // ledState used to set the LED
const int pwmPin = 9;  // PWM pin 9 på Uno
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change:
const long interval = 1000;  // interval at which to blink (milliseconds)




void setup() {
  // Sæt PWM-pinen til output
  pinMode(pwmPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  // Du kan starte Serial Monitor for at se værdierne (valgfrit)
  Serial.begin(115200);
}

void loop() {
  unsigned long currentMillis = millis();
  // Læs værdien fra potentiometeret (0 - 1023)
  int sensorValue = analogRead(potPin);
  
  // Konverter den analoge læsning (0-1023) til en PWM værdi (0-255)
  // Ved at mappe (map) værdien
  int pwmValue = map(sensorValue, 0, 1023, 0, 255);
  
  // Skriv PWM-værdien til pinen
  analogWrite(pwmPin, pwmValue);
  
  // (Valgfrit) Print værdierne til Serial Monitor
  Serial.print("Potentiometer: ");
  Serial.print(sensorValue);
  Serial.print(" | PWM Output: ");
  Serial.println(pwmValue);

    if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  
  // En lille forsinkelse for at gøre Serial output læsbart
  // Dette kan gøres meget kortere eller fjernes helt efter behov
  delay(300); 
}