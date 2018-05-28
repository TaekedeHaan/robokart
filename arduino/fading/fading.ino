/*
 Fading

 This example shows how to fade an LED using the analogWrite() function.

 The circuit:
 * LED attached from digital pin 9 to ground.

 Created 1 Nov 2008
 By David A. Mellis
 modified 30 Aug 2011
 By Tom Igoe

 http://www.arduino.cc/en/Tutorial/Fading

 This example code is in the public domain.

 */


int ledPin3 = 3;    // LED connected to digital pin 9
int ledPin5 = 5;    // LED connected to digital pin 9
int ledPin6 = 6;    // LED connected to digital pin 9
int ledPin9 = 9;    // LED connected to digital pin 9
int ledPin10 = 10;    // LED connected to digital pin 9
int ledPin11 = 11;    // LED connected to digital pin 9

void setup() {
  // nothing happens in setup
}

void loop() {
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    // digitalWrite(ledPin, HIGH);
    analogWrite(ledPin3, fadeValue);
    analogWrite(ledPin5, fadeValue);
    analogWrite(ledPin6, fadeValue);
    analogWrite(ledPin9, fadeValue);
    analogWrite(ledPin10, fadeValue);
    analogWrite(ledPin11, fadeValue);
    
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    // digitalWrite(ledPin, LOW);
    analogWrite(ledPin3, fadeValue);
    analogWrite(ledPin5, fadeValue);
    analogWrite(ledPin6, fadeValue);
    analogWrite(ledPin9, fadeValue);
    analogWrite(ledPin10, fadeValue);
    analogWrite(ledPin11, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
}


