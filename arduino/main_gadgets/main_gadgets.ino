// color swirl! connect an RGB LED to the PWM pins as indicated
// in the #defines
// public domain, enjoy!
 
#define REDPIN 5
#define GREENPIN 6
#define BLUEPIN 3
#define ESCPIN A0      // pin from receiver

#define FADESPEED 5     // make this higher to slow down
#define DTPRINT 200     // make this higher to slow down

#define AVG 1500
#define MAX 2000
#define MIN 1000

long t = 0;
long tPrintPrev = 0;

int r;
int g;
int b;

double escValue;
 
void setup() {
  Serial.begin(115200);
  
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  pinMode(ESCPIN, INPUT);

  Serial.println(F("Complete setup!")); Serial.println("");
}
 
 
void loop() {

  /* read receiver, pulse length from 1ms (zero throttle) to 2ms (full throttle) */
  escValue = pulseIn(ESCPIN, HIGH); //[us] read pwm pin

 /*
  // fade from blue to violet
  for (r = 0; r < 256; r++) { 
    analogWrite(REDPIN, r);
    delay(FADESPEED);
  } 
  // fade from violet to red
  for (b = 255; b > 0; b--) { 
    analogWrite(BLUEPIN, b);
    delay(FADESPEED);
  } 
  // fade from red to yellow
  for (g = 0; g < 256; g++) { 
    analogWrite(GREENPIN, g);
    delay(FADESPEED);
  } 
  // fade from yellow to green
  for (r = 255; r > 0; r--) { 
    analogWrite(REDPIN, r);
    delay(FADESPEED);
  } 
  // fade from green to teal
  for (b = 0; b < 256; b++) { 
    analogWrite(BLUEPIN, b);
    delay(FADESPEED);
  } 
  // fade from teal to blue
  for (g = 255; g > 0; g--) { 
    analogWrite(GREENPIN, g);
    delay(FADESPEED);
  } 
  */

  t = millis();
  
  // print 
  if (t - tPrintPrev > DTPRINT){
    Serial.print(F("esc value: "));
    Serial.print(escValue);   
    Serial.print(F(" [ns]"));
    Serial.println("");
  }
}
