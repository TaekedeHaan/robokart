#include <hcsr04.h>
//#include <HCSR04.h>
#include <Servo.h>              // servo

#define TRIG_PIN_LEFT 8
#define ECHO_PIN_LEFT 9

#define TRIG_PIN_RIGHT 8
#define ECHO_PIN_RIGHT 13

#define SERVO_PIN 3

// HCSR04 Library
//UltraSonicDistanceSensor hcsr04left(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
//UltraSonicDistanceSensor hcsr04right(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

// Bifrost library
HCSR04 hcsr04left(TRIG_PIN_LEFT, ECHO_PIN_LEFT, 20, 400);
HCSR04 hcsr04right(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, 20, 400);

//Servo myservo;  // create servo object to control a servo

double servoAngle1;
double servoAngle2;
double servoAngleNew;
double servoAngle; // Angle to write to the servo

double maxAngle = 90;

double distLeft;
double distRight;

int brightness;

int dt = 10;

void setup() {
  Serial.begin(9600);

  //  myservo.attach(SERVO_PIN);  // attaches the servo on pin 3 to the servo object
}

void loop() {
  //    Serial.println("LEFT: \n");
  //    Serial.println(hcsr04left.ToString());

  //    Serial.println("RIGHT: \n");
  //    Serial.println(hcsr04right.ToString());

  // HCSR04 Library
  //      distLeft = hcsr04left.measureDistanceCm();
  //      distRight = hcsr04right.measureDistanceCm();

  // Bifrost library
    delay(dt);
    distLeft = (double)hcsr04left.distanceInMillimeters();
    delay(dt);
    distRight = (double)hcsr04right.distanceInMillimeters();

  //  Serial.println("Left:");
  //  Serial.println(distLeft);

  //  Serial.println("Right:");
  //  Serial.println(distRight);

  //      servoAngle1 = 0.0;
  //      servoAngle2 = 0.0;

  if (distLeft > 0) {
    servoAngle1 = (1 / (1 + distLeft / 100)) * maxAngle / 2;
  }
  else {
    servoAngle1 = 0.0;
  }

  if (distRight > 0) {
    servoAngle2 = (1 / (1 + distRight / 100)) * maxAngle / 2;
  }
  else {
    servoAngle2 = 0.0;
  }

  //  Serial.println("Angle 1:");
  //  Serial.println(servoAngle1);

  //  Serial.println("Angle 2");
  //  Serial.println(servoAngle2);

  //      servoAngle = 0;

  servoAngleNew = servoAngle1 + servoAngle2;

  servoAngle = servoAngleNew;// 0.8 * servoAngle + 0.2 * servoAngleNew;

  //  Serial.println("Angle");
  //  Serial.println(servoAngle);

  //  Serial.println("\n");


  //      Serial.println("Angle: \n");
  //      Serial.println(servoAngle);

  //  myservo.write(servoAngle);

  brightness = servoAngle / maxAngle * 255;
  analogWrite(SERVO_PIN, brightness); // There is an LED on the servo pin.

  //  delay(250);
  //    delay(1000);
}
