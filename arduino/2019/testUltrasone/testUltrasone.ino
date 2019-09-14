#include <hcsr04.h>
//#include <HCSR04.h>
#include <Servo.h>              // servo

#define TRIG_PIN_X_POS 8
#define ECHO_PIN_X_POS 9

#define TRIG_PIN_Y_POS 8
#define ECHO_PIN_Y_POS 10

#define TRIG_PIN_X_NEG 8
#define ECHO_PIN_X_NEG 11

//#define SERVO_PIN 3

// HCSR04 Library
//UltraSonicDistanceSensor hcsr04left(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
//UltraSonicDistanceSensor hcsr04right(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

// Bifrost library
HCSR04 Sensor_X_POS(TRIG_PIN_X_POS, ECHO_PIN_X_POS, 20, 400);
HCSR04 Sensor_Y_POS(TRIG_PIN_Y_POS, ECHO_PIN_Y_POS, 20, 400);
HCSR04 Sensor_X_NEG(TRIG_PIN_X_NEG, ECHO_PIN_X_NEG, 20, 400);

HCSR04 sensorArray[3] = {Sensor_X_POS, Sensor_Y_POS, Sensor_X_NEG};

//Servo myservo;  // create servo object to control a servo
//
//double servoAngle1;
//double servoAngle2;
//double servoAngleNew;
//double servoAngle; // Angle to write to the servo
//
//double maxAngle = 90;
//
double dist_X_POS;
double dist_Y_POS;
double dist_X_NEG;

double dist_X_POS_old = 0;
double dist_Y_POS_old = 0;
double dist_X_NEG_old = 0;
//
//
//int brightness;

int dt = 10;

void setup() {
  Serial.begin(9600);

  //  myservo.attach(SERVO_PIN);  // attaches the servo on pin 3 to the servo object
}

double readSensor(int sensorNumber) {
  double dist;
switch(sensorNumber){
  case 1:
    dist = (double)Sensor_X_POS.distanceInMillimeters();
    break;
  case 2:
    dist = (double)Sensor_Y_POS.distanceInMillimeters();
    break;
  case 3:
    dist = (double)Sensor_X_NEG.distanceInMillimeters();
    break;
  default:
    dist = 0;
    break;
  }
return dist;
}

void loop() {

  // HCSR04 Library
  //      distLeft = hcsr04left.measureDistanceCm();
  //      distRight = hcsr04right.measureDistanceCm();

  // Bifrost library
    delay(dt);
    dist_X_POS = readSensor(1);
    dist_X_POS = 0.5*dist_X_POS + 0.5*dist_X_POS_old;
    
    Serial.print("X+");
    Serial.print(dist_X_POS);
    Serial.print("\n");
        
    delay(dt);
    dist_Y_POS = readSensor(2);
    dist_Y_POS = 0.5*dist_Y_POS + 0.5*dist_Y_POS_old;
    
    Serial.print("Y+");
    Serial.print(dist_Y_POS);
    Serial.print("\n");
    
    delay(dt);
    dist_X_NEG = readSensor(3);
    dist_X_NEG = 0.5*dist_X_NEG + 0.5*dist_X_NEG_old;
    
    Serial.print("X-");
    Serial.print(dist_X_NEG);
    Serial.print("\n");

    dist_X_POS_old = dist_X_POS;
    dist_Y_POS_old = dist_Y_POS;
    dist_X_NEG_old = dist_X_NEG;
    
//
//  if (distLeft > 0) {
//    servoAngle1 = (1 / (1 + distLeft / 100)) * maxAngle / 2;
//  }
//  else {
//    servoAngle1 = 0.0;
//  }
//
//  if (distRight > 0) {
//    servoAngle2 = (1 / (1 + distRight / 100)) * maxAngle / 2;
//  }
//  else {
//    servoAngle2 = 0.0;
//  }

//  servoAngleNew = servoAngle1 + servoAngle2;
//
//  servoAngle = servoAngleNew;// 0.8 * servoAngle + 0.2 * servoAngleNew;

  //  myservo.write(servoAngle);
//
//  brightness = servoAngle / maxAngle * 255;
//  analogWrite(SERVO_PIN, brightness); // There is an LED on the servo pin.

  //  delay(250);
}
