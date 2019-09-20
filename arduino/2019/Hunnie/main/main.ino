#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Servo.h>              // servo

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

Servo myservo;  // create servo object to control a servo

/* init pins*/
// byte velocityReadPin = A2;
// int releasePin = 6;
byte sensorPin = A1;      //
int servoPin = 3;         // servo pin

double filterPosGoal = 1.0; // importantance curerent value

double aServo;
double velGoal = 0;
double vel;

double receiverValue;
double error;
double errorI = 0;
double errorD = 0;
double errorPrev = 0;
double errorTot;

double scaling;
double scalingPrev = 0;
double scalingIncrement = 0.025;
double scaleIntenstity = 1;

double velTreshold = 0.02; // below treshold vel is set to 0

/* steering */
const double RECEIVEWIDTHMIN = 972;   // [us] minimum width receiving steering signal
const double RECEIVEWIDTHAVG = 1437;  // [us] default width receiving steering signal
const double RECEIVEWIDTHMAX = 1970;  // [us] maximum width receiving steering signal

const double STEERANGLEMIN = 45; //[deg] minimum steering angle
const double STEERANGLEAVG = 90; //[deg] default steering angle
const double STEERANGLEMAX = 135;//[deg] maximum steering angle

/* CONTROLLER CONSTANTS*/
const double filterP = 1;     // importantance curerent value
const double filterI = 0.95;   // 0.75  // importantance curerent value
const double filterD = 1;     // // importantance curerent value
const double KP = 5; // 7.5; //5.00;
const double KI = 100; //50;
const double KD = 0.5; //0.5;

double posGoal = STEERANGLEAVG;   // variable to store the servo position. initialize in middle
double posGoalPrev = posGoal;     // variable to store previous pos goal

/* Timer variables */
unsigned long t = 0;
unsigned long tControlPrev = 0;
unsigned long tPrintPrev = 0;
unsigned long tBlinkPrev = 0;
unsigned long jitter;
// unsigned long tReadPrev;

/* init frequencies */
const long dtControl = 20;
const long dtPrint = 500;
const long dtBlink = 100;
//const long dtRead = 10;

boolean debug = true;
int ledMode = LOW;

void displaySensorDetails(void)
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  delay(500);
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 9DOF Tester")); Serial.println("");
  
  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

  Serial.println(F("Complete!")); Serial.println("");
  
  /* Initialise the servo */
  Serial.println(F("====Initializing sensors====")); Serial.println("");
  
  myservo.attach(servoPin);  // attaches the servo on pin 3 to the servo object
  aServo = posGoal;
  myservo.write(aServo);
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(sensorPin, INPUT);
}

void loop(void)
{

  t = millis(); // [ms]


  /* Control loop */
  if ((t - tControlPrev) >= dtControl){
    
    /* Activate led when too much jitter*/
    if ((t - tControlPrev) >= 1.5 * dtControl){
      ledMode = HIGH;
      if (debug) {
        Serial.print(F("==Didn't reach loop=="));
        Serial.println("");
      }
    }

    /* read receiver */
    receiverValue = pulseIn(sensorPin, HIGH); //[us] read pwm pin

    /* filter noise */
    if (abs(receiverValue - RECEIVEWIDTHAVG) < 50){
      receiverValue = RECEIVEWIDTHAVG;
    }

    /* convert reveiver value to desired steering angle with simple interpolation */
    posGoal = (receiverValue - RECEIVEWIDTHAVG) * (STEERANGLEMAX - STEERANGLEMIN)/(RECEIVEWIDTHMAX - RECEIVEWIDTHMIN) + STEERANGLEAVG;

    /* fitler desired steering angle */
    posGoal = filterPosGoal * posGoal + (1 - filterPosGoal) * posGoalPrev;
    posGoalPrev = posGoal;

    /* updatre timing */
    jitter = t - tControlPrev - dtControl;  //[ms]
    tControlPrev = t;                       //[ms]

    /* read gyro */
    sensors_event_t event;  // create sensor event
    gyro.getEvent(&event);  // get gyro sensor event
    vel = event.gyro.z;     // [rad/s] extract velocity data around z axis.

    /* filter velocity */
    if (abs(vel) <= velTreshold){
      vel = 0;      
    }

    /* compute scaling */
    scaling = -pow(abs(posGoal/90 - 1), scaleIntenstity) + 1;   // [-] get scaling factor for error value

    /* when we exit a corner we dont want the PID to directly kick in */
    if (scaling > scalingPrev){
      scaling = scalingPrev + scalingIncrement * (scaling - scalingPrev);
    }
    
    /* compute controler */
    error  = scaling * filterP * (velGoal - vel) + (1 - filterP) * error;                            //[rad/s]
    errorI = scaling * error * dtControl * 0.001 + filterI * errorI;                                //[rad]
    errorD = scaling * filterD * (error - errorPrev)/(dtControl * 0.001) + (1 - filterD) * errorD;  //[rad/s^2]
    errorTot = KP * error + KI * errorI + KD * errorD;                                    //[-]

    // errorTot = scaling * errorTot;                  // [-] scale total error
    errorPrev = error;
    scalingPrev = scaling; 
    
    /* determine new setpoint */
    aServo = posGoal + errorTot;
    if (aServo < STEERANGLEMIN){ // limit minimum angle
      aServo = STEERANGLEMIN;
    }
    else if (aServo > STEERANGLEMAX){ // limit maximum angle
      aServo = STEERANGLEMAX;
    }

    myservo.write(aServo);
  }

  /* prints for debugging */
  if (debug){
      if ((t - tPrintPrev) > dtPrint){
        tPrintPrev = t;

        Serial.print(F("I error: "));
        Serial.print(errorI);
        Serial.print("\t");              // prints a tab
        
        if (false){  
          Serial.print(F("vel goal: "));
          Serial.print(velGoal);
          Serial.print("\t");              // prints a tab
  
          Serial.print(F("vel error: "));
          Serial.print(velGoal);
          Serial.print("\t");              // prints a tab
          
          Serial.print(F("D error: "));
          Serial.print(errorD);
          Serial.print("\t");              // prints a tab
          
          Serial.print(F("tot error: "));
          Serial.print(errorTot);
          Serial.print("\t");              // prints a tab

          Serial.print(F("vel: "));
          Serial.print(vel);
          Serial.print("\t");              // prints a tab
          
          Serial.print(F("receiver: "));
          Serial.print(receiverValue);
          Serial.print("\t");              // prints a tab
          
          Serial.print(F("angle servo: "));
          Serial.print(aServo);
          Serial.print("\t");              // prints a tab
  
          Serial.print(F("jitter: "));
          Serial.print(jitter);   
          Serial.print(F(" [ms]"));
          Serial.println("");
        }
      }
    }

  /* Blink LED if we do not meet desider control frequency */
  if ((t - tBlinkPrev) > dtBlink){
    tBlinkPrev = t;
    digitalWrite(LED_BUILTIN, ledMode);
    ledMode = LOW;
  }
}
