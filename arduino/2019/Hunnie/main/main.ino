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

Servo steerServo;  // create servo object to control the steering servo
Servo releaseServo;  // create servo object to control the release servo

/* init pins*/
#define RELEASEPIN 6
#define STEERIN A1     //
#define STEEROUTPIN 3     // servo pin
#define VELOCITYIN A3  // pin which reads velocity

/* receiver velocity PWM width */
#define VELOCITY_IN_MID 1470
#define VELOCITYINMAX 2000
#define VELOCITYINMIN 1000
#define VELOCITY_IN_MARGIN 50
#define DTRELEASE 2000 // [ms]

/* receiver steering PWM width */
#define STEER_IN_MID 1437  // [us] default width receiving steering signal
#define STEER_IN_AMPLITUDE 500  // [us] maximum width receiving steering signal
#define STEER_IN_MARGIN 60

/* Steering servo angles */
// #define STEERANGLEMIN 45 //[deg] minimum steering angle
#define STEER_OUT_MID 107 //[deg] default steering angle
#define STEER_OUT_AMPLITUDE 47//[deg] maximum steering angle

#define STEER_OUT_MIN STEER_OUT_MID - STEER_OUT_AMPLITUDE
#define STEER_OUT_MAX STEER_OUT_MID + STEER_OUT_AMPLITUDE
// #define STEERIN2ANGLE STEERANGLEWIDTH/RECEIVEWIDTHAMPLITUDE

#define RELEASE_NEUTRAL_ANGLE 155
#define RELEASE_ACTIVATE_ANGLE 90

double filterPosGoal = 1.0; // importantance curerent value

double aServo;
double velGoal = 0;
double vel;

// velocity
double velocityIn;
double velocityInPrev;

double steerIn;
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

/* CONTROLLER CONSTANTS*/
const double filterP = 1;     // importantance curerent value
const double filterI = 0.95;   // 0.75  // importantance curerent value
const double filterD = 1;     // // importantance curerent value
const double KP = 5; // 7.5; //5.00;
const double KI = 100; //50;
const double KD = 0.5; //0.5;

double posGoal = STEER_OUT_MID;   // variable to store the servo position. initialize in middle
double posGoalPrev = posGoal;     // variable to store previous pos goal

/* */
int brakeCount = 0; // brake counter
int releasePhase = 0; // throttle counter
int throttleThreshold = 3; // threshold

/* Timer variables */
unsigned long t = 0;
unsigned long tControlPrev = 0;
unsigned long tPrintPrev = 0;
unsigned long tGadgetsPrev = 0;
unsigned long tBlinkPrev = 0;
unsigned long tReleasePhase = 0;
unsigned long tBrakeCounter = 0;
unsigned long jitter;


/* gadgets*/
boolean releaseWeapon = false;

/* init frequencies */
const long dtControl = 20; //20;
const long dtPrint = 500;
const long dtBlink = 100;
const long dtGadgets = 200;
const long dtBrakeCounter = 100;
//const long dtRead = 10;

boolean debug = false;
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
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  if (!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

  Serial.println(F("Complete!")); Serial.println("");

  /* Initialise the servo */
  Serial.println(F("====Initializing sensors====")); Serial.println("");

  /* Steer servo */
  steerServo.attach(STEEROUTPIN);
  aServo = posGoal;
  steerServo.write(aServo);

  /* Release servo */
  releaseServo.attach(RELEASEPIN);
  releaseServo.write(RELEASE_NEUTRAL_ANGLE);

  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STEERIN, INPUT);
  pinMode(VELOCITYIN, INPUT);
}

void loop(void)
{

  t = millis(); // [ms]

  /* Control loop */
  if ((t - tControlPrev) >= dtControl) {

    /* Activate led when too much jitter*/
    if ((t - tControlPrev) >= 1.5 * dtControl) {
      ledMode = HIGH;
      if (debug) {
        Serial.print(F("==Didn't reach loop=="));
        Serial.println("");
      }
    }

    /* updatre timing */
    jitter = t - tControlPrev - dtControl;  //[ms]
    tControlPrev = t;                       //[ms]

    /* read receiver */
    steerIn = pulseIn(STEERIN, HIGH); //[us] read pwm pin

    /* filter noise on steer */
    if (abs(steerIn - STEER_IN_MID) < STEER_IN_MARGIN) {
      steerIn = STEER_IN_MID;
    }

    /* shift troug release phases */
    if ((steerIn >= STEER_IN_MID + 0.9 * STEER_IN_AMPLITUDE) && (releasePhase == 0)) { // if we were previously not pressing gass
      releasePhase = 1;
      tReleasePhase = t;
    }

    if ((steerIn <= STEER_IN_MID - 0.9 * STEER_IN_AMPLITUDE) && (releasePhase == 1)) { // if we were previously not pressing gass
      releasePhase = 2;
    }

    if ((steerIn >= STEER_IN_MID + 0.9 * STEER_IN_AMPLITUDE) && (releasePhase == 2)) { // if we were previously not pressing gass
      releasePhase = 3;
    }

    if (t - tReleasePhase > DTRELEASE) {
      releasePhase = 0;
    }

    /* convert reveiver value to desired steering angle with simple interpolation */
    posGoal = (steerIn - STEER_IN_MID) * STEER_OUT_AMPLITUDE / STEER_IN_AMPLITUDE + STEER_OUT_MID;

    /* fitler desired steering angle */
    // posGoal = filterPosGoal * posGoal + (1 - filterPosGoal) * posGoalPrev;
    // posGoalPrev = posGoal;

    /* read gyro */
    sensors_event_t event;  // create sensor event
    gyro.getEvent(&event);  // get gyro sensor event
    vel = event.gyro.z;     // [rad/s] extract velocity data around z axis.

    /* filter velocity */
    if (abs(vel) <= velTreshold) {
      vel = 0;
    }

    /* compute scaling */
    scaling = -pow(abs( (posGoal - STEER_OUT_MID) / STEER_OUT_AMPLITUDE), scaleIntenstity) + 1; // [-] get scaling factor for error value

    if (brakeCount >= 2) {
      scaling = 0;
    }

    /* when we exit a corner we dont want the PID to directly kick in */
    if (scaling > scalingPrev) {
      scaling = scalingPrev + scalingIncrement * (scaling - scalingPrev);
    }

    /* compute controler */
    error  = scaling * filterP * (velGoal - vel) + (1 - filterP) * error;                            //[rad/s]
    errorI = scaling * error * dtControl * 0.001 + filterI * errorI;                                //[rad]
    errorD = scaling * filterD * (error - errorPrev) / (dtControl * 0.001) + (1 - filterD) * errorD; //[rad/s^2]
    errorTot = KP * error + KI * errorI + KD * errorD;                                    //[-]

    // errorTot = scaling * errorTot;                  // [-] scale total error
    errorPrev = error;
    scalingPrev = scaling;

    /* determine new setpoint */
    aServo = posGoal + errorTot;
    if (aServo < STEER_OUT_MIN) { // limit minimum angle
      aServo = STEER_OUT_MIN;
    }
    else if (aServo > STEER_OUT_MAX) { // limit maximum angle
      aServo = STEER_OUT_MAX;
    }

    steerServo.write(aServo);
  }

  /* */
  if ((t - tBrakeCounter) > dtBrakeCounter) {
    tBrakeCounter = t;
    velocityIn = pulseIn(VELOCITYIN, HIGH); //[us] read pwm pin

    /* filter noise on velocity */
    if (abs(velocityIn - VELOCITY_IN_MID) < VELOCITY_IN_MARGIN) {
      velocityIn = VELOCITY_IN_MID;
    }

    /* count braking */
    if (velocityInPrev >= VELOCITY_IN_MID) { // if we were previously pressing gass

      /* If we are braking and previously weren't */
      if (velocityIn < VELOCITY_IN_MID) {
        brakeCount = brakeCount + 1;
      }
    }

    velocityInPrev = velocityIn;

    /* if we are pressing the gass pedal reset the braking counter */
    if (velocityIn > VELOCITY_IN_MID) {
      brakeCount = 0;
    }
  }

  /* gadgets loop */
  if ((t - tGadgetsPrev) > dtGadgets) {
    tGadgetsPrev = t;

    if (releasePhase == 3) {
      releaseWeapon = true;
    }

    if (releaseWeapon) {
      releaseServo.write(RELEASE_ACTIVATE_ANGLE);
    }
  }

  /* prints for debugging */
  if (debug) {
    if ((t - tPrintPrev) > dtPrint) {
      tPrintPrev = t;

      Serial.print("Brake counter: ");
      Serial.print(brakeCount);
      Serial.print("\t");

      Serial.print("Throttle counter: ");
      Serial.print(releasePhase);
      Serial.print("\t");

      Serial.print("velocity PWM width: ");
      Serial.print(velocityIn);
      Serial.print("[us] \t");

      Serial.print("Steer PWM width: ");
      Serial.print(steerIn);
      Serial.print("\t");              // prints a tab


      Serial.print(F("jitter: "));
      Serial.print(jitter);
      Serial.print(F(" [ms]"));
      Serial.println("");


      if (false) {
        Serial.print(F("I error: "));
        Serial.print(errorI);
        Serial.print("\t");              // prints a tab

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
        Serial.print(steerIn);
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
  if ((t - tBlinkPrev) > dtBlink) {
    tBlinkPrev = t;
    digitalWrite(LED_BUILTIN, ledMode);
    ledMode = LOW;
  }
}
