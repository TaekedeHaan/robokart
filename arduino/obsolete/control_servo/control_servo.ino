#include <Wire.h>               // This library allows you to communicate with I2C / TWI devices. 
#include <Adafruit_Sensor.h>    // adafruit sensors
#include <Adafruit_LSM303_U.h>  // accel
#include <Adafruit_L3GD20_U.h>  // mag
#include <Adafruit_9DOF.h>      // ?
#include <Servo.h>              // servo

/* Assign a unique ID to the sensors */
// Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

Servo myservo;  // create servo object to control a servo

/* init pins*/
byte sensorPin = A0;      // PWM pin
int servoPin = 3;

double a = 0.5;

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

/* steering */
const double receiveDef = 1437;  // [us]
const double receiveMin = 972;   // [us]
const double receiveMax = 1970;  // [us]

const double steerAngleMin = 45; //[deg]
const double steerAngleMax = 135; //[deg]
const double steerAngleDef = 90; //[deg]

double posGoal = steerAngleDef;    // variable to store the servo position
double posGoalPrev = posGoal;

/* CONTROLLER CONSTANTS*/
double gamma = 0.99;
double alpha = 0.5;
double kP = 2; //0.01;
double kI = 0.0; //0.5;
double kD = 0.0000; //0.0001;

/* Timer variables */
unsigned long t = 0;
unsigned long tControlPrev = 0;
unsigned long tPrintPrev = 0;
unsigned long tBlinkPrev = 0;

/* init frequencies */
const long dtControl = 20;
const long dtPrint = 500;
const long dtBlink = 100;
//const long dtRead = 10;

unsigned long jitter;
unsigned long tReadPrev;

boolean debug = true;
int ledMode = LOW;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    setup
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  
  /* Initialise the sensors */
  Serial.println(F("====Initializing sensors====")); Serial.println("");
  
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

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

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  t = millis();

  if ((t - tControlPrev) >= dtControl){
    
    /* Activate led when too mhuch jitter*/
    if ((t - tControlPrev) >= 1.5 * dtControl){
      ledMode = HIGH;
      if (debug) {
        Serial.print(F("==Didn't reach loop=="));
        Serial.println("");
      }
    }

    /* read receiver */
    receiverValue = pulseIn(sensorPin, HIGH); //[us] read pwm pin
    
    if (abs(receiverValue - receiveDef) < 50){
      receiverValue = receiveDef;
    }
    
    posGoal = (receiverValue - receiveDef) * (steerAngleMax - steerAngleMin)/(receiveMax - receiveMin) + steerAngleDef;

    posGoal = a * posGoal + (1-a) * posGoalPrev;
    posGoalPrev = posGoal;
    
    // sensorValue = analogRead(sensorPin);  // [-] read analog pin
    // posGoal = (sensorValue/1024) * 180;   // [deg] convert to degrees 

    /* updatre timing */
    jitter = t - tControlPrev - dtControl;  //[ms]
    tControlPrev = t;                       //[ms]

    /* read gyro */
    sensors_event_t event;  // create sensor event
    gyro.getEvent(&event);  // get gyro sensor event
    vel = event.gyro.z;     // extract velocity data around z axis.

    // filter velocity
    if (abs(vel) < 0.01){
      vel = 0;      
    }

    /* compute controler */
    error = velGoal - vel;                                                            //[deg/s]
    errorI = error * dtControl * 0.001 + gamma * errorI;                              //[deg]
    errorD = (1 - alpha) * (error - errorPrev)/(dtControl * 0.001) + alpha * errorD;  //[deg/s^2]
    errorTot = kP * error + kI * errorI + kD * errorD;                                //[-]

    /* apply scaling */
    scaling = -pow(abs(posGoal/90 - 1), 0.5) + 1;   // [-] get scaling factor for error value
    errorTot = scaling * errorTot;        // [-] scale total error
    
    /* set error to 0 when outside certain input range
    if (posGoal > 100 || posGoal < 80){
      errorTot = 0;
    } 
    */
    
    /* determine new setpoint */
    aServo = posGoal + errorTot;
    if (aServo < steerAngleMin){ // limit minimum angle
      aServo = steerAngleMin;
    }
    else if (aServo > steerAngleMax){ // limit maximum angle
      aServo = steerAngleMax;
    }

    errorPrev = error;  
    myservo.write(aServo);
  }

  if (debug){
      if ((t - tPrintPrev) > dtPrint){
        tPrintPrev = t;
        if (false){  
          Serial.print(F("vel goal: "));
          Serial.print(velGoal);
          Serial.print("\t");              // prints a tab
  
          Serial.print(F("vel error: "));
          Serial.print(velGoal);
          Serial.print("\t");              // prints a tab
          
          Serial.print(F("I error: "));
          Serial.print(errorI);
          Serial.print("\t");              // prints a tab
          
          Serial.print(F("D error: "));
          Serial.print(errorD);
          Serial.print("\t");              // prints a tab
          
          Serial.print(F("tot error: "));
          Serial.print(errorTot);
          Serial.print("\t");              // prints a tab
        }

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

  if ((t - tBlinkPrev) > dtBlink){
    tBlinkPrev = t;
    digitalWrite(LED_BUILTIN, ledMode);
    ledMode = LOW;
  }

  /*
  if ((t - tReadPrev) > dtRead){ 
  }
  */
}
