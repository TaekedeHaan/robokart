#include <Wire.h>               // This library allows you to communicate with I2C / TWI devices. 
#include <Adafruit_Sensor.h>    // adafruit sensors
#include <Adafruit_LSM303_U.h>  // accel
#include <Adafruit_L3GD20_U.h>  // mag
#include <Adafruit_9DOF.h>      // ?
#include <Servo.h>              // servo

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);

Servo myservo;  // create servo object to control a servo

/* init pins*/
int sensorPin = A0;    // select the input pin for the potentiometer
int servoPin = 3;

double posGoal = 90;    // variable to store the servo position
double aServo;
double velGoal = 0;
double vel;

double sensorValue;
double error;
double errorI = 0;
double errorD = 0;
double errorPrev = 0;
double errorTot;
double scaling;

/* CONTROLLER CONSTANTS*/
double gamma = 0.99;
double alpha = 0.5;
double kP = 0.01; //0.001;
double kI = 0.5; //0.05;
double kD = 0.0001; //0.00005;

/* Timer variables */
unsigned long t = 0;
unsigned long tControlPrev = 0;
unsigned long tPrintPrev = 0;
unsigned long tBlinkPrev = 0;

/* init frequencies */
const long dtControl = 5;
const long dtPrint = 200;
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
void initSensors()
{
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
}

/**************************************************************************/
/*!
    setup
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Control servo")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();

  /* Initialise the servo */
  myservo.attach(servoPin);  // attaches the servo on pin 3 to the servo object
  aServo = posGoal;
  myservo.write(aServo);
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);
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

    sensorValue = analogRead(sensorPin);
    posGoal = (sensorValue/1024) * 180;

    /* updatre timing */
    jitter = t - tControlPrev - dtControl;  //[ms]
    tControlPrev = t;                //[ms]

    /* read gyro */
    sensors_event_t event; //
    gyro.getEvent(&event);
    vel = event.gyro.z;

    /* compute controler */
    error = velGoal - vel; //[deg/s]
    errorI = error * dtControl * 0.001 + gamma * errorI; //[deg]
    errorD = (1 - alpha) * (error - errorPrev)/(dtControl * 0.001) + alpha * errorD; //[deg/s^2]
    errorTot = kP * error + kI * errorI + kD * errorD;

    scaling = -abs(posGoal/90 - 1) + 1;
    errorTot = scaling * errorTot;
    /*
    if (posGoal > 100 || posGoal < 80){
      errorTot = 0;
    } 
    */
    
    /* determine new setpoint */
    aServo = posGoal - errorTot;
    if (aServo < 0){
      aServo = 0;
    }
    else if (aServo > 180){
      aServo = 180;
    }

    errorPrev = error;  
    myservo.write(aServo);
  }

  if (debug){
      if ((t - tPrintPrev) > dtPrint){
        tPrintPrev = t;
        Serial.print(F("vel: "));
        Serial.print(vel);
        Serial.print("\t");              // prints a tab

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

        Serial.print(F("set point angle: "));
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

  //delay(10);
  /*
  if ((t - tReadPrev) > dtRead){ 
  }
  */
}
