#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Servo.h>
#include <PID_v1.h>

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);

Servo myservo;  // create servo object to control a servo

double aServoInit= 90;    // variable to store the servo position
double aServo;
double vGoal = 0;
double vel;
double pos;
double velPrev = 0;
double acc;

double gamma = 0.95;
double alpha = 0.9;
double kP = 0.001;
double kI = 0.05;
double kD = 0.00005;

unsigned long t;
unsigned long tPrev;
const long dt = 2;

int sampleTime = 0;
// PID orrPID(&v, &aServo, &vGoal,kP,kI,kD, REVERSE);

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
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();

  /* Initialise the servo */
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object
  aServo = aServoInit;
  myservo.write(aServo);
  delay(1000);

  //turn the PID on
  // orrPID.SetMode(AUTOMATIC);
  // orrPID.SetSampleTime(sampleTime);
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  t = millis();

  if ((t - tPrev) >= dt){
    tPrev = t;
    
    sensors_event_t event;
    gyro.getEvent(&event);
    vel = event.gyro.z;
    pos = vel * dt * 0.001 + gamma * pos;
    acc = (1 - alpha) * (vel - velPrev)/(dt * 0.001) + alpha * acc;
    // orrPID.Compute();
    aServo = aServo + kP * vel + kI * pos + kD * acc;
    
    if (aServo < 0){
      aServo = 0;
    }
    else if (aServo > 180){
      aServo = 180;
    }

    velPrev = vel;
    Serial.print(F("vel: "));
    Serial.print(vel);
    Serial.print(F("pos: "));
    Serial.print(pos);
    Serial.print(F("acc: "));
    Serial.print(acc);
    Serial.println("");
    
  }
  
  myservo.write(aServo);
  
    /* 
  if (!orrPID.Compute()){
    Serial.println(F("Failed to PID!"));
  }
  Serial.print(F("v: "));
  Serial.print(v);
  Serial.println(F(""));

  'orientation' should have valid .heading data now 
  Serial.print(F("aSensor: "));
  Serial.print(aSensor);
  Serial.print(F("; "));
  
  Serial.print(F("aError: "));
  Serial.print(aError);
  Serial.print(F("; "));

  Serial.print(F("angle servo: "));
  Serial.print(aServo);
  Serial.print(F("; "));
  Serial.println(F(""));
  */
  
  
  /* delay(15); */
}
