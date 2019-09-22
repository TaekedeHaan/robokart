#include <hcsr04.h>
#include <Servo.h>              // servo

#define Nsensors 8

bool debug = true;
bool visualize = false;

// Pin definitions
#define DIR_LEFT 3 // A-IB
#define VEL_LEFT 5 // A-IA

#define DIR_RIGHT 6 // B-IB
#define VEL_RIGHT 9 // B-IA

#define TRIG_PIN 2 // One trigger pin for all sensors

// A0 A1 receiver
const int ECHO_PINS[Nsensors] = {4,   7,    8,    12,   A2,   A3,   A4,   A5};
String SensorNames[Nsensors] = {"F_", "FR", "_R", "BR", "B_", "BL", "_L", "FL"};

enum possible_states {FREE, FRONT, FRONT_LEFT, FRONT_RIGHT}; // enum class for possible states
possible_states state; // Variable for current state
const unsigned int dt_state[4] = {0, 500, 500, 500}; // Minimal time to spend in state in ms.

// HD params
double radius = 1;

// Driving valriables/constants
double w_des = 0;
double v_des = 0;

double v_max = 1;
double w_max = 1;

double v_nominal = 1;
double w_nominal = 1;

double wheelSpeeds[2] = {0, 0};
const double drift_penalty = 0.9;

// Initialize counters etc.
int count = 0;
int idx = 0;

// Timers
unsigned long t = 0;

unsigned long t_sensor = 0;
unsigned long t_main_loop = 0;
unsigned long t_print = 0;
unsigned long t_drive = 0;
unsigned long t_state = 0;

unsigned long dt = 0; // ??
//unsigned long dt = 20;
unsigned int dt_print = 1000;
unsigned int dt_drive = 100;
unsigned long dt_sensor = 160 / Nsensors;

// Sensor range in mm
int minRange = 5;
int maxRange = 4000;

// Array of pointers to sensor objects. (Not exactly sure how it works but it works.)
HCSR04 *sensorArray[Nsensors];

// A few variables for distances
int distances[Nsensors];
int dist_cur;
int dist_filt[Nsensors];

int rightIndex[3] = {1, 2, 3};
int leftIndex[3] = {5, 6, 7};
int leftSum = 0;
int rightSum = 0;

int dist_xp;
int dist_xm;
int dist_yp;
int dist_ym;
int dist_max = 300; // Safe distance from wall

// Velocity variables
double v_x_des;
double v_xp_des;
double v_xm_des;
double v_x_cur = 0;
// double v_max = 1;

double wp_des;
double wm_des;
// double w_des;
double w_cur = 0;
// double w_max = 1;

// Accelerations
double acc_max = 1;
double w_dot_max = 1;

const int PWM_MOTOR_MIN = 155;
const int PWM_MOTOR_MAX = 255;

// Tuning parameters
double diag_sens_factor = 0.125;
double orth_sens_factor = 1 - 2 * diag_sens_factor;

void setup() {
  Serial.begin(9600);

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(VEL_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(VEL_RIGHT, OUTPUT);

  // Initialize sensors
  for (int i = 0; i < Nsensors; i++) {
    pinMode(ECHO_PINS[i], INPUT);
    sensorArray[i] = new HCSR04(TRIG_PIN, ECHO_PINS[i], minRange, maxRange);
  }
}

// Helper functions
double readSensor(int sensorNumber) {
  // Declarevariables
  double dist;
  HCSR04* sensor = sensorArray[sensorNumber];

  // Use pointer to sensor object to call distance function
  dist = sensor->distanceInMillimeters();
  return dist;
}

double sign(double val) {
  double sgn = 1;
  if (val < 0) {
    sgn = -1;
  }
  if (val == 0) {
    sgn = 0;
  }
  return sgn;
}

void VW2wheelSpeeds(double v, double w, double wheelSpeeds[2]) {
  double diff;
  double vL;
  double vR;

  w = min(abs(w) * radius, v_max) / radius * sign(w);

  // Left
  vL = v - w * radius;

  // Right
  vR = v + w * radius;

  // In case of actuator windup
  diff = (max(abs(vL), v_max) - v_max) * sign(vL);
  vL = vL - diff;
  vR = vR - diff;

  diff = (max(abs(vR), v_max) - v_max) * sign(vR);
  vL = vL - diff;
  vR = vR - diff;

  // store results in vector
  wheelSpeeds[0] = vL; //(int) (vL / v_max * 255);
  wheelSpeeds[1] = drift_penalty * vR; //(int) (vR / v_max * 255);
}

void writeWheelSpeeds(double wheelSpeeds[2]) {

  double vL = wheelSpeeds[0];
  double vR = -wheelSpeeds[1];

  int LW;
  int RW;

  vL = min(vL, v_max); // Maximum velocity
  vL = max(vL, -v_max); // Maximum negative velocity
  vL = vL / v_max; // Normalize
  if (abs(vL) < 0.1) {
    vL = 0; // Minimum velocity (due to hysteresis in motors etc.)
  }

  vR = min(vR, v_max); // Maximum velocity
  vR = max(vR, -v_max); // Maximum velocity
  vR = vR / v_max; // Normalize
  if (abs(vR) < 0.1) {
    vR = 0; // Minimum velocity (due to hysteresis in motors etc.)
  }

  if (abs(vL) > 0) {
    LW = (int) (PWM_MOTOR_MIN + (PWM_MOTOR_MAX - PWM_MOTOR_MIN) * abs(vL)) * sign(vL);
  }
  else {
    LW = 0;
  }

  if (abs(vR) > 0) {
    RW = (int) (PWM_MOTOR_MIN + (PWM_MOTOR_MAX - PWM_MOTOR_MIN) * abs(vR)) * sign(vR);
  }
  else {
    RW = 0;
  }

  if (LW > 0) {
    digitalWrite(DIR_LEFT, LOW);
    analogWrite(VEL_LEFT, LW);
  }
  else if (LW < 0) {
    digitalWrite(DIR_LEFT, HIGH);
    analogWrite(VEL_LEFT, 255 + LW);
  }
  else {
    digitalWrite(DIR_LEFT, LOW);
    analogWrite(VEL_LEFT, 0);
  }

  if (RW > 0) {
    digitalWrite(DIR_RIGHT, LOW);
    analogWrite(VEL_RIGHT, RW);
  }
  else if (RW < 0) {
    digitalWrite(DIR_RIGHT, HIGH);
    analogWrite(VEL_RIGHT, 255 + RW);
  }
  else {
    digitalWrite(DIR_RIGHT, LOW);
    analogWrite(VEL_RIGHT, 0);

  }
}

// TODO: Fix iets met sizeof
int sumArray(int theArray[], int indexVec[]) {
  int sumOfVec = 0;
  for (int index = 0; index < sizeof(indexVec); index++) {
    sumOfVec += theArray[indexVec[index]];
  }
  return sumOfVec;
}


void loop() {
  t = millis();

  // read sensor
  if (t - t_sensor > dt_sensor) {

    // Save current time
    t_sensor = t;

    // Index of current sensor, every loop pick a different one.
    idx = count % Nsensors;

    // Make measurement
    dist_cur = readSensor(idx);

    if (dist_cur > 0) {
      // Only update distance if a measurement was made succesfully.
      distances[idx] = dist_cur;
    }

    // Apply filter
    dist_filt[idx] = 0.4 * dist_filt[idx] + 0.6 * distances[idx];

    digitalWrite(TRIG_PIN, LOW);
    count++;

    if (visualize) {
      Serial.print(SensorNames[idx]);
      Serial.print(dist_filt[idx]);
      Serial.print("\n");
    }
  }

  // Time since last loop
  //  dt = t - t_main_loop;
  //  t_main_loop = t;

  if (t - t_state > dt_state[state]) {
    if (dist_filt[1] < dist_max) {
      state = FRONT_RIGHT;
      t_state = t;
    }
    else if (dist_filt[7] < dist_max) {
      state = FRONT_LEFT;
      t_state = t;
    }
    else if (dist_filt[0] < dist_max) {
      state = FRONT;
      t_state = t;
    }
    else {
      state = FREE;
      t_state = t;
    }
  }

  if (t - t_drive > dt_drive) {
//    leftSum = sumArray(dist_filt, leftIndex);
//    rightSum = sumArray(dist_filt, rightIndex);

    switch (state) {
      case FREE:
        v_des = v_nominal;
        w_des = 0;
        break;

      case FRONT:
        v_des = -v_nominal;
        w_des = w_nominal;
        break;

      case FRONT_RIGHT:
        v_des = -v_nominal;
        w_des = w_nominal;
        break;

      case FRONT_LEFT:
        v_des = -v_nominal;
        w_des = -w_nominal;
        break;

      default:
        v_des = 1;
        w_des = 0;
        break;
    }


    VW2wheelSpeeds(v_des, w_des, wheelSpeeds);
    writeWheelSpeeds(wheelSpeeds);
  }

  if (debug) {
    if (t - t_print > dt_print) {
      t_print = t;

      for (int i = 0; i < Nsensors; i++) {
        Serial.print(SensorNames[i]); Serial.print("\t");
      }
      Serial.print("\n");

      for (int i = 0; i < Nsensors; i++) {
        Serial.print(dist_filt[i]); Serial.print("\t");
      }
      Serial.print("\n");
      Serial.print("Desired v: "); Serial.print(v_des); Serial.print("\t"); Serial.print("Desired w: "); Serial.print(w_des);
      Serial.print("\n");
      Serial.print("State: "); Serial.print(state); Serial.print("\t");
      Serial.print("\n");
      Serial.print("\n");
    }
  }

}
