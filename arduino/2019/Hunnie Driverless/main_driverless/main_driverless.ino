#include <hcsr04.h>
//#include <HCSR04.h>
#include <Servo.h>              // servo



#define Nsensors 8

bool debug = true;

// Use this to nuber and label your sensors
#define TRIG_PIN 2 // One trigger pin for all sensors
const int ECHO_PINS[Nsensors] = {4, 7, 8, 12, A4, A5, A6, A7};
String SensorNames[Nsensors] = {"X+", "X+Y+", "Y+", "X-Y+", "X-", "X-Y-", "Y-", "X+Y-"};

// Initialize counters etc.
int count = 0;
int idx = 0;
unsigned long t = 0;
unsigned long t_sensor = 0;
unsigned long t_main_loop = 0;
unsigned long dt = 0;

// Sensor range in mm
int minRange = 5;
int maxRange = 4000;

// Array of pointers to sensor objects. (Not exactly sure how it works but it works.)
HCSR04 *sensorArray[Nsensors];

// A few variables for distances
int distances[Nsensors];
int dist_cur;
int dist_filt[Nsensors];
int dist_xp;
int dist_xm;
int dist_yp;
int dist_ym;
int dist_max = 200; // Safe distance from wall

// Velocity variables
double v_x_des;
double v_xp_des;
double v_xm_des;
double v_x_cur = 0;
double v_max = 1;

double wp_des;
double wm_des;
double w_des;
double w_cur = 0;
double w_max = 1;

// Accelerations
double acc_max = 1;
double w_dot_max = 1;

// Tuning parameters
double diag_sens_factor = 0.125;
double orth_sens_factor = 1 - 2 * diag_sens_factor;

//unsigned long dt = 20;

// Sync sample time with filter_design M-file.
unsigned long dt_sensor = 160 / Nsensors;
//unsigned long dt_sensor = 1000 / Nsensors;

void setup() {
  Serial.begin(9600);

  // Initialize sensors
  for (int i = 0; i < Nsensors; i++) {
    pinMode(ECHO_PINS[i],INPUT);
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

void loop() {

  // Timing
//  delay(50);

  t = millis();
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

    if (debug) {
      // Print everything
      Serial.print(SensorNames[idx]);
      Serial.print(distances[idx]);
      Serial.print("\n");

      Serial.print("t:");
      Serial.print(t);
      Serial.print("\n");
    }

    digitalWrite(TRIG_PIN, LOW);
    count++;
  }

  // Time since last loop
  dt = t - t_main_loop;
  t_main_loop = t;

  // Straight ahead should be the first sensor.
  // Take some linear combination of the 3 forward facing sensors.
  dist_xp = orth_sens_factor * dist_filt[0] + diag_sens_factor * dist_filt[7] + diag_sens_factor * dist_filt[1];

  // Distance behind the car
  // Take the same linear combination of the 3 backward facing sensors.
  dist_xm = orth_sens_factor * dist_filt[4] + diag_sens_factor * dist_filt[3] + diag_sens_factor * dist_filt[5];

  // Distances left and right
  dist_yp = orth_sens_factor * dist_filt[2] + diag_sens_factor * dist_filt[1] + diag_sens_factor * dist_filt[3];
  dist_ym = orth_sens_factor * dist_filt[6] + diag_sens_factor * dist_filt[5] + diag_sens_factor * dist_filt[7];

  // Forward velocity depends on distance to wall (front and back).
  v_xp_des = min(v_max, v_max * ((double) dist_xp - (double) dist_max) / (double) dist_max);
  v_xm_des = max(0, -v_max * ((double) dist_xm - (double) dist_max) / (double) dist_max); // This has a zero minimum, otherwise the car would be attracted to walls behind it.
  v_x_des  = min(v_max, v_xp_des + v_xm_des);

  // Angular velocity depends on distance to left and right walls.
  wp_des = max(0, -w_max * ((double) dist_yp - (double) dist_max) / (double) dist_max); // This has a zero minimum, otherwise the car would be attracted to walls to its right.
  wm_des = min(0, w_max * ((double) dist_ym - (double) dist_max) / (double) dist_max);  // This has a zero maximum, otherwise the car would be attracted to walls to its left.
  w_des  = min(v_max, v_xp_des + v_xm_des);

  // Update actual velocity according to acceleration
  v_x_cur = v_x_cur + acc_max * (double) dt / 1000 * sign(v_x_des - v_x_cur);
  w_cur = w_cur + w_dot_max * (double) dt / 1000 * sign(w_des - w_cur);

  Serial.print("distances:  ");
  Serial.print(dist_filt[0]); Serial.print("  ");
  Serial.print(dist_filt[1]); Serial.print("  ");
  Serial.print(dist_filt[2]); Serial.print("  ");
  Serial.print(dist_filt[3]); Serial.print("  ");
  Serial.print(dist_filt[4]); Serial.print("  ");
  Serial.print(dist_filt[5]); Serial.print("  ");
  Serial.print(dist_filt[6]); Serial.print("  ");
  Serial.print(dist_filt[7]); Serial.print("  ");
  Serial.print("\n");
  Serial.print("dt: ");
  Serial.print(dt);
  Serial.print("\n");
  //  Serial.print("   v_x_cur:  ");
  //  Serial.print(v_x_cur);
  //  Serial.print("\n");


}
