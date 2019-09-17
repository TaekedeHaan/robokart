#include <hcsr04.h>
//#include <HCSR04.h>
#include <Servo.h>              // servo



#define Nsensors 3

bool debug = false;

// Use this to nuber and label your sensors
#define TRIG_PIN 8 // One trigger pin for all sensors
const int ECHO_PINS[Nsensors] = {9, 10, 11};
String SensorNames[Nsensors] = {"X+", "Y+", "X-"};

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
int dist_max = 200; // Safe distance from wall

// Velocity variables
double v_x_des;
double v_x_cur = 0;
double v_max = 1;
double acc_max = 1;

//unsigned long dt = 20;

// Sync sample time with filter_design M-file.
unsigned long dt_sensor = 160 / Nsensors;

void setup() {
  Serial.begin(9600);

  // Initialize sensors
  for (int i = 0; i < Nsensors; i++) {
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
  dist_xp = dist_filt[0];

  // Forward velocity depends only on distance to wall.
  v_x_des = min(v_max, v_max * ((double) dist_xp - (double) dist_max) / (double) dist_max);

  // Update actual velocity according to acceleration
  v_x_cur = v_x_cur + acc_max * (double) dt/1000 * sign(v_x_des - v_x_cur);

  Serial.print("v_x_des:  ");
  Serial.print(v_x_des);
  Serial.print("   v_x_cur:  ");
  Serial.print(v_x_cur);
  Serial.print("\n");


}
