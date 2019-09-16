#include <hcsr04.h>
//#include <HCSR04.h>
#include <Servo.h>              // servo



#define Nsensors 3

// Use this to nuber and label your sensors
#define TRIG_PIN 8 // One trigger pin for all sensors
const int ECHO_PINS[Nsensors] = {9, 10, 11};
String SensorNames[Nsensors] = {"X+", "Y+", "X-"};

// Initialize counters etc.
int count = 0;
int idx = 0;
unsigned long t = 0;
unsigned long t_old = 0;

// Sensor range in mm
int minRange = 5;
int maxRange = 4000;

// Array of pointers to sensor objects. (Not exactly sure how it works but it works.)
HCSR04 *sensorArray[Nsensors];

double distances[Nsensors];
double dist_filt[Nsensors];

//unsigned long dt = 20;

// Sync sample time with filter_design M-file.
unsigned long dt = 160 / Nsensors;

void setup() {
  Serial.begin(9600);

  // Initialize sensors
  for (int i = 0; i < Nsensors; i++) {
    sensorArray[i] = new HCSR04(TRIG_PIN, ECHO_PINS[i], minRange, maxRange);
  }
}

double readSensor(int sensorNumber) {
  // Declarevariables
  double dist;
  HCSR04* sensor = sensorArray[sensorNumber];

  // Use pointer to sensor object to call distance function
  dist = sensor->distanceInMillimeters();
  return dist;
}


void loop() {

  // Timing
  t = millis();
  if (t - t_old > dt) {

    // Save current time
    t_old = t;

    // Index of current sensor, every loop pick a different one.
    idx = count % Nsensors;

    // Update distance array.
    distances[idx] = readSensor(idx);

    // Apply filter
    dist_filt[idx] = 0.4 * dist_filt[idx] + 0.6 * distances[idx];

    // Print everything
    Serial.print(SensorNames[idx]);
    Serial.print(distances[idx]);
    Serial.print("\n");

    Serial.print("t:");
    Serial.print(t);
    Serial.print("\n");

    digitalWrite(TRIG_PIN, LOW);
    count++;
  }
}
