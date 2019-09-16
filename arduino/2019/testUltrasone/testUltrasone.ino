#include <hcsr04.h>
//#include <HCSR04.h>
#include <Servo.h>              // servo

#define TRIG_PIN 8
#define ECHO_PIN_X_POS 9

//#define TRIG_PIN_Y_POS 8
#define ECHO_PIN_Y_POS 10

//#define TRIG_PIN_X_NEG 8
#define ECHO_PIN_X_NEG 11

//#define SERVO_PIN 3

// Bifrost library
int Nsensors = 3;
int count = 0;
int idx = 0;
unsigned long t = 0;
unsigned long t_old = 0;

int minRange = 5;
int maxRange = 4000;

HCSR04 Sensor_X_POS(TRIG_PIN, ECHO_PIN_X_POS, minRange, maxRange);
HCSR04 Sensor_Y_POS(TRIG_PIN, ECHO_PIN_Y_POS, minRange, maxRange);
HCSR04 Sensor_X_NEG(TRIG_PIN, ECHO_PIN_X_NEG, minRange, maxRange);

HCSR04 sensorArray[3] = {Sensor_X_POS, Sensor_Y_POS, Sensor_X_NEG};

double distances[3] = {0, 0, 0};

//unsigned long dt = 20;

// Sync sample time with filter_design M-file.
unsigned long dt = 160 / Nsensors;

void setup() {
  Serial.begin(9600);
}

double readSensor(int sensorNumber) {
  double dist;
  HCSR04 sensor = sensorArray[sensorNumber];

  dist = sensor.distanceInMillimeters();

  return dist;
}

void loop() {
  //  for (int i = 0; i <= Nsensors - 1; i++) {

  t = millis();
  if (t - t_old > dt) {

    t_old = t;

    idx = count % Nsensors;

    distances[idx] = readSensor(idx);

    switch (idx) {
      case 0:
        Serial.print("X+");
        Serial.print(distances[0]);
        Serial.print("\n");
        break;
      case 1:
        Serial.print("Y+");
        Serial.print(distances[1]);
        Serial.print("\n");
        break;
      case 2:
        Serial.print("X-");
        Serial.print(distances[2]);
        Serial.print("\n");
        break;
    }

    Serial.print("t:");
    Serial.print(t);
    Serial.print("\n");

    digitalWrite(TRIG_PIN, LOW);
    count++;
    //    t_old = millis();
  }
}
