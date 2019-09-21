
#define DIR_LEFT 3 // A-IB
#define VEL_LEFT 5 // A-IA

#define DIR_RIGHT 6 // B-IB
#define VEL_RIGHT 9 // B-IA

int radius = 1;

// Variables:
unsigned long t;
unsigned long t_prev;
unsigned long dt;

int w = 0;
int v = 0;

int w_des = 0;
int v_des = 0;

int w_max = 255;
int v_max = 255;

int a_max = 510;
int w_dot_max = 510;

int wheelSpeeds[2] = {0, 0};

int t_switch = 10000;
int rem;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(VEL_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(VEL_RIGHT, OUTPUT);

}

// Helper functions
int sign(int val) {
  int sgn = 1;
  if (val < 0) {
    sgn = -1;
  }
  if (val == 0) {
    sgn = 0;
  }
  return sgn;
}

void VW2wheelSpeeds(int v, int w, int wheelSpeeds[2]) {
  int diff;

  v = (float)v / (float)v_max * 255;
  w = (float)w / (float)w_max * 255;

  w = min(abs(w) * radius, 255) / radius * sign(w);

  Serial.print("v: ");
  Serial.print(v);
  Serial.print("    ");
  
  Serial.print("w: ");
  Serial.print(w);
  Serial.print("\n");


  // Left
  wheelSpeeds[0] = v - w * radius;

  // Right
  wheelSpeeds[1] = v + w * radius;

  // In case of actuator windup
  diff = (max(abs(wheelSpeeds[0]), 255) - 255) * sign(wheelSpeeds[0]);
  wheelSpeeds[0] = wheelSpeeds[0] - diff;
  wheelSpeeds[1] = wheelSpeeds[1] - diff;

  diff = (max(abs(wheelSpeeds[1]), 255) - 255) * sign(wheelSpeeds[1]);
  wheelSpeeds[1] = wheelSpeeds[1] - diff;
  wheelSpeeds[0] = wheelSpeeds[0] - diff;
}

void writeWheelSpeeds(int wheelSpeeds[2]) {
  int LW = wheelSpeeds[0];
  int RW = -wheelSpeeds[1];

  if (LW > 0) {
    digitalWrite(DIR_LEFT, LOW);
    analogWrite(VEL_LEFT, LW);
    //    digitalWrite(VEL_LEFT, HIGH);
  }
  else if (LW < 0) {
    digitalWrite(DIR_LEFT, HIGH);
    analogWrite(VEL_LEFT, 255 + LW);
    //    digitalWrite(VEL_LEFT, LOW);
  }
  else {
    digitalWrite(DIR_LEFT, LOW);
    analogWrite(VEL_LEFT, 0);
    //    digitalWrite(VEL_LEFT, LOW);
  }

  if (RW > 0) {
    digitalWrite(DIR_RIGHT, LOW);
    analogWrite(VEL_RIGHT, RW);
    //    digitalWrite(VEL_RIGHT, HIGH);
  }
  else if (RW < 0) {
    digitalWrite(DIR_RIGHT, HIGH);
    analogWrite(VEL_RIGHT, 255 + RW);
    //    digitalWrite(VEL_RIGHT, LOW);
  }
  else {
    digitalWrite(DIR_RIGHT, LOW);
    analogWrite(VEL_RIGHT, 0);
    //    digitalWrite(VEL_RIGHT,LOW);

  }
}

void loop() {
  // put your main code here, to run repeatedly:

  // Timing
  t = millis();
  dt = t - t_prev;

  t_prev = t;

  rem = t % t_switch;

  //  w_des = 0;
  //  v_des = v_max;


  if (rem < (t_switch / 3)) {
    /// Go forward
    w_des = 0;
    v_des = v_max;
  }
  else if (rem < (2 * t_switch / 3)) {
    // Turn left
    w_des = w_max;
    v_des = 0;
  }
  else {
    // Turn right
    w_des = -w_max;
    v_des = 0;
  }


  // Update actual velocity according to acceleration
  v = v_des; // v + a_max * dt / 1000 * sign(v_des - v);
  v = min(v, v_max);
  v = max(v, -v_max);

  w = w_des; // w + w_dot_max * dt / 1000 * sign(w_des - w);
  w = min(w, w_max);
  w = max(w, -w_max);

  VW2wheelSpeeds(v, w, wheelSpeeds);
  writeWheelSpeeds(wheelSpeeds);
}
