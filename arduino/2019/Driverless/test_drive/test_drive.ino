
#define FWD_LEFT 3
#define BCK_LEFT 5

#define FWD_RIGHT 6
#define BCK_RIGHT 9

int radius = 1;

// Variables:
unsigned long t;
unsigned long t_prev;
unsigned long dt;

int w = 0;
int v = 0;

int w_des = 0;
int v_des = 0;

int w_max = 128;
int v_max = 128;

int a_max = 128;
int w_dot_max = 128;

int wheelSpeeds[2] = {0, 0};

int t_switch = 8000;
int rem;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(FWD_LEFT, OUTPUT);
  pinMode(BCK_LEFT, OUTPUT);
  pinMode(FWD_RIGHT, OUTPUT);
  pinMode(BCK_RIGHT, OUTPUT);

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

  // Left
  wheelSpeeds[0] = v - w * radius;

  // Right
  wheelSpeeds[1] = v + w * radius;
  return wheelSpeeds;
}

void writeWheelSpeeds(int wheelSpeeds[2]) {
  int LW = -wheelSpeeds[0];
  int RW = wheelSpeeds[1];

  Serial.print("LW: ");
  Serial.print(LW);
  Serial.print("    ");

  Serial.print("RW: ");
  Serial.print(RW);
  Serial.print("\n");

  if (LW > 0) {
    analogWrite(BCK_LEFT, 0);
    analogWrite(FWD_LEFT, LW);
  }
  else if (LW < 0) {
    analogWrite(BCK_LEFT, -LW);
    analogWrite(FWD_LEFT, 0);
  }
  else {
    analogWrite(BCK_LEFT, 0);
    analogWrite(FWD_LEFT, 0);
  }

  if (RW > 0) {
    analogWrite(BCK_RIGHT, 0);
    analogWrite(FWD_RIGHT, RW);
  }
  else if (RW < 0) {
    analogWrite(BCK_RIGHT, -RW);
    analogWrite(FWD_RIGHT, 0);
  }
  else {
    analogWrite(BCK_RIGHT, 0);
    analogWrite(FWD_RIGHT, 0);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  // Timing
  t = millis();
  dt = t - t_prev;

  t_prev = t;

  rem = t % t_switch;

  if (rem < (t_switch / 3)) {
    /// Go forward
    w_des = 0;
    v_des = v_max;
  }
  else if(rem < (2 * t_switch / 3)) {
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
  v = v + a_max * dt / 1000 * sign(v_des - v);
  w = w + w_dot_max * dt / 1000 * sign(w_des - w);

  VW2wheelSpeeds(v, w, wheelSpeeds);
  writeWheelSpeeds(wheelSpeeds);
}
