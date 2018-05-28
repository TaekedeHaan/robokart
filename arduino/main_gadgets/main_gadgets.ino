// set pins
#define RPIN 5
#define GPIN 6
#define BPIN 3
#define ESCPIN A0     // pin from receiver
#define LFPIN 7 // left forward
#define RFPIN 8 // right forward
#define LBPIN 9 // left back
#define RBPIN 10 // right back
#define SERVOPIN 11 // servo pin

// set prefquencies
#define DTPRINT 200   // [ms] for printing
#define DTLED 50      // [ms] for updating LEDs
#define DTREAD 20    // [ms] for reading velocity value
#define DTBLINK 200
#define DTWEAPON 50   // [ms]
#define DTTHROTTLE 2000 // [ms]
#define DTACTION 100 // [ms]

// recever output PWM width
#define AVG 1500
#define MAX 2000
#define MIN 1000
#define MARGIN 80

// led strip const
#define OUTMAX 255
#define OUTMIN 0
#define SLOPE 1.0200 //0.2550 //255/500
#define LEDINT 0.95

# define DEBUG true

boolean releaseWeapon = false;

unsigned long t;
unsigned long tPrintPrev = 0;
unsigned long tReadVelPrev = 0;
unsigned long tLedPrev = 0;
unsigned long tBlinkPrev = 0;
unsigned long tActionPrev = 0;
unsigned long tThrottlePrev = 0;

int brakeCount = 0; // brake counter
int throttleCount = 0; // throttle counter
int throttleThreshold = 3; // threshold

// init 
int lf = LOW;
int rf = LOW;
int lb = 0;
int rb = 0;

// init RGB
double r;
double g;
double b;

// inputs
double velWidth;
double prevVelWidth;
 
void setup() {
  Serial.begin(115200);
  
  pinMode(RPIN, OUTPUT);
  pinMode(GPIN, OUTPUT);
  pinMode(BPIN, OUTPUT);
  pinMode(ESCPIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  /* led pins*/
  pinMode(LFPIN, OUTPUT);
  pinMode(RFPIN, OUTPUT);
  pinMode(LBPIN, OUTPUT);
  pinMode(RBPIN, OUTPUT);  

  pinMode(SERVOPIN, OUTPUT);
  
  Serial.println(F("Complete setup!")); Serial.println("");
}
 
 
void loop() {

  t = millis();

  if (t - tReadVelPrev > DTREAD){
    tReadVelPrev = t;
    
    /* read receiver, pulse length from 1ms (zero throttle) to 2ms (full throttle) */
    velWidth = pulseIn(ESCPIN, HIGH); //[us] read pwm pin

    /* filter velocity in */
    if (abs(velWidth - AVG) < MARGIN){
      velWidth = AVG;
    }

    /* count braking */
    if (prevVelWidth >= AVG){ // if we were previously pressing gass

      /* If we are braking and previously weren't */
      if (velWidth < AVG){
        brakeCount = brakeCount + 1;
      }
    }

    /* count gass */
    if (prevVelWidth <= AVG){ // if we were previously not pressing gass

      /* If we are braking and previously weren't */
      if (velWidth > AVG){

        /* If within timer increment throttle count, otherwise reset */
        if (t - tThrottlePrev < DTTHROTTLE){
          throttleCount = throttleCount + 1;
        }
        else {
          tThrottlePrev = t;
          throttleCount = 0;
        }
      }
    }

    /* if we are pressing the gass pedal reset the braking counter */
    if (velWidth > AVG) {
      brakeCount = 0;
    }
    
    prevVelWidth = velWidth;
  }

  if (t - tLedPrev > DTLED){
    tLedPrev = t;

    /* color function */
    if (brakeCount == 0){
      r = OUTMAX + SLOPE * (velWidth - MAX);
      g = OUTMAX - SLOPE * abs(-velWidth + (AVG + MAX)/2);
      b = OUTMAX + SLOPE * (-velWidth + AVG);
    }
    else {
      r = OUTMAX - SLOPE * (velWidth - MIN);
      g = OUTMAX - SLOPE * abs(-velWidth + (AVG + MIN)/2);
      b = OUTMAX - SLOPE * (-velWidth + AVG); 
    }   
  
    /* saturate */
    r = min(OUTMAX, max(OUTMIN, r));
    g = min(OUTMAX, max(OUTMIN, g));
    b = min(OUTMAX, max(OUTMIN, b));

    /* if braking put LEDs to max */
    if (brakeCount == 1 && velWidth < AVG){
      r = OUTMAX;
      g = OUTMAX;
      b = OUTMAX;
    }

    /* Blink when going backwards */
    if (brakeCount >= 3){
      if (t - tBlinkPrev > DTBLINK){
        r = OUTMIN;
        g = OUTMIN;
        b = OUTMIN;
      }
      if (t - tBlinkPrev > 2 * DTBLINK){
        tBlinkPrev = t;
      }
    }

    /* LED's */
    lf = HIGH;
    rf = HIGH;
    rb = 255;
    lb = 255;

    /* output new LED values */
    analogWrite(RPIN, r);
    analogWrite(GPIN, g);
    analogWrite(BPIN, b);

    /* update LED's */
    digitalWrite(LFPIN, rf);
    digitalWrite(RFPIN, rf);
    analogWrite(LBPIN, lb);
    analogWrite(RBPIN, rb);
  }

  if (t - tActionPrev > DTACTION){

    t = tActionPrev;
    
    /* wehn sufficient amount of trottles set realease weapon flag */
    if (throttleCount >= throttleThreshold) {
      releaseWeapon = true;
    }

   /* release weapon*/
    if (releaseWeapon) {
      digitalWrite(LED_BUILTIN, releaseWeapon);
    }
  }
  
  /* print */
  if (DEBUG) {
    if (t - tPrintPrev > DTPRINT){
      tPrintPrev = t;

      /*
      Serial.print(F("esc width: "));
      Serial.print(velWidth);   
      Serial.print(F(" [ns]"));
      */
      Serial.print(F("r: "));
      Serial.print(r);   
      Serial.print("\t");              // prints a tab

      Serial.print(F("g: "));
      Serial.print(g);   
      Serial.print("\t");              // prints a tab

      Serial.print(F("b: "));
      Serial.print(b);   
      Serial.print("\t");              // prints a tab

      Serial.print(F("braking counter: "));
      Serial.print(brakeCount);   
      Serial.print("\t");              // prints a tab
      
      Serial.println("");
    }
  }
}
