// set pins
#define RPIN 5
#define GPIN 6
#define BPIN 3
#define ESCPIN A0      // pin from receiver

// set prefquencies
#define DTPRINT 200   // [ms] for printing
#define DTLED 50     // [ms] for updating LEDs
#define DTREAD 100    // [ms] for reading velocity value
#define DTBLINK 200

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

unsigned long t;
unsigned long tPrintPrev = 0;
unsigned long tReadVelPrev = 0;
unsigned long tLedPrev = 0;
unsigned long tBlinkPrev = 0;

int brakeCount = 0; // brake counter

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

  Serial.println(F("Complete setup!")); Serial.println("");
}
 
 
void loop() {

  t = millis();

  if (t - tReadVelPrev > DTREAD){
    tReadVelPrev = t;
    
    /* read receiver, pulse length from 1ms (zero throttle) to 2ms (full throttle) */
    velWidth = pulseIn(ESCPIN, HIGH); //[us] read pwm pin

    if (abs(velWidth - AVG) < MARGIN){
      velWidth = AVG;
    }

    /* count braking */
    if (prevVelWidth >= AVG){

      /* If we are braking and previously weren't */
      if (velWidth < AVG){
        brakeCount = brakeCount + 1;
      }
    }

    /* if we are floowing that gass pedal again reset the braking counter */
    if (velWidth > AVG) {
      brakeCount = 0;
    }
    
    prevVelWidth = velWidth;
  }

  if (t - tLedPrev > DTLED){
    tLedPrev = t;

    /* color function */
    if (brakeCount == 0){
      r = OUTMAX + pow(SLOPE * (velWidth - MAX),LEDINT);
      g = OUTMAX - pow(SLOPE * abs(-velWidth + (AVG + MAX)/2),LEDINT);
      b = OUTMAX + pow(SLOPE * (-velWidth + AVG),LEDINT);
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

    /* output new LED values */
    analogWrite(RPIN, r);
    analogWrite(GPIN, g);
    analogWrite(BPIN, b);
  }
  
  /* print */
  if (DEBUG) {
    if (t - tPrintPrev > DTPRINT){
      tPrintPrev = t;
      
      Serial.print(F("esc width: "));
      Serial.print(velWidth);   
      Serial.print(F(" [ns]"));

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
