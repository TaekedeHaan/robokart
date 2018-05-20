// set pins
#define RPIN 5
#define GPIN 6
#define BPIN 3
#define ESCPIN A0      // pin from receiver

// set prefquencies
#define DTPRINT 200   // [ms] for printing
#define DTLED 100     // [ms] for updating LEDs
#define DTREAD 100    // [ms] for reading velocity value

// recever output PWM width
#define AVG 1500
#define MAX 2000
#define MIN 1000

// led strip const
#define OUTMAX 255
#define OUTMIN 0
#define SLOPE 255/500
#define LEDINT 0.95

unsigned long t;
unsigned long tPrintPrev = 0;
unsigned long tReadVelPrev = 0;
unsigned long tLedPrev = 0;

// init RGB
int r;
int g;
int b;

// inputs
double velWidth;
 
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
  }

  if (t - tLedPrev > DTLED){
    tLedPrev = t;
    
    /* color function */
    r = OUTMAX + SLOPE * pow(velWidth - MAX, LEDINT);
    g = OUTMAX - SLOPE * pow(abs(-velWidth + AVG), LEDINT);
    b = OUTMAX + SLOPE * pow(-velWidth + MIN, LEDINT);
  
    /* saturate */
    r = min(OUTMAX, max(OUTMIN, r));
    g = min(OUTMAX, max(OUTMIN, g));
    b = min(OUTMAX, max(OUTMIN, b));

    /* output new LED values */
    analogWrite(RPIN, r);
    analogWrite(GPIN, g);
    analogWrite(BPIN, b);
  }
  
  // print 
  if (t - tPrintPrev > DTPRINT){
    tPrintPrev = t;
    
    Serial.print(F("esc width: "));
    Serial.print(velWidth);   
    Serial.print(F(" [ns]"));
    Serial.println("");
  }
}
