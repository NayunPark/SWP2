#include <Servo.h>

// Arduino pin assignment
#define PIN_IR    A0         // IR sensor at Pin A0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 1000       // servo full clock-wise position (0 degree)
#define _DUTY_MAX 2000       // servo full counter-clockwise position (180 degree)

#define _DIST_MIN  100.0     // minimum distance 100mm
#define _DIST_MAX  250.0     // maximum distance 250mm

#define EMA_ALPHA  0.9       // for EMA Filter
#define LOOP_INTERVAL 20     // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time;   // unit: msec

float dist_ema = _DIST_MIN;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  
  myservo.attach(PIN_SERVO); 
  myservo.write(0);  // Start at 0 degrees for visual reference
  
  Serial.begin(1000000);    // 1,000,000 bps
}

void loop()
{
  unsigned long time_curr = millis();
  int angle;
  float a_value, dist_raw;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  a_value = analogRead(PIN_IR);
  dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0;

  // Apply EMA filter
  dist_ema = EMA_ALPHA * dist_ema + (1.0 - EMA_ALPHA) * dist_raw;

  // LED control
  if (dist_ema >= _DIST_MIN && dist_ema <= _DIST_MAX) {
    digitalWrite(PIN_LED, HIGH); // Turn on LED if within range
  } else {
    digitalWrite(PIN_LED, LOW); // Turn off LED if out of range
  }

  // Direct calculation for servo angle without map function
  if (dist_ema >= _DIST_MIN && dist_ema <= _DIST_MAX) {
    angle = 0 + (dist_ema - _DIST_MIN) * 180 / (_DIST_MAX - _DIST_MIN);
    myservo.write(angle);  // Move servo to calculated angle
  }

  // Serial output for debugging
  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print(",_DIST_MIN:"); Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo angle:"); Serial.print(angle);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print("_DUTY_MAX:");  Serial.print(_DUTY_MAX);
  Serial.println("");
}
