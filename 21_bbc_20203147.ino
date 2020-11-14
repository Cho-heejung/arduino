#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_SERVO 10

#define _DUTY_MIN 1300
#define _DUTY_NEU 1450
#define _DUTY_MAX 1600

#define INTERVAL 20  // servo update interval
#define _DIST_ALPHA 0.3 // EMA weight of new sample

int a, b; // unit: mm
Servo myservo;
int servo_neu;
float dist_ema, alpha;
unsigned long last_sampling_time; // unit: ms

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
  Serial.begin(57600);

  a = 69;
  b = 290;

  servo_neu = 87;
  last_sampling_time = 0;
  alpha = _DIST_ALPHA;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);

  dist_ema = alpha * dist_cali + (1-alpha) * dist_ema;

  Serial.println(dist_ema);

  if (myservo.read() == servo_neu) {
    if (dist_ema > 255) myservo.writeMicroseconds(_DUTY_MIN);
    else if (dist_ema < 255) myservo.writeMicroseconds(_DUTY_MAX);
  }
  else if (myservo.read() < servo_neu && dist_ema < 255) myservo.writeMicroseconds(_DUTY_MAX);
  else if (myservo.read() > servo_neu && dist_ema > 255) myservo.writeMicroseconds(_DUTY_MIN);

  last_sampling_time += INTERVAL;
}
