#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9                
#define PIN_SERVO 10    
#define PIN_IR A0     

// Framework setting
#define _DIST_TARGET 255  
#define _DIST_MIN 10                   
#define _DIST_MAX 410  

// Servo range
#define _DUTY_MIN 1050   
#define _DUTY_NEU 1450     
#define _DUTY_MAX 2050              

// Servo speed control
#define _SERVO_ANGLE 10
#define _SERVO_SPEED 2000           

// Event periods
#define _INTERVAL_DIST 20   
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 3.6
#define _KD 85  

#define _INTERVAL_DIST 30  
#define DELAY_MICROS  1500 
#define EMA_ALPHA 0.35     

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  
// Distance sensor
float dist_target;
float dist_raw, ir_filtered_dist, dist_cali; 

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr, duty_neutral; 

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; 

float ema_dist=0;          
float filtered_dist;      
float samples_num = 3;   

int a, b;


void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT); 
  myservo.attach(PIN_SERVO); 
// initialize global variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false; 
  dist_target = _DIST_TARGET; 
  duty_neutral = _DUTY_NEU;

  a = 70;
  b = 370;
  error_curr = error_prev = 0;

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); 
// initialize serial port
  Serial.begin(57600); 
// convert angle speed into duty change per interval.
  duty_chg_per_interval = int((_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (float(_INTERVAL_SERVO) / 1000.0));  
} 
  

void loop() {
/////////////////////
// Event generator // 
/////////////////////
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    float ir_filtered_dist = filtered_ir_distance();

  // PID control logic
    error_curr = dist_target - ir_filtered_dist;
    pterm = _KP * error_curr; 
    dterm = _KD * (error_curr - error_prev);
    control = pterm + dterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;  
    else if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; 
    
    error_prev = error_curr;
  }
  
  if(event_servo) {
    event_servo = false; 
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }  
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }       
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);   
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(ema_dist);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void)
{ 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;        
}

float under_noise_filter(void){ 
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ 
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  dist_cali = 100 + 300.0 / (b - a) * (lowestReading - a);
  // eam 필터 추가
  ema_dist = EMA_ALPHA*dist_cali + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
