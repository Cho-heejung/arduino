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

// Distance sensor
#define _DIST_ALPHA 0.3 

// Servo range
#define _DUTY_MIN 1250   
#define _DUTY_NEU 1450     
#define _DUTY_MAX 1650              

// Servo speed control
#define _SERVO_ANGLE 10 
#define _SERVO_SPEED 50           

// Event periods
#define _INTERVAL_DIST 20   
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 0.05   

//filter
#define LENGTH 30
#define k_LENGTH 8
#define Horizontal_Angle 2160
#define Max_Variable_Angle 100

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  
// Distance sensor
float dist_target;
float dist_raw, dist_ema, ir_filtered_dist, alpha, dist_cali; 

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr, duty_neutral; 

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; 

int a, b; // unit: mm
int correction_dist, iter;
float dist_list[LENGTH], sum;


void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT); 
  myservo.attach(PIN_SERVO); 
// initialize global variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false; 
  dist_target = _DIST_TARGET; 
  ir_filtered_dist = 0;
  duty_neutral = _DUTY_NEU;
  
  a = 20;
  b = 223;
  correction_dist = 0;
  iter = 0; sum = 0;
  alpha = _DIST_ALPHA;

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
    ir_filtered_dist = ir_distence_filter(); 

  // PID control logic
    error_curr = dist_target - ir_filtered_dist;
    pterm = _KP * error_curr; 
    control = pterm;

  // duty_target = f(duty_neutral, control)
    duty_target = pterm * duty_neutral;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  
    }else if (duty_target > _DUTY_MAX) {
      duty_target = _DUTY_MAX; 
    } 
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
    Serial.print(dist_cali);
    Serial.print(",pterm:");
    Serial.print(pterm);
//    Serial.print(",duty_target:");
//    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
//    Serial.print(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    Serial.print(",dist_ema:");
    Serial.println(dist_ema);
  }
}

float ir_distance(void)
{ 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;        
}

float ir_distence_filter() {
  sum = 0;
  iter = 0;
  dist_raw = ir_distance();
  while (iter < LENGTH)
  {
    dist_list[iter] = 100 + 300.0 / (b - a) * (dist_raw - a);
    sum += dist_list[iter];
    iter++;
  }

  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  dist_cali = sum/(LENGTH-2*k_LENGTH);

  dist_ema = alpha * dist_cali + (1 - alpha) * dist_ema;

  return dist_ema;
}
