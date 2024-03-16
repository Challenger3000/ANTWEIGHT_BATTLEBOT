// servo
const int servoPin = 21;
unsigned long servo_start_time = 0;
unsigned long servo_off_time = 0;
unsigned long servo__next_on_time = 0;
const int servoPeriod = 20000;
uint8_t current_duty = 0;
bool servo_on = true;

// servo manager
unsigned long last_flip = 0; 
bool flag=true;

void init_servo_pwm() {
  servo_start_time = micros();
  digitalWrite(servoPin, HIGH);
  servo_on = true;
  servo_off_time = servo_start_time + map(current_duty, 0, 255, 1000, 2000);
  servo__next_on_time = servo_start_time + servoPeriod;
}



void update_servo() {
  unsigned long currentTime = micros();
  if(servo_on){
    if (currentTime >= servo_off_time) {
      digitalWrite(servoPin, LOW);
      servo_on = false;
    }
  }else{
    if (currentTime >= servo__next_on_time) {
      digitalWrite(servoPin, HIGH);
      servo_on = true;
      servo_off_time = servo__next_on_time + map(current_duty, 0, 255, 1000, 2000);
      servo__next_on_time += servoPeriod;
    }
  }
}

void setup() {
  pinMode(servoPin, OUTPUT);
  init_servo_pwm();
}

void loop() {
  if(millis()-last_flip>1000){
    last_flip = millis();
    if(flag){
      flag=false;
      current_duty=255;
    }else{
      flag=true;
      current_duty=0;
    }
  }
  update_servo();
  
}
