
// servo code start
#include <ESP32Servo.h>
Servo myservo;
Servo myservo2;
int servo_position = 0;
#define SERVO_1 38
#define SERVO_2 37

void init_servo(){
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);
	myservo.attach(SERVO_1, 500, 2500);
  
	myservo2.setPeriodHertz(50);
	myservo2.attach(SERVO_2, 500, 2500);
}
// servo code end

void setup() {
  init_servo();
}

void loop() {
  myservo.write(0);
  delay(700);
  myservo2.write(0);
  delay(700);
  myservo.write(180);
  delay(700);
  myservo2.write(180);
  delay(700);
}