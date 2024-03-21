
// servo code start
#include <ESP32Servo.h>
Servo myservo;
int servo_position = 0;
#define SERVO_1 38

void init_servo(){
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);
	myservo.attach(SERVO_1, 500, 2500);
}
// servo code end

void setup() {
  init_servo();
}

void loop() {
  myservo.write(0);
  delay(700);
  myservo.write(180);
  delay(700);
}