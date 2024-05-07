
void init_servo(){
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	
	servo_1.setPeriodHertz(50);
	servo_1.attach(SERVO_1_PIN, 500, 2500);
  servo_1.write(servo_1_init_position);
	
	servo_2.setPeriodHertz(50);
	servo_2.attach(SERVO_2_PIN, 500, 2500);
  servo_2.write(servo_2_init_position);
}
