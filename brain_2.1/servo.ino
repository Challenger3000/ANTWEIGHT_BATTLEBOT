
void init_servo(){
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	
	servo_1.setPeriodHertz(50);
	servo_1.attach(SERVO_1_PIN, 1000, 2000);
  servo_1.write(servo_1_init_position);
	
	servo_2.setPeriodHertz(50);
	servo_2.attach(SERVO_2_PIN, 1000, 2000);
  servo_2.write(servo_2_init_position);
}

// checks that servos where at 0 throttle before applying signal
// protects against arming with servo positon different from failsave
// (no arming with weapon motor on)
void check_servo_0_before_arming(){
  if(map(rxData.pot_1,0,4950,0,180) == 0){
    servo_1_was_0_before_arming = true;
    servo_2_was_0_before_arming = true;
  }
}
