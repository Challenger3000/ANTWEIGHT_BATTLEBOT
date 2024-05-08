
void init_pid(){
  Setpoint = 0.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-2048,2048);
  myPID.SetTunings(EEPROM_DATA.PID_P,EEPROM_DATA.PID_I,EEPROM_DATA.PID_D);
  myPID.SetSampleTime(1);
  // Serial.print("EEPROM PID PARAMS P: ");
  // Serial.print(EEPROM_DATA.PID_P);
  // Serial.print("\tI: ");
  // Serial.print(EEPROM_DATA.PID_I);
  // Serial.print("\tD: ");
  // Serial.println(EEPROM_DATA.PID_D);
}

void update_pid(){

  // IMU.update();
  // IMU.getGyro(&gyroData);
  
  Input = (double)filtered_signal;
  myPID.Compute();

  // if(rxData.sw_1 == 3){
  //   Serial.print("set:");
  //   Serial.print(Setpoint,3);
  //   Serial.print(",filt:");
  //   Serial.print(filtered_signal,3);
  //   Serial.print(",raw_out:");
  //   Serial.print(Output);
  //   Serial.print(",core:");
  //   Serial.println(xPortGetCoreID());
  // }

}