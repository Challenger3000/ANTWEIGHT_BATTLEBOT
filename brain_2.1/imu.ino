void init_imu(){
  Wire.setPins(48, 47);
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  for(int i=0; i<12; i++){
    led_color(128,0,0);
    delay(100);    
    led_color(10,0,0);
    delay(100);    
  }
    led_color(128,0,0);
  
#ifdef PERFORM_CALIBRATION
  // Serial.println("FastIMU calibration & data example");
  IMU.calibrateAccelGyro(&calib);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  led_color(0,10,0);
}

void imu_print(){
  IMU.update();
  IMU.getAccel(&accelData);
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.println(accelData.accelZ);
  // Serial.print("\t");
  IMU.getGyro(&gyroData);
  // Serial.print(gyroData.gyroX);
  // Serial.print("\t");
  // Serial.print(gyroData.gyroY);
  // Serial.print("\t");
  // Serial.print(gyroData.gyroZ);
  // Serial.print("\t");
  // Serial.println(IMU.getTemp());  
}