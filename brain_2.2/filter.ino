
void update_filter(){
  IMU.update();
  IMU.getGyro(&gyroData);
  IMU.getAccel(&accelData);
  float elapsed_time = 1E-6 * (micros() - start_time); // in seconds
  filtered_signal = f.filter(gyroData.gyroZ, elapsed_time);
}

void init_filter(){
  f.begin(FREQUENCY, MINCUTOFF, BETA);  
  start_time = micros();  
}