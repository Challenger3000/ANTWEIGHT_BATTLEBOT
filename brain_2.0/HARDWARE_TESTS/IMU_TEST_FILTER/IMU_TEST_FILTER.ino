#include "FastIMU.h"
#include <Wire.h>
#define IMU_ADDRESS 0x6B    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
LSM6DSL IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658
calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;




// filter code start
#include "1euroFilter.h"

static OneEuroFilter f; // not enabled yet, setup has to be called later

// Frequency of your incoming noisy data
// If you are able to provide timestamps, the frequency is automatically determined
#define FREQUENCY   60   // [Hz] 
#define MINCUTOFF   1.0   // [Hz] needs to be tuned according to your application
#define BETA        0.04   // needs to be tuned according to your application

unsigned long start_time;
float filtered_signal;

void update_filter(){
  IMU.update();
  IMU.getGyro(&gyroData);
  float elapsed_time = 1E-6 * (micros() - start_time); // in seconds
  filtered_signal = f.filter(gyroData.gyroZ, elapsed_time);
}

void init_filter(){
  f.begin(FREQUENCY, MINCUTOFF, BETA);  
  start_time = micros();
}
// filter code end










void setup() {  
  // calib.accelBias[0] = -0.02;
  // calib.accelBias[1] = -0.03;
  // calib.accelBias[2] = 0.03;

  // calib.gyroBias[0] = 0.44;
  // calib.gyroBias[1] = -2.79;
  // calib.gyroBias[2] = -0.46;

  Wire.setPins(48, 47);
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(1000000);
  while (!Serial) {
    ;
  }
  Serial.print("Starting...");

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");


  delay(2000);
  Serial.println("Keep IMU level.");
  delay(3000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  delay(5000);
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

  init_filter();
}



void loop() {

  update_filter();
  Serial.print(gyroData.gyroZ);
  Serial.print("\t");
  Serial.println(filtered_signal);

  // if (IMU.hasTemperature()) {
	//   Serial.print("\t");
	//   Serial.println(IMU.getTemp());
  // }

  delay(5);
}
