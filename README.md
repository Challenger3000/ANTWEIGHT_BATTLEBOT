"# ANTWEIGHT_ESP_BATTLEBOT" 

This is a repository with some working example code that is meant to run an ESP32S3 battlebot and a transmitter 

The main brain of the robot has the following hardware
1. ESP32S3
2. DRV8908 brushed motor driver IC
3. 3.3v regulator
4. 5.0v regulator
5. LSM6DSLTR Gyro/accelerometer
6. individually addressable rbg leds

The main code example includes features like...
1. Binding betwheen robot ant transmitter, with encryption and variable frequency
2. Abstracted interface with motor driver
3. Simple input mixing for driving,
4. 2x servo outputs on pins IO37 and IO38
5. PID control loop to aid in driving
6. Website 

To compile this example...
1. install Arduino IDE 2.3.2 from https://www.arduino.cc/en/software
2. add "Arduino ESP32 Boards By Arduino" and "esp32 by Espressif" following this tutorial:
https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
3. Try to compile, flash/verify, and install all missing libraries.

Binding proces...
1. Hold down "B" button on transmitter and turn it ononce you see the second rgb led from the left side blinking blue and cyan.
2. Hold top side button on the robot while turning the robot on until you see rgb led blinking blue and cyan.
3. Keep the robots withing 2m or so, it should take around 5 seconds to bind.
   
Motor driver abstraction...
The motor driver onboard has 4 full bridges. So its capable of driving 4 1.0 - 1.5 A motors, or 2 2.0 - 3.0 A motors
You have to chose the descired configuration in configuration.h by setting 
MOTOR_LAYOUT = PARALEL_AC_BD or INDIVIDUAL_A_B_C_D
After initialising the motor driver, you can simply use drive_motor_A() and motor A will drive.
If your using 2 paralel motors and not 4 individual motors, motor A C, and B D, will work together to acheave more current.
