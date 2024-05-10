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
3. 2x servo outputs on pins IO37 and IO38
4. PID control loop to aid in driving
5. Website hosting over wifi for PID/Configuration setting changes.

To compile this example...
1. install Arduino IDE 2.3.2 from https://www.arduino.cc/en/software
2. add "Arduino ESP32 Boards By Arduino" and "esp32 by Espressif" following this tutorial:
https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
3. Try to compile, flash/verify, and install all missing libraries.
4. Connect robot, select com port and chose ESP32S3 Dev
5. Set these parameters in arduino ide:

![image](https://github.com/Challenger3000/ANTWEIGHT_BATTLEBOT/assets/73142814/3b4f20c7-2340-4be0-b26f-d579e45eb88b)
7. Upload code, if exit status 1 or status 2, then hold the programming button while adding power to robot controller, and retry upload.  

Binding process...
1. Hold down "B" button on transmitter and turn it ononce you see the second rgb led from the left side blinking blue and cyan.
2. Hold top side button on the robot while turning the robot on until you see rgb led blinking blue and cyan.
3. Keep the robots withing 2m or so, it should take around 5 seconds to bind.
   
Motor driver abstraction...
The motor driver onboard has 4 full bridges. So its capable of driving 4 1.0 - 1.5 A motors, or 2 2.0 - 3.0 A motors
You have to chose the descired configuration in configuration.h by setting 
MOTOR_LAYOUT = PARALEL_AC_BD or INDIVIDUAL_A_B_C_D
After initializing the motor driver, you can simply use drive_motor_A() and motor A will drive.
If you're using 2 parallel motors and not 4 individual motors, motor A C, and B D, will work together to achieve more current.
For more detailed information for the motor driver, you can check out the datasheet: 
https://www.ti.com/lit/ds/symlink/drv8912-q1.pdf?ts=1715276670041&ref_url=https%253A%252F%252Fwww.google.com%252F
The motor driver has OVER CURRENT PROTECTION, if robot detects the over current protection it will flash onboard status led YELOW.

PID control...
The robot and remote has a built-in imu in both robot and transmitter. Currently not supported on transmitters side. IMU used is LSM6DSLTR, SDA is connected to io48 and SCL to io47, in case you want to add support.
By default, robot comes with, 3 modes on 2nd switch. Off on the bottom, On no IMU in the middle, On with imu stabilisation pointing forwards.
When in imu stabilisation mode, robot will track rotational speed.
For example if the sitck is in the middle, the robot will try to stay motionless, it will resist pushes and hit better.
If you rotate your stick as far to the left as you can, it will by default try to maintain 600 deg/s. This can be changed in configuration.h.

Website hosting over wifi...
For imu PIDs to work you need to tune them. the easiest way to do that is by using wifi website mode. 
To aces this website press the general purpose button while the robot is on, led will change to purple and controll will be lost.
Then you can connect to wifi, by default its called PID_SETUP_WIFI with pasword 12345678 . PLEASE CHANGE THIS TO YOUR OWN WIFI NAME AND PASS.
After your done changing pids, you click send, if the robot receives the changes, it will flash white once. then you can click the button again to go back to driving mode.
The website being hosted can be changed, in wifi.ino file.

Robot led status explanations...
Red - not connected,
Red blinking - Calibrating IMU
Green - connected, 
Yellow - motor driver over current

Robot board consists of main brain with the driver, and imu. and an addon charger board. 


![image](https://github.com/Challenger3000/ANTWEIGHT_BATTLEBOT/assets/73142814/5f309d43-2840-40ca-801e-7a01e9710031)
![image](https://github.com/Challenger3000/ANTWEIGHT_BATTLEBOT/assets/73142814/db7778c4-27f8-436b-be8a-a035e42542d5)
![image](https://github.com/Challenger3000/ANTWEIGHT_BATTLEBOT/assets/73142814/d8dc95cc-22ef-4ea0-a8a9-47e5171e8d5c)
Charger board has solder jumpers to sellect battery cell count.

Transmitter has 4 adressable leds

![image](https://github.com/Challenger3000/ANTWEIGHT_BATTLEBOT/assets/73142814/c964265b-a961-4c52-9257-dbc815d25be4)

By default 1st one from the left is the transmitter battery status with the following colors
green - yellow - red - red blinking.... all of them represent the charge state of the remote.

2nd one is robot status:
green - yellow - red - red blinking.... all of them represent the charge state of the robot.
Blue/cyan blinking - binding mode


Keep in mind!
Esp32 s3 doesn't like Printing in serial while no serial connection is present, so comment out all serial print or use switches on the transmitter to enable/disable serial prints on the robot/transmitter.
If you do a lot of printing, with no serial cable attached, ESP will start to lag
