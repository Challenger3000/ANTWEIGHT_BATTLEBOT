
// Drive motor related settings
bool motors_on = true;                  // turns on and off all motors in one boolean. 

// INDIVIDUAL_A_B_C_D - for 1x4 individual motors.
// PARALEL_AC_BD      - for 2x2 motors with A C outputs and B D outputs being paralel.

uint8_t MOTOR_LAYOUT = PARALEL_AC_BD;   // selects motor layout betwheen 2 paralel, and 4 individual

// servo
int servo_1_init_position = 0;          // initial servo position
int servo_2_init_position = 0;          // initial servo position

int servo_1_failsave_position = 0;          // initial servo position
int servo_2_failsave_position = 0;          // initial servo position

// Wifi related settings
const char* ssid = "TEST_WIFI";
const char* password = "12345678";

// imu settings
bool use_imu_for_yaw_rate = true;       // if true, yaw rate will be adjusted from imu yaw rate

// hardware pins
#define SERVO_1_PIN 38
#define SERVO_2_PIN 37