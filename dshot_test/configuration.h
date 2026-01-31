
// Drive motor related settings
bool motors_on = true;                      // turns on and off all motors in one boolean. 
#define max_throttle 255                    // max throttle value  
// INDIVIDUAL_A_B_C_D - for 1x4 individual motors.
// PARALEL_AC_BD      - for 2x2 motors with A C outputs and B D outputs being paralel.
// DSHOT              - if brushless motors ar used for drive 
uint8_t MOTOR_LAYOUT = PWM;       // selects motor layout betwheen 2 paralel, and 4 individual
uint8_t MOTOR_DIRECTION = 0; // 0 - normal, 1 - reversed
float battery_critical_v = 3.0;
#define cell_count 3.0;

// servo
int servo_1_init_position = 0;              // initial servo position
int servo_2_init_position = 0;              // initial servo position

int servo_1_failsave_position = 0;          // initial servo position
int servo_2_failsave_position = 0;          // initial servo position

bool arming_throttle_protection = true;     // prevents esc from spinning if armed with above 0 throttle

// Wifi related settings
const char* ssid = "PID_SETUP_WIFI";
const char* password = "12345678";

// imu settings
bool use_imu_for_yaw_rate = true;           // if true, yaw rate will be adjusted from imu yaw rate
#define max_yaw_rate 400                    // max yaw rate in degrees per second

// failsave
unsigned int failsave_delay = 500;          // ms until failsave is activated.
#define WDT_TIMEOUT 1                       // watchdog timeout (1s)
esp_task_wdt_config_t twdt_config = 
{
  .timeout_ms = WDT_TIMEOUT,
  .idle_core_mask = 0,
  .trigger_panic = true,
};

// hardware pins
#define SERVO_1_PIN 38
#define SERVO_2_PIN 37
#define PWM_DRIVE_2_PIN 36
#define PWM_DRIVE_1_PIN 35
#define BUTTON 4

// rev 2.0
// #define VSENSE 10

// rev v2.1
#define VSENSE 9
// #define VSENSE 10 // changed on rev_2.1 brain boards. can be used to detect voltage of extra external battery