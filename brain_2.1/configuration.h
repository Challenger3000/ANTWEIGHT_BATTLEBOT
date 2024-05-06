
// Drive motor related settings
bool motors_on = true;                  // turns on and off all motors in one boolean. 

// INDIVIDUAL_A_B_C_D - for 1x4 individual motors.
// PARALEL_AC_BD      - for 2x2 motors with A C outputs and B D outputs being paralel.

uint8_t MOTOR_LAYOUT = PARALEL_AC_BD;   // selects motor layout betwheen 2 paralel, and 4 individual

int servo_position = 0;                 // initial servo position

// Wifi related settings
const char* ssid = "TEST_WIFI";
const char* password = "12345678";
