/**
 * @file throttle_percent_serial.ino
 * @brief Read throttle percentage from Serial (0–100) and output via DShot
 */

#include <Arduino.h>
#include <DShotRMT.h>

static constexpr gpio_num_t MOTOR01_PIN = GPIO_NUM_36;
static constexpr dshot_mode_t DSHOT_MODE = DSHOT600;
static constexpr bool IS_BIDIRECTIONAL = false;

// ESC expects 0–2000 throttle command
// 48–2047 are valid throttle values (0–47 = special commands)
// We'll map 0–100% into 48–2047
static constexpr int DSHOT_MIN_THROTTLE = 48;
static constexpr int DSHOT_MAX_THROTTLE = 2047;

DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL, 20);

int currentCmd = 0;

void setup()
{
    Serial.begin(115200);
    motor01.begin();

    Serial.println("Enter throttle percentage (0–100):");
}

void loop()
{
    // Check if anything was typed
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        int percent = input.toInt();

        // Clamp range
        percent = constrain(percent, 0, 100);

        // Convert percent → DShot value
        int throttleValue = map(percent,
                                0, 100,
                                DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);

        currentCmd = throttleValue;

        Serial.print("Set throttle: ");
        Serial.print(percent);
        Serial.print("%  -> DShot value: ");
        Serial.println(throttleValue);
    }

    // Send output continuously
    if (currentCmd > 0)
    {
        motor01.sendThrottle(currentCmd);
    }
    else
    {
        motor01.sendCommand(0); // motor stop
    }

    delay(5);  // keep ESC fed with packets
}
