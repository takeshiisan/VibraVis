// ALL VALUES IN THIS FILE ARE PLACEHOLDERS AND SHOULD BE ADJUSTED ACCORDING TO HARDWARE SETUP
#ifndef CONFIG_H 
#define CONFIG_H

// ESP32-S3 Default I2C Pins (can be changed if needed)
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// I2C BUS 1 - Right Motor Driver (ESP32-S3's SECOND hardware I2C controller)
#define I2C_SDA_PIN2 15
#define I2C_SCL_PIN2 16

enum MotorSide { MOTOR_LEFT = 0, MOTOR_RIGHT, MOTOR_COUNT };
#define MASK_LEFT_MOTOR  (1 << MOTOR_LEFT)
#define MASK_RIGHT_MOTOR (1 << MOTOR_RIGHT)
#define MASK_BOTH_MOTORS (MASK_LEFT_MOTOR | MASK_RIGHT_MOTOR)

enum ZoneGroup { ZONE_LEFT = 0, ZONE_CENTER, ZONE_RIGHT, ZONE_GROUP_COUNT };

// Column grouping of the 8x8 grid (columns 0-7 per row)
#define ZONE_GRID_SIZE      8
#define ZONE_LEFT_MAX_COL   2  // columns 0,1,2 = LEFT
#define ZONE_RIGHT_MIN_COL  5  // columns 5,6,7 = RIGHT
                                // columns 3,4 = CENTER

enum WhiteLedMode {
  WHITE_LED_OFF,
  WHITE_LED_SOLID,
  WHITE_LED_STROBE
};

enum BatteryLevel {
  BATTERY_LEVEL_CRITICAL,
  BATTERY_LEVEL_LOW,
  BATTERY_LEVEL_MEDIUM,
  BATTERY_LEVEL_HIGH
};


// Thresholds and timings 
#define OBSTACLE_DETECTION_THRESHOLD_MM 1500
#define IMMEDIATE_DANGER_MM 300
#define SENSOR_POLL_INTERVAL_MS 100
#define RTP_MAX_AMPLITUDE 127
#define RTP_MIN_AMPLITUDE 40

// I2S AUDIO AMPLIFIER (MAX98357A)
#define I2S_BCLK 14
#define I2S_LRC  12
#define I2S_DOUT 13

//MAX17043
#define BATTERY_HIGH_THRESHOLD   75.0f // HIGH (green)
#define BATTERY_MEDIUM_THRESHOLD 50.0f // MEDIUM (yellow)
#define BATTERY_LOW_THRESHOLD    30.0f // LOW (red), anything bellow is critical 
#define BATTERY_CHECK_INTERVAL_MS 10000 // check battery every 10 seconds

// RGB LED (battery level indicator)
#define LED_RED_PIN   38
#define LED_GREEN_PIN 39
#define LED_BLUE_PIN  40

// Indicator WHITE LED
#define WHITE_LED_PIN 7
#define DOUBLE_CLICK_WINDOW_MS 400
#define STROBE_INTERVAL_MS 200
#define BUTTON_DEBOUNCE_MS 50

// TACTILE BUTTON
#define POWER_BUTTON 2
#define LONG_PRESS_MS 2000
#define LIGHT_BUTTON 1

#endif // CONFIG_H

