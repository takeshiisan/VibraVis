// ALL VALUES IN THIS FILE ARE PLACEHOLDERS AND SHOULD BE ADJUSTED ACCORDING TO HARDWARE SETUP
#ifndef CONFIG_H 
#define CONFIG_H

// I2C 
// Shared I2C pins for both TCA9548A multiplexers
// Adjust these pins according to the actual ESP32-S3 wiring.
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// TCA9548A Multiplexer Addresses 
// 2 Multiplexers = 2 distinct I2C addresses
#define TCA9548A_1_ADDRESS 0x70 // First TCA9548A multiplexer address
#define TCA9548A_2_ADDRESS 0x71 // Second TCA9548A multiplexer address

// SENSOR-TO-MUX-CHANNEL MAPPING 
// Adjust according to physical mux channel of each sensor is wired to.
enum SensorPosition {
    SENSOR_LEFT_ARM = 0,
    SENSOR_RIGHT_ARM,
    SENSOR_BRIDGE,
    SENSOR_BOTTOM_LEFT,
    SENSOR_BOTTOM_RIGHT,
    SENSOR_COUNT
};

struct SensorMuxMapping {
    uint8_t muxAddress; // TCA9548A multiplexer address
    uint8_t channel;    // Channel on the multiplexer (0-7)
};

// EXAMPLE MAPPING LANG: ADJUST ACCORDING TO ACTUAL HARDWARE SETUP
static const struct SensorMuxMapping sensorMuxMappings[SENSOR_COUNT] = {
    {TCA9548A_1_ADDRESS, 0}, // SENSOR_LEFT_ARM
    {TCA9548A_1_ADDRESS, 1}, // SENSOR_RIGHT_ARM
    {TCA9548A_1_ADDRESS, 2}, // SENSOR_BRIDGE
    {TCA9548A_2_ADDRESS, 0}, // SENSOR_BOTTOM_LEFT
    {TCA9548A_2_ADDRESS, 1}  // SENSOR_BOTTOM_RIGHT
};

// VIBRATION MOTORS CONFIGURATION (DRV2605L) 
enum MotorZone {
    MOTOR_LEFT = 0,
    MOTOR_CENTER,
    MOTOR_RIGHT,
    MOTOR_COUNT
};

// NOTE: The DRV2605L's I2C address is fixed at 0x5A per its datasheet and is
// NOT user-configurable like the TCA9548A.
static const uint8_t motorDriverAddresses[MOTOR_COUNT] = {
    0x5A, // MOTOR_LEFT
    0x5B, // MOTOR_CENTER
    0x5C  // MOTOR_RIGHT
};

// Detection and timing consts 
#define OBSTACLE_DETECTION_THRESHOLD 100 // mm - placeholder, tune during testing
#define IMMEDIATE_DANGER_MM          300 // mm - hard safety threshold, always alerts
                                          // regardless of approach speed
#define DEBOUNCE_INTERVAL_MS         500 // min time between re-triggers per motor
#define SENSOR_POLL_INTERVAL_MS      100 // rate of poll on all sensors

// I2S AUDIO AMPLIFIER (MAX98357A)
#define I2S_BCLK 14
#define I2S_LRC  12
#define I2S_DOUT 13

#endif // CONFIG_H