// ALL VALUES IN THIS FILE ARE PLACEHOLDERS AND SHOULD BE ADJUSTED ACCORDING TO HARDWARE SETUP
#ifndef CONFIG_H 
#define CONFIG_H

// ESP32-S3 Default I2C Pins (can be changed if needed)
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// TCA9548A Multiplexer Addresses 
#define TCA9548A_1_ADDRESS 0x70 
#define TCA9548A_2_ADDRESS 0x71 

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

struct DeviceMuxMapping {
    uint8_t muxAddress; // TCA9548A multiplexer address
    uint8_t channel;    // Channel on the multiplexer (0-7)
};

// EXAMPLE MAPPING LANG: ADJUST ACCORDING TO ACTUAL HARDWARE SETUP
static const struct DeviceMuxMapping sensorMuxMappings[SENSOR_COUNT] = {
    {TCA9548A_1_ADDRESS, 0}, // SENSOR_LEFT_ARM
    {TCA9548A_1_ADDRESS, 1}, // SENSOR_RIGHT_ARM
    {TCA9548A_1_ADDRESS, 2}, // SENSOR_BRIDGE
    {TCA9548A_2_ADDRESS, 0}, // SENSOR_BOTTOM_LEFT
    {TCA9548A_2_ADDRESS, 1}  // SENSOR_BOTTOM_RIGHT
};

// VIBRATION MOTORS MAPPING (DRV2605L) 
enum MotorZone {
    MOTOR_LEFT = 0,
    MOTOR_CENTER,
    MOTOR_RIGHT,
    MOTOR_COUNT
};

// NOTE: The DRV2605L's I2C address is fixed at 0x5A per its datasheet and is NOT user-configurable like the TCA9548A.
#define DRV2605L_ADDRESS 0x5A

static const struct DeviceMuxMapping motorMuxMappings[MOTOR_COUNT] = {
    {TCA9548A_2_ADDRESS, 3}, // MOTOR_LEFT
    {TCA9548A_2_ADDRESS, 4}, // MOTOR_CENTER
    {TCA9548A_2_ADDRESS, 5}  // MOTOR_RIGHT
};

// Bitmasks for triggering multiple motors simultaneously (if needed)
#define MASK_LEFT   (1 << MOTOR_LEFT)
#define MASK_CENTER (1 << MOTOR_CENTER)
#define MASK_RIGHT (1 << MOTOR_RIGHT)

// Thresholds and timings 
#define OBSTACLE_DETECTION_THRESHOLD_MM 1500 // mm
#define IMMEDIATE_DANGER_MM 300 // mm
#define DEBOUNCE_INTERVAL_MS 500 // ms
#define SENSOR_POLL_INTERVAL_MS 100 // ms

// I2S AUDIO AMPLIFIER (MAX98357A)
#define I2S_BCLK 14
#define I2S_LRC  12
#define I2S_DOUT 13

//RTP 
#define RTP_MAX_AMPLITUDE 127 
#define RTP_MIN_AMPLITUDE 40
#endif // CONFIG_H