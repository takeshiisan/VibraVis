/* 
* VibraVis: Specialized Haptic Feedback Eyeglasses
* ****************************************************
* Written by: Nathan "takeshiisan" Tan with assistance from Claude Code
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <Adafruit_VL53L7CX.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <SPIFFS.h>
#include "Audio.h"
#include "config.h"

Adafruit_VL53L7CX tofsensors[SENSOR_COUNT];
Adafruit_DRV2605 motors[MOTOR_COUNT];
VL53L7CX_ResultsData results;
SFE_MAX1704X lipo;
Audio audio;

float batteryPercent = 100.0f; 
void playLowBatteryAlert();
bool lowBatteryAlertPlayed = false;
bool lowBatteryAlert = false; 
unsigned long lastBatteryCheck = 0;

unsigned long lastMotorTrigger = 0;
unsigned long lastPollTime = 0;
 
// Tracks previous distance/time per sensor, used to compute approach speed
uint16_t previousDistance[SENSOR_COUNT]   = {0};
unsigned long previousReadTime[SENSOR_COUNT] = {0};

//Track successfully initialized sensors, motors, battery guage, and audio system
bool sensorActive[SENSOR_COUNT] = {false};
bool motorActive[MOTOR_COUNT] = {false};
bool batteryGaugeActive = false;
bool spiffsReady = false;

// Multiplexer channel select 
void selectMuxChannel(uint8_t muxAddress, uint8_t channel) {
  Wire.beginTransmission(TCA9548A_1_ADDRESS); 
  Wire.write(0x00); // disable all channels
  Wire.endTransmission();

  Wire.beginTransmission(TCA9548A_2_ADDRESS);
  Wire.write(0x00); // disable all channels
  Wire.endTransmission();

  Wire.beginTransmission(muxAddress); 
  Wire.write(1 << channel); // enable the desired channel
  Wire.endTransmission();

  Serial.printf("[MUX] Active -> addr: 0x%02X, channel: %d\n", muxAddress, channel);
}

// Initialize all 5 ToF sensors through their mux channels 
bool initSensors() {
  bool allOk = true;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    selectMuxChannel(sensorMuxMappings[i].muxAddress, sensorMuxMappings[i].channel);
    delay(5);

    if (!tofsensors[i].begin(VL53L7CX_DEFAULT_ADDRESS, &Wire, 400000)) {
      Serial.printf("Failed to init sensor %d\n", i);
      Serial.flush(); 
      allOk = false;
      continue;
    }
    tofsensors[i].setResolution(64); // 8x8 resolution
    tofsensors[i].setRangingFrequency(30); // 30HZ
    tofsensors[i].startRanging();

    sensorActive[i] = true;

    Serial.printf("Sensor %d initialized successfully.\n", i);
    Serial.flush();
  }
  return allOk;
}

// Initialize the 3 DRV2605L motor drivers 
bool initMotors() {
  bool allOk = true;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    selectMuxChannel(motorMuxMappings[i].muxAddress, motorMuxMappings[i].channel);
    delay(5);

    if(motors[i].begin()) {
      motors[i].selectLibrary(1); // select ERM library
      motors[i].setMode(DRV2605_MODE_REALTIME); // RTP MODE
      motors[i].setRealtimeValue(0); // start silent
      motorActive[i] = true;
      Serial.printf("Motor %d initialized successfully.\n", i);
    } else {
      Serial.printf("Failed to init motor %d\n", i);
      allOk = false;
    }
  }
  return allOk;
}

// Read one sensor's minimum in-range distance
// Returns 0 if no valid reading or sensor not ready.
uint16_t readSensorMinDistance(int sensorIndex) {
  // CRITICAL: Skip this sensor if it failed to initialize
  if (!sensorActive[sensorIndex]) {
      return 0; 
  }
  selectMuxChannel(sensorMuxMappings[sensorIndex].muxAddress, sensorMuxMappings[sensorIndex].channel);
  
  if (tofsensors[sensorIndex].isDataReady()) {   
   if(!tofsensors[sensorIndex].getRangingData(&results)) {
     Serial.printf("Failed to get ranging data from sensor %d\n", sensorIndex);
     return 0;
   }
   
   uint16_t minDistance = 65535; // max uint16_t
   // Scan all 64 zones for closest target
  for(int j = 0; j < 64; j++) {
    //Status indicating the measurement validity (5 & 9 means ranging OK
      if(results.target_status[j] == 5 || results.target_status[j] == 9) {
        if(results.distance_mm[j] > 0 && results.distance_mm[j] < minDistance) {
          minDistance = results.distance_mm[j];
        }
      }
    }
  return (minDistance == 65535) ? 0 : minDistance;
  }
  return 0;
}

// Calculate approach speed (mm/s). Positive = approaching. 
float calculateApproachSpeed(int sensorIndex, uint16_t currentDistance) {
  unsigned long now = millis();
  float speed = 0;
 
  if (previousDistance[sensorIndex] > 0 && previousReadTime[sensorIndex] > 0) {
    float deltaTimeSec = (now - previousReadTime[sensorIndex]) / 1000.0f;
    if (deltaTimeSec > 0) {
      speed = (previousDistance[sensorIndex] - currentDistance) / deltaTimeSec;
    }
  }
  previousDistance[sensorIndex] = currentDistance;
  previousReadTime[sensorIndex] = now;
  return speed;
}

// ---------- Select ONE priority obstacle per cycle ----------
// Rule: any obstacle within IMMEDIATE_DANGER_MM always wins (nearest of
// those, if multiple). Otherwise, the fastest-approaching obstacle wins.
// Returns sensor index, or -1 if nothing needs an alert this cycle.
int selectPriorityObstacle(uint16_t distances[SENSOR_COUNT], float speeds[SENSOR_COUNT]) {
  int immediateIndex = -1;
  int fastestIndex = -1;
  float fastestSpeed = 0;
 
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (distances[i] == 0) continue; // no valid reading this cycle
 
    if (distances[i] < IMMEDIATE_DANGER_MM) {
      if (immediateIndex == -1 || distances[i] < distances[immediateIndex]) {
        immediateIndex = i;
      }
    }
 
    if (speeds[i] > fastestSpeed) {
      fastestSpeed = speeds[i];
      fastestIndex = i;
    }
  }

  if (immediateIndex != -1) return immediateIndex; // safety threshold overrides
  return fastestIndex;                              // otherwise, fastest wins
}
 
// Returns a bitmask so multiple motors can be triggered simultaneously if needed.
uint8_t sensorToMotorMask(int sensorIndex) {
  switch(sensorIndex) {
    case SENSOR_LEFT_ARM: return MASK_LEFT;
    case SENSOR_RIGHT_ARM: return MASK_RIGHT;
    case SENSOR_BOTTOM_LEFT: return MASK_LEFT | MASK_CENTER; // both left and center motors
    case SENSOR_BOTTOM_RIGHT: return MASK_RIGHT | MASK_CENTER; // both right and center motors
    case SENSOR_BRIDGE: default: return MASK_CENTER; // bridge sensor triggers center motor
  }
}

/*
EFFECT ID MAPPING: For simplicity, we can use the same effect ID for all motors. 
(WILL NOT BE USED FOR NOW).
*/

// void triggerMotor(uint8_t motorMask, uint8_t effectId) {
//   for(int i = 0; i < MOTOR_COUNT; i++) {
//     if(motorMask & (1 << i)) {
//       selectMuxChannel(motorMuxMappings[i].muxAddress, motorMuxMappings[i].channel);
//       motors[i].setWaveform(0, effectId); // set effect
//       motors[i].setWaveform(1, 0); // end of sequence
//       motors[i].go();
//     }
//   }
// }


// THEORETICALLY, we could use the DRV2605L's RTP mode to set a continuous vibration intensity based on distance.

uint8_t distanceToAmplitude(uint16_t distanceMm) {
  if(distanceMm == 0 || distanceMm > OBSTACLE_DETECTION_THRESHOLD_MM) {
    return 0; // nothing in range = no vibration
  }
  if (distanceMm < IMMEDIATE_DANGER_MM) {
    return RTP_MAX_AMPLITUDE; // immediate danger = max vibration
  }
  float ration = (float)(OBSTACLE_DETECTION_THRESHOLD_MM - distanceMm) / (OBSTACLE_DETECTION_THRESHOLD_MM - IMMEDIATE_DANGER_MM);
  return RTP_MIN_AMPLITUDE + (uint8_t)(ration * (RTP_MAX_AMPLITUDE - RTP_MIN_AMPLITUDE));
}

void updateMotorIntensities(uint8_t motorMask, uint8_t amplitude) {
  for(int i = 0; i < MOTOR_COUNT; i++) {
      if(!motorActive[i]) continue; // Skip if motor failed to initialize
      selectMuxChannel(motorMuxMappings[i].muxAddress, motorMuxMappings[i].channel);
      motors[i].setRealtimeValue((motorMask & (1 << i)) ? amplitude : 0); // set amplitude or silence
  }
}

void processObstacles() {
  uint16_t distances[SENSOR_COUNT] = {0};
  float speeds[SENSOR_COUNT] = {0};

  // Gather all sensor data
  for (int i = 0; i < SENSOR_COUNT; i++) {
    distances[i] = readSensorMinDistance(i);
    speeds[i] = (distances[i] > 0) ? calculateApproachSpeed(i, distances[i]) : 0;
  }

  // Decide priority obstacle
  int priorityIndex = selectPriorityObstacle(distances, speeds);

  if (priorityIndex == -1) {
    updateMotorIntensities(0,0);
    return; // no alert needed
  }

  uint8_t mask = sensorToMotorMask(priorityIndex);
  uint8_t amplitude = distanceToAmplitude(distances[priorityIndex]);
  updateMotorIntensities(mask, amplitude);

  Serial.printf("ALERT! Sensor %d | Dist: %d mm | Speed: %.1f mm/s | Amplitude: %d\n", priorityIndex, distances[priorityIndex], speeds[priorityIndex], amplitude);
    // triggerMotor(mask, effect);
    // lastMotorTrigger = millis();
    // Serial.printf("ALERT! Sensor %d | Dist: %d mm | Speed: %.1f mm/s\n", priorityIndex, distances[priorityIndex], speeds[priorityIndex]);
  //}
}

// Initializes the MAX17043 fuel gauge and sets the low battery alert threshold.

bool initBatteryGuage() {
  if (!lipo.begin()) {
    Serial.println("MAX17043 not detected. Check wiring.");
    return false;
  }
  lipo.quickStart(); // Reset the fuel gauge to improve accuracy
  lipo.setThreshold(LOW_BATTERY_PERCENT); // Set low battery alert threshold
  batteryGaugeActive = true;
  Serial.println("MAX17043 initialized successfully.");
  return true;
}

// Periodic reading of battery SOC
void checkBattery() {
  if (!batteryGaugeActive) return; // Skip if battery gauge failed to initialize

  unsigned long now = millis();
  if (now - lastBatteryCheck < BATTERY_CHECK_INTERVAL_MS) return;
  lastBatteryCheck = now; 

  batteryPercent = lipo.getSOC();
  lowBatteryAlert = (batteryPercent <= LOW_BATTERY_PERCENT);
  Serial.printf("Battery: %.1f%% | Voltage: %.2f V | Low Battery Alert: %s\n", batteryPercent, lipo.getVoltage(), lowBatteryAlert ? "YES" : "NO");

  if (lowBatteryAlert && !lowBatteryAlertPlayed) {
    playLowBatteryAlert();
    lowBatteryAlertPlayed = true;
  } else if (!lowBatteryAlert) {
    lowBatteryAlertPlayed = false; // reset if battery is no longer low
  }
}

void playLowBatteryAlert() {
  if (!spiffsReady) {
    Serial.println("SPIFFS not ready. Cannot play low battery alert.");
    return;
  }
  if (audio.isRunning()) return; // Don't interrupt if audio is already playing
  audio.connecttoFS(SPIFFS, "/low_battery_alert.wav"); // Ensure this file exists in SPI

}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
 
  Serial.println("VibraVis booting...");
 
  if (!initSensors()) {
    Serial.println("WARNING: one or more sensors failed to init.");
  }
  if (!initMotors()) {
    Serial.println("WARNING: one or more motors failed to init.");
  }
  if (!initBatteryGuage()) {
    Serial.println("WARNING: Battery gauge failed to init.");
  }
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  } else {
    spiffsReady = true;
    Serial.println("SPIFFS mounted successfully.");
  }
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(15); //  0-21 scale, unrelated to the 0-127 RTP haptic scale.
 
  Serial.println("VibraVis ready.");
}
 
void loop() {
  unsigned long now = millis();
  if (now - lastPollTime >= SENSOR_POLL_INTERVAL_MS) {
    lastPollTime = now;
    processObstacles();
  }
  checkBattery();
  audio.loop(); // process audio playback
}