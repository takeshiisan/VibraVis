/* 
VibraVis: Specialized Haptic Feedback Eyeglasses
--------------------------------------------------



*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <Adafruit_VL53L7CX.h>
#include "Audio.h"
#include "config.h"

Adafruit_VL53L7CX vl53l7cx;
VL53L7CX_ResultsData results;

Adafruuit_DRV2605 motors[MOTOR_COUNT];
Audio audio;

unsigned long lastMotorTrigger[MOTOR_COUNT] = {0};
unsigned long lastPollTime = 0;
 
// Tracks previous distance/time per sensor, used to compute approach speed
uint16_t previousDistance[SENSOR_COUNT]   = {0};
unsigned long previousReadTime[SENSOR_COUNT] = {0};

// Multiplexer channel select 
void selectMuxChannel(uint8_t muxAddress, uint8_t channel) {
  Wire.beginTransmission(muxAddress);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Initialize all 5 ToF sensors through their mux channels 
bool initSensors() {
  bool allOk = true;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    selectMuxChannel(sensorMuxMappings[i].muxAddress, sensorMuxMappings[i].channel);
    delay(5);

    if (!vl53l7cx.begin(VL53L7CX_DEFAULT_ADDRESS, &Wire, 400000)) {
      Serial.print(F("Failed to initialize VL53L7CX sensor at index "));
      Serial.println(i);
      allOk = false;
    }
    // Set 8x8 resolution (64 zones)
    if (!vl53l7cx.setResolution(64)) {
      Serial.print(F("Failed to set resolution for sensor at index "));
      Serial.println(i);
      allOk = false;
    } if (!vl53l7cx.setRangingFrequency(30)) {
      Serial.print(F("Failed to set ranging frequency for sensor at index "));
      Serial.println(i);
      allOk = false;

        vl53l7cx.stopRanging();

    } if (!vL53L7cx.initMotionIndicator(64)) {
      Serial.print(F("Failed to init motion indicator for sensor at index "));
      Serial.println(i);
      allOk = false;
    } if (!vl53l7cx.setMotionDistance(400, 3000)) {
      Serial.print(F("Failed to set motion distance for sensor at index "));
      Serial.println(i);
      allOk = false;
    } if (!vl53l7cx.startRanging()) {
      Serial.print(F("Failed to start ranging for sensor at index "));
      Serial.println(i);
      allOk = false;
      continue;
    } else {
      Serial.print(F("Sensor at index "));
      Serial.print(i);
      Serial.println(F(" initialized successfully."));
    }
  }
  
  }
  return allOk;
}
// Initialize the 3 DRV2605L motor drivers 
void initMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // NOTE: see config.h TODO - DRV2605L address is fixed at 0x5A per
    // datasheet, these three chips likely need mux routing rather than
    // distinct I2C addresses. Update this init loop once confirmed.
    motors[i].begin();
    motors[i].selectLibrary(1);
    motors[i].setMode(DRV2605_MODE_INTTRIG); // internal trigger, waveform playback
  }
}

// Read one sensor's minimum in-range distance
// Returns 0 if no valid reading or sensor not ready.
uint16_t readSensorMinDistance(int sensorIndex) {
  selectMuxChannel(sensorMuxMappings[sensorIndex].muxAddress,
                    sensorMuxMappings[sensorIndex].channel);
 
  // TODO: replace with real Adafruit_VL53L7 data-ready check + read call.
  // Placeholder returns 0 (no reading) until the real API is wired in.
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