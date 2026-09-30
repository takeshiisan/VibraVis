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
#include "driver/rtc_io.h"

Adafruit_VL53L7CX tofSensor;
Adafruit_DRV2605 motors[MOTOR_COUNT];
VL53L7CX_ResultsData results;
SFE_MAX1704X lipo;
Audio audio;

float batteryPercent = 100.0f; 
unsigned long lastBatteryCheck = 0;
unsigned long lastPollTime = 0;

unsigned long buttonPressStart = 0;
bool buttonHeld = false;
 
// Tracks previous distance/time per sensor, used to compute approach speed
uint16_t previousZoneDistance[ZONE_GROUP_COUNT]   = {0};
unsigned long previousZoneReadTime[ZONE_GROUP_COUNT] = {0};

//Track successfully initialized sensors, motors, battery guage, and audio system
bool sensorActive = {false};
bool motorActive[MOTOR_COUNT] = {false, false};
bool batteryGaugeActive = false;
bool spiffsReady = false;

bool mux2Present = false;

unsigned long lastStrobeToggle = 0;
bool strobeState = false;

unsigned long lightButtonLastClickTime = 0;
bool lightButtonPendingAction = false;

WhiteLedMode whiteLedMode = WHITE_LED_OFF; // starts led off
BatteryLevel lastBatteryLevel = BATTERY_LEVEL_HIGH; // Start assuming high battery till read cycle starts

// TwoWire Wire1 = TwoWire(1); 

// Initialize all 5 ToF sensors through their mux channels 
bool initSensor() {
  if (!tofSensor.begin(VL53L7CX_DEFAULT_ADDRESS, &Wire, 400000)) {
    Serial.println("Failed to init sensor");
    return false;
  }
  tofSensor.setResolution(64);
  tofSensor.setRangingFrequency(30);
  tofSensor.startRanging();
  sensorActive = true;
  Serial.println("Sensor initialized successfully.");
  return true;
}

// Initialize the 3 DRV2605L motor drivers 
bool initMotors() {
  bool allOk = true;

  if (!motors[MOTOR_LEFT].begin(&Wire)) { // shares bus 0 with the sensor - different address, no conflict
    Serial.println("Failed to init left motor");
    allOk = false;
  } else {
    motors[MOTOR_LEFT].selectLibrary(1);
    motors[MOTOR_LEFT].setMode(DRV2605_MODE_REALTIME);
    motors[MOTOR_LEFT].setRealtimeValue(0);
    motorActive[MOTOR_LEFT] = true;
    Serial.println("Left motor initialized successfully.");
  }

  if (!motors[MOTOR_RIGHT].begin(&Wire1)) { // its own separate bus
    Serial.println("Failed to init right motor");
    allOk = false;
  } else {
    motors[MOTOR_RIGHT].selectLibrary(1);
    motors[MOTOR_RIGHT].setMode(DRV2605_MODE_REALTIME);
    motors[MOTOR_RIGHT].setRealtimeValue(0);
    motorActive[MOTOR_RIGHT] = true;
    Serial.println("Right motor initialized successfully.");
  }

  return allOk;
}

// Read one sensor's minimum in-range distance
// Returns 0 if no valid reading or sensor not ready.
// Splits the sensor's 64 zones into LEFT/CENTER/RIGHT column groups,
// finding the closest valid reading within each group.
bool readZoneDistances(uint16_t distances[ZONE_GROUP_COUNT]) {
  if (!sensorActive || !tofSensor.isDataReady()) return false;

  VL53L7CX_ResultsData results;
  if (!tofSensor.getRangingData(&results)) return false;

  uint16_t leftMin = 65535, centerMin = 65535, rightMin = 65535;

  for (int zone = 0; zone < 64; zone++) {
    uint8_t status = results.target_status[zone];
    if (status != 5 && status != 9) continue;
    uint16_t d = results.distance_mm[zone];
    if (d == 0) continue;

    // ASSUMPTION: zones are row-major, column = zone % 8. VERIFY this
    // against your actual sensor mounting/orientation once hardware is
    // running - some mountings mirror or rotate this indexing.
    int col = zone % ZONE_GRID_SIZE;

    if (col <= ZONE_LEFT_MAX_COL) {
      if (d < leftMin) leftMin = d;
    } else if (col >= ZONE_RIGHT_MIN_COL) {
      if (d < rightMin) rightMin = d;
    } else {
      if (d < centerMin) centerMin = d;
    }
  }

  distances[ZONE_LEFT]   = (leftMin == 65535) ? 0 : leftMin;
  distances[ZONE_CENTER] = (centerMin == 65535) ? 0 : centerMin;
  distances[ZONE_RIGHT]  = (rightMin == 65535) ? 0 : rightMin;
  return true;
}


// Calculate approach speed (mm/s). Positive = approaching. 
float calculateApproachSpeed(int zoneIndex, uint16_t currentDistance) {
  unsigned long now = millis();
  float speed = 0;
  if (previousZoneDistance[zoneIndex] > 0 && previousZoneReadTime[zoneIndex] > 0) {
    float deltaTimeSec = (now - previousZoneReadTime[zoneIndex]) / 1000.0f;
    if (deltaTimeSec > 0) {
      speed = (previousZoneDistance[zoneIndex] - currentDistance) / deltaTimeSec;
    }
  }
  previousZoneDistance[zoneIndex] = currentDistance;
  previousZoneReadTime[zoneIndex] = now;
  return speed;
}

int selectPriorityZone(uint16_t distances[ZONE_GROUP_COUNT], float speeds[ZONE_GROUP_COUNT]) {
  int immediateIndex = -1, fastestIndex = -1;
  float fastestSpeed = 0;

  for (int i = 0; i < ZONE_GROUP_COUNT; i++) {
    if (distances[i] == 0) continue;
    if (distances[i] < IMMEDIATE_DANGER_MM) {
      if (immediateIndex == -1 || distances[i] < distances[immediateIndex]) immediateIndex = i;
    }
    if (speeds[i] > fastestSpeed) { fastestSpeed = speeds[i]; fastestIndex = i; }
  }

  if (immediateIndex != -1) return immediateIndex;
  return fastestIndex;
}

uint8_t zoneToMotorMask(int zoneIndex) {
  switch (zoneIndex) {
    case ZONE_LEFT:  return MASK_LEFT_MOTOR;
    case ZONE_RIGHT: return MASK_RIGHT_MOTOR;
    case ZONE_CENTER:
    default:         return MASK_BOTH_MOTORS;
  }
}
// THEORETICALLY, we could use the DRV2605L's RTP mode to set a continuous vibration intensity based on distance.
uint8_t distanceToAmplitude(uint16_t distanceMm) {
  if (distanceMm == 0 || distanceMm > OBSTACLE_DETECTION_THRESHOLD_MM) return 0;
  if (distanceMm < IMMEDIATE_DANGER_MM) return RTP_MAX_AMPLITUDE;
  float ratio = (float)(OBSTACLE_DETECTION_THRESHOLD_MM - distanceMm) /
                (OBSTACLE_DETECTION_THRESHOLD_MM - IMMEDIATE_DANGER_MM);
  return RTP_MIN_AMPLITUDE + (uint8_t)(ratio * (RTP_MAX_AMPLITUDE - RTP_MIN_AMPLITUDE));
}

void updateMotorIntensities(uint8_t motorMask, uint8_t amplitude) {
  if (motorActive[MOTOR_LEFT]) {
    motors[MOTOR_LEFT].setRealtimeValue((motorMask & MASK_LEFT_MOTOR) ? amplitude : 0);
  }
  if (motorActive[MOTOR_RIGHT]) {
    motors[MOTOR_RIGHT].setRealtimeValue((motorMask & MASK_RIGHT_MOTOR) ? amplitude : 0);
  }
}

void processObstacles() {
  uint16_t distances[ZONE_GROUP_COUNT] = {0, 0, 0};
  float speeds[ZONE_GROUP_COUNT] = {0, 0, 0};

  if (readZoneDistances(distances)) {
    for (int i = 0; i < ZONE_GROUP_COUNT; i++) {
      speeds[i] = (distances[i] > 0) ? calculateApproachSpeed(i, distances[i]) : 0;
    }
  }

  int priorityZone = selectPriorityZone(distances, speeds);
  if (priorityZone == -1) {
    updateMotorIntensities(0, 0);
    return;
  }

  uint8_t mask = zoneToMotorMask(priorityZone);
  uint8_t amplitude = distanceToAmplitude(distances[priorityZone]);
  updateMotorIntensities(mask, amplitude);

  Serial.printf("ALERT! Zone %d | Dist: %d mm | Amplitude: %d\n",
                priorityZone, distances[priorityZone], amplitude);
}

// White LED
void onLightButtonSingleClick() {
  if (whiteLedMode == WHITE_LED_SOLID) {
    whiteLedMode = WHITE_LED_STROBE;
  } else if (whiteLedMode == WHITE_LED_STROBE){
    whiteLedMode = WHITE_LED_SOLID;
  } else {
    whiteLedMode = WHITE_LED_SOLID;
  }
  Serial.printf("White LED mode -> %s\n", 
  whiteLedMode == WHITE_LED_SOLID ? "SOLID" : 
  whiteLedMode == WHITE_LED_STROBE ? "STROBE" : "OFF");
}

void onLightButtonDoubleClick() {
  whiteLedMode = WHITE_LED_OFF;
  Serial.printf("White LED mode -> OFF");
}

void updateWhiteLed () {
  switch (whiteLedMode) {
    case WHITE_LED_SOLID:
    digitalWrite(WHITE_LED_PIN, HIGH);
    break;
    case WHITE_LED_OFF:
    digitalWrite(WHITE_LED_PIN, LOW);
    break;
    case WHITE_LED_STROBE:
    if (millis() - lastStrobeToggle >= STROBE_INTERVAL_MS) {
      lastStrobeToggle = millis();
      strobeState = !strobeState;
      digitalWrite(WHITE_LED_PIN, strobeState ? HIGH : LOW);
    }
    break;
    }
  } 

void checkLightButton() {
  static bool lastRawState = HIGH;
  static bool debounceState = HIGH;
  static unsigned long lastDebounceTime = 0;

  bool currentRawState = digitalRead(LIGHT_BUTTON);

  if (currentRawState != lastRawState) {
    lastDebounceTime = millis(); // raw reading just changed - restart the debounce timer
  }
  lastRawState = currentRawState;  

  if (millis() - lastDebounceTime > BUTTON_DEBOUNCE_MS) {
    // raw reading has been stable long enough - accept it as real
    if (currentRawState != debounceState) {
      bool previousDebounceState = debounceState;
      debounceState = currentRawState;

      if (debounceState == LOW && previousDebounceState == HIGH) { // confirmed press edge
        unsigned long now = millis();
        if (now - lightButtonLastClickTime <= DOUBLE_CLICK_WINDOW_MS) {
          lightButtonPendingAction = false;
          onLightButtonDoubleClick();
        } else {
          lightButtonLastClickTime = now;
          lightButtonPendingAction = true;
        }
      }
    }
  }
  // Double-click window expired with no 2nd click -> resolve as single click
  if (lightButtonPendingAction && (millis() - lightButtonLastClickTime > DOUBLE_CLICK_WINDOW_MS)) {
  lightButtonPendingAction = false;
  onLightButtonSingleClick();
  }
} 
  
//Battery Reading
BatteryLevel getBatteryLevel(float batteryPercent) {
  if (batteryPercent >= BATTERY_HIGH_THRESHOLD) {
    return BATTERY_LEVEL_HIGH; // full charged
  } else if (batteryPercent >= BATTERY_MEDIUM_THRESHOLD) {
    return BATTERY_LEVEL_MEDIUM; // medium charged
  } else  if (batteryPercent >= BATTERY_LOW_THRESHOLD) {
    return BATTERY_LEVEL_LOW; // low charged
  } else {
    return BATTERY_LEVEL_CRITICAL; // critical low battery
  }
}
//uses annode so inverted
void setBatteryLED(BatteryLevel level) {
  switch(level) {
    case BATTERY_LEVEL_HIGH:
      digitalWrite(LED_RED_PIN, HIGH); // turn off red LED
      digitalWrite(LED_GREEN_PIN, LOW); // turn on green LED
      digitalWrite(LED_BLUE_PIN, HIGH); //  turn off blue LED
      break;
    case BATTERY_LEVEL_MEDIUM:
      digitalWrite(LED_RED_PIN, LOW); // turn on red LED
      digitalWrite(LED_GREEN_PIN, LOW); // turn on green LED
      digitalWrite(LED_BLUE_PIN, HIGH); // turn off blue LED
      break;
    case BATTERY_LEVEL_LOW:
      digitalWrite(LED_RED_PIN, LOW); // turn on red LED
      digitalWrite(LED_GREEN_PIN, HIGH); // turn off green LED
      digitalWrite(LED_BLUE_PIN, HIGH); // turn off blue LED
      break;
    case BATTERY_LEVEL_CRITICAL:
      digitalWrite(LED_RED_PIN, LOW); // turn on red LED
      digitalWrite(LED_GREEN_PIN, HIGH); // turn off green LED
      digitalWrite(LED_BLUE_PIN, LOW); // turn on blue LED for purple color
      break;
  }
}

// Initializes the MAX17043 fuel gauge and sets the low battery alert threshold.
bool initBatteryGauge() {
  Wire.beginTransmission(0x36); 
  if (Wire.endTransmission() != 0) {
    Serial.println("MAX17043 not detected. Check wiring.");
    return false;
  }
  if (!lipo.begin()) {
    Serial.println("MAX17043 not detected. Check wiring.");
    return false;
  }
  lipo.quickStart(); // Reset the fuel gauge to improve accuracy
  batteryGaugeActive = true;
  Serial.println("MAX17043 initialized successfully.");

  batteryPercent = lipo.getSOC();
  lastBatteryLevel = getBatteryLevel(batteryPercent);
  setBatteryLED(lastBatteryLevel);
  Serial.printf("Initial battery reading: %.1f%%\n", batteryPercent);

  return true;
}

void playBatteryAlert(BatteryLevel level) {
  if (!spiffsReady) return; 
  if (audio.isRunning()) return;

  switch(level) {
    case BATTERY_LEVEL_LOW:
      audio.connecttoFS(SPIFFS, "/low_battery_alert.wav");
      break;
    case BATTERY_LEVEL_MEDIUM:
      audio.connecttoFS(SPIFFS, "/medium_battery_alert.wav");
      break;
    case BATTERY_LEVEL_HIGH:
      audio.connecttoFS(SPIFFS, "/high_battery_alert.wav");
      break;
    case BATTERY_LEVEL_CRITICAL:
      audio.connecttoFS(SPIFFS, "/critical_battery_alert.wav");
      break;
  }

}

// Periodic reading of battery SOC
void checkBattery() {
   if (!batteryGaugeActive) return;

   unsigned long now = millis();
    if (now - lastBatteryCheck < BATTERY_CHECK_INTERVAL_MS) return; // not time yet
    lastBatteryCheck = now;

    batteryPercent = lipo.getSOC();
    BatteryLevel currentLevel = getBatteryLevel(batteryPercent);

    Serial.printf("Battery: %.1f%% | Voltage: %.2f V | Level: %s\n",
                batteryPercent, lipo.getVoltage(),
                currentLevel == BATTERY_LEVEL_HIGH ? "HIGH" :
                currentLevel == BATTERY_LEVEL_MEDIUM ? "MEDIUM" : 
                currentLevel == BATTERY_LEVEL_LOW ? "LOW" : "CRITICAL");
    
    setBatteryLED(currentLevel); // Update LED based on current battery level

    if (currentLevel != lastBatteryLevel) {
      playBatteryAlert(currentLevel);
      lastBatteryLevel = currentLevel;
    }
}

void enterDeepSleep() {
  Serial.println("Shutting Down...");
  updateMotorIntensities(0, 0);

  // Wait for button release so low signal doesn't instantly trigger ext0 wake
  while (digitalRead(POWER_BUTTON) == LOW) {
    delay(10);
  }
  delay(100);

  rtc_gpio_pullup_en((gpio_num_t)POWER_BUTTON);
  rtc_gpio_pulldown_dis((gpio_num_t)POWER_BUTTON);
  
  esp_err_t wakeResult = esp_sleep_enable_ext0_wakeup((gpio_num_t)POWER_BUTTON, 0);
  if (wakeResult != ESP_OK) {
    Serial.println("WARNING: failed to configure wake source - aborting sleep.");
    return;
  }
  esp_deep_sleep_start();

}

void checkPowerButton() {
  bool pressed = (digitalRead(POWER_BUTTON) == LOW);

  if (pressed && !buttonHeld) {
    buttonPressStart = millis(); // press just started
  }

  if (pressed && buttonHeld) {
    if (millis() - buttonPressStart >= LONG_PRESS_MS) {
      enterDeepSleep(); // held long enough - shut down
    }
  }

  buttonHeld = pressed;
}

void setup() {
  rtc_gpio_deinit((gpio_num_t)POWER_BUTTON); // release RTC IO hold from previous wake, restore normal digital GPIO mode

  Serial.begin(115200);
  delay(1000); // Allow time for Serial to initialize
  Serial.println("VibraVis starting...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  //LED pins for Battery reading
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(WHITE_LED_PIN, OUTPUT);
  pinMode(LIGHT_BUTTON, INPUT_PULLUP);
  setBatteryLED(BATTERY_LEVEL_HIGH); // Start with green LED
  //Button
  pinMode(POWER_BUTTON, INPUT_PULLUP);
  whiteLedMode = WHITE_LED_SOLID;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire1.begin(I2C_SDA_PIN2, I2C_SCL_PIN2);
 
  Serial.println("VibraVis booting...");  
 
  if (!initSensor()) {
    Serial.println("WARNING: one or more sensors failed to init.");
  }
  if (!initMotors()) {
    Serial.println("WARNING: one or more motors failed to init.");
  }
  if (!initBatteryGauge()) {
    Serial.println("WARNING: Battery gauge failed to init.");
  }
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  } else {
    spiffsReady = true;
    Serial.println("SPIFFS mounted successfully.");
  }
  Serial.println("Files in SPIFFS:");
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.printf("  %s (%d bytes)\n", file.name(), file.size());
    file = root.openNextFile();
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
    // Serial.println("VibraVis is alive.");
    // delay(5000); // small delay to avoid flooding the serial output
  }
  checkPowerButton();
  checkBattery();
  checkLightButton();
  updateWhiteLed();
  audio.loop(); // process audio playback
}
