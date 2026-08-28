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