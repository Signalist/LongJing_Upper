#include "Globals.h"

// ============================================================
// Flash / NVS 对象和零位运行变量
// ============================================================

Preferences zeroPrefs;

static float normalizeSingleTurnPosition(float position) {
  return zerohero::positive(position);
}

float homeAOffset = 0.0f;
float homeBOffset = 0.0f;
float homeCOffset = 0.0f;

// Old data stored the raw sensor angle. Each bit records whether that axis has
// been converted to SimpleFOC shaft coordinates:
//   shaft_angle = sensor_direction * sensor_angle - sensor_offset
static const uint8_t ZERO_COORD_A = 0x01;
static const uint8_t ZERO_COORD_B = 0x02;
static const uint8_t ZERO_COORD_C = 0x04;
static const uint8_t ZERO_COORD_ALL =
    ZERO_COORD_A | ZERO_COORD_B | ZERO_COORD_C;
static uint8_t zeroCoordMask = 0;

void loadZeroOffsetsFromFlash() {
  zeroPrefs.begin("zh_ssi", false);

  homeAOffset = zeroPrefs.getFloat("A_offset", 0.0f);
  homeBOffset = zeroPrefs.getFloat("B_offset", 0.0f);
  homeCOffset = zeroPrefs.getFloat("C_offset", 0.0f);
  uint8_t oldVersion = zeroPrefs.getUChar("coord_ver", 0);
  zeroCoordMask = zeroPrefs.getUChar(
      "coord_mask",
      oldVersion == 1 ? ZERO_COORD_ALL : 0);
}


void saveZeroOffsetsToFlash() {
  zeroPrefs.putFloat("A_offset", homeAOffset);
  zeroPrefs.putFloat("B_offset", homeBOffset);
  zeroPrefs.putFloat("C_offset", homeCOffset);
  zeroPrefs.putUChar("coord_mask", zeroCoordMask);
  // Keep the old all-or-nothing key for compatibility with the first fix.
  zeroPrefs.putUChar(
      "coord_ver",
      zeroCoordMask == ZERO_COORD_ALL ? 1 : 0);
}


static bool ensureShaftCoordinateOffset(char axis) {
  uint8_t axisBit = 0;
  BLDCMotor* motor = nullptr;
  float* homeOffset = nullptr;

  if (axis == 'A') {
    axisBit = ZERO_COORD_A;
    motor = &motorA;
    homeOffset = &homeAOffset;
  } else if (axis == 'B') {
    axisBit = ZERO_COORD_B;
    motor = &motorB;
    homeOffset = &homeBOffset;
  } else if (axis == 'C') {
    axisBit = ZERO_COORD_C;
    motor = &motorC;
    homeOffset = &homeCOffset;
  } else {
    return false;
  }

  if (zeroCoordMask & axisBit) {
    return true;
  }

  // initFOC() determines each axis direction independently.
  if (motor->sensor_direction == Direction::UNKNOWN) {
    return false;
  }

  *homeOffset =
      (float)motor->sensor_direction * *homeOffset - motor->sensor_offset;
  zeroCoordMask |= axisBit;
  saveZeroOffsetsToFlash();

  Serial.print("Zero offset migrated to SimpleFOC shaft coordinates: ");
  Serial.println(axis);
  return true;
}


void printZeroOffsetsAtBoot() {
  Serial.println("Zero offsets loaded from Flash:");

  Serial.print("A_offset=");
  Serial.println(homeAOffset, 6);

  Serial.print("B_offset=");
  Serial.println(homeBOffset, 6);

  Serial.print("C_offset=");
  Serial.println(homeCOffset, 6);
}


float getRelativePosA() {
  ensureShaftCoordinateOffset('A');
  sensorA.update();
  return normalizeSingleTurnPosition(motorA.shaftAngle() - homeAOffset);
}


float getRelativePosB() {
  ensureShaftCoordinateOffset('B');
  sensorB.update();
  return normalizeSingleTurnPosition(motorB.shaftAngle() - homeBOffset);
}


float getRelativePosC() {
  return 0.0f; // C removed: never sample an uninitialized encoder.
}


float getAbsoluteTargetA(float relativeTarget) {
  ensureShaftCoordinateOffset('A');
  return homeAOffset + relativeTarget;
}


float getAbsoluteTargetB(float relativeTarget) {
  ensureShaftCoordinateOffset('B');
  return homeBOffset + relativeTarget;
}


float getAbsoluteTargetC(float relativeTarget) {
  ensureShaftCoordinateOffset('C');
  return homeCOffset + relativeTarget;
}


String getCalibrationInfo() {
  String msg = "";

  msg += "A_offset=";
  msg += String(homeAOffset, 6);
  msg += ", B_offset=";
  msg += String(homeBOffset, 6);

  return msg;
}


String setCurrentPositionAsZero(char axis) {
  if (axis == 'C') return "ERR, C_AXIS_REMOVED";
  if ((axis == 'A' && motorA.sensor_direction == Direction::UNKNOWN) ||
      (axis == 'B' && motorB.sensor_direction == Direction::UNKNOWN) ||
      (axis == 'C' && motorC.sensor_direction == Direction::UNKNOWN) ||
      (axis != 'A' && axis != 'B' && axis != 'C' &&
       (motorA.sensor_direction == Direction::UNKNOWN ||
        motorB.sensor_direction == Direction::UNKNOWN))) {
    return "ERR, ZERO_SENSOR_DIRECTION_UNKNOWN";
  }

  if (axis == 'A') {
    sensorA.update();
    homeAOffset = motorA.shaftAngle();
    zeroCoordMask |= ZERO_COORD_A;
    targetPosA = 0.0f;
    motorA.target = homeAOffset;
    motorA.PID_angle.reset();
    motorA.PID_velocity.reset();
  } else if (axis == 'B') {
    sensorB.update();
    homeBOffset = motorB.shaftAngle();
    zeroCoordMask |= ZERO_COORD_B;
    targetPosB = 0.0f;
    motorB.target = homeBOffset;
    motorB.PID_angle.reset();
    motorB.PID_velocity.reset();
  } else if (axis == 'C') {
    sensorC.update();
    homeCOffset = motorC.shaftAngle();
    zeroCoordMask |= ZERO_COORD_C;
    targetPosC = 0.0f;
    motorC.target = homeCOffset;
    motorC.PID_angle.reset();
    motorC.PID_velocity.reset();
  } else {
    sensorA.update();
    homeAOffset = motorA.shaftAngle();
    sensorB.update();
    homeBOffset = motorB.shaftAngle();
    targetPosA = 0.0f;
    targetPosB = 0.0f;
    targetPosC = 0.0f;

    motorA.target = homeAOffset;
    motorB.target = homeBOffset;

    motorA.PID_angle.reset();
    motorA.PID_velocity.reset();
    motorB.PID_angle.reset();
    motorB.PID_velocity.reset();
    zeroCoordMask |= ZERO_COORD_A | ZERO_COORD_B;
  }

  saveZeroOffsetsToFlash();

  String msg = "OK, ";
  msg += getCalibrationInfo();
  return msg;
}
