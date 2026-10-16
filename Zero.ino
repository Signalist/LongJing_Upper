#include "Globals.h"

// ============================================================
// Flash / NVS 对象和零位运行变量
// ============================================================

Preferences zeroPrefs;

// RAM only: calibration/startup turns and previous boots never select a home turn.
static float runtimeHomeA = 0.0f;
static float runtimeHomeB = 0.0f;
static bool runtimeHomeAReady = false;
static bool runtimeHomeBReady = false;

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

  *homeOffset = zerohero::positive(
      (float)motor->sensor_direction * *homeOffset - motor->sensor_offset);
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
  if (!ensureShaftCoordinateOffset('A')) return 0.0f;
  sensorA.update();
  if (!runtimeHomeAReady) {
    runtimeHomeA = zerohero::nearestHome(motorA.shaftAngle(), homeAOffset);
    runtimeHomeAReady = true;
  }
  return motorA.shaftAngle() - runtimeHomeA;
}


float getRelativePosB() {
  if (!ensureShaftCoordinateOffset('B')) return 0.0f;
  sensorB.update();
  if (!runtimeHomeBReady) {
    runtimeHomeB = zerohero::nearestHome(motorB.shaftAngle(), homeBOffset);
    runtimeHomeBReady = true;
  }
  return motorB.shaftAngle() - runtimeHomeB;
}


float getRelativePosC() {
  return 0.0f; // C removed: never sample an uninitialized encoder.
}


float getAbsoluteTargetA(float relativeTarget) {
  if (!runtimeHomeAReady) getRelativePosA();
  return runtimeHomeA + relativeTarget;
}


float getAbsoluteTargetB(float relativeTarget) {
  if (!runtimeHomeBReady) getRelativePosB();
  return runtimeHomeB + relativeTarget;
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


String rebaseRuntimeTurn(char axis, int turns) {
  if (axis != 'A' && axis != 'B') return "ERR, C_AXIS_REMOVED";
  if (turns != 1 && turns != -1) return "ERR, REBASE_REQUIRES_PLUS_OR_MINUS_ONE";
  const bool isA = axis == 'A';
  if (!(isA ? sleepA : sleepB)) return "ERR, REBASE_REQUIRES_SLEEP";
  BLDCMotor& motor = isA ? motorA : motorB;
  if (motor.sensor_direction == Direction::UNKNOWN) return "ERR, ZERO_SENSOR_DIRECTION_UNKNOWN";
  const float position = isA ? getRelativePosA() : getRelativePosB();
  const float shifted = position + turns * _2PI;
  const float lower = isA ? aMinPos : B_MIN_POS;
  const float upper = isA ? aMaxPos : B_MAX_POS;
  if (shifted < lower || shifted > upper) return "ERR, REBASE_POSITION_OUT_OF_RANGE";
  float& runtimeHome = isA ? runtimeHomeA : runtimeHomeB;
  float& target = isA ? targetPosA : targetPosB;
  runtimeHome -= turns * _2PI;
  target = zerohero::boundedPosition(target + turns * _2PI, lower, upper);
  motor.target = runtimeHome + target;
  motor.PID_angle.reset();
  motor.PID_velocity.reset();
  // No flash write or motor enable: only this boot's coordinate branch changes.
  return String("OK, REBASE") + axis + ", position=" + String(shifted, 6);
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
    runtimeHomeA = motorA.shaftAngle();
    homeAOffset = zerohero::positive(runtimeHomeA);
    runtimeHomeAReady = true;
    zeroCoordMask |= ZERO_COORD_A;
    targetPosA = 0.0f;
    motorA.target = runtimeHomeA;
    motorA.PID_angle.reset();
    motorA.PID_velocity.reset();
  } else if (axis == 'B') {
    sensorB.update();
    runtimeHomeB = motorB.shaftAngle();
    homeBOffset = zerohero::positive(runtimeHomeB);
    runtimeHomeBReady = true;
    zeroCoordMask |= ZERO_COORD_B;
    targetPosB = 0.0f;
    motorB.target = runtimeHomeB;
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
    runtimeHomeA = motorA.shaftAngle();
    homeAOffset = zerohero::positive(runtimeHomeA);
    runtimeHomeAReady = true;
    sensorB.update();
    runtimeHomeB = motorB.shaftAngle();
    homeBOffset = zerohero::positive(runtimeHomeB);
    runtimeHomeBReady = true;
    targetPosA = 0.0f;
    targetPosB = 0.0f;
    targetPosC = 0.0f;

    motorA.target = runtimeHomeA;
    motorB.target = runtimeHomeB;

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
