#include "Globals.h"

// ============================================================
// MT6701 SSI 编码器类
// Official SimpleFOC MT6701 SSI framing: MODE2, 16 bits, discard leading bit.
// ============================================================

MT6701SSI::MT6701SSI(uint8_t csPin, SPIClass* spiBus) {
  _csPin = csPin;
  _spiBus = spiBus;
  _angle = 0.0f;
  _velocity = 0.0f;
  _velocityAngle = 0.0f;
  _velocityTimestamp = 0;
  _hasSample = false;
  _hasVelocitySample = false;
  _calibrationUnwrap = false;
  _unwrapHasSample = false;
  _unwrapAngle = 0.0f;
  _unwrapPrevious = 0.0f;
}

void MT6701SSI::init() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  this->Sensor::init();
  _angle = angle_prev;
  _velocity = 0.0f;
  _velocityAngle = _angle;
  _velocityTimestamp = micros();
  _hasSample = true;
  _hasVelocitySample = true;
  _unwrapAngle = _angle;
  _unwrapPrevious = _angle;
  _unwrapHasSample = true;
  full_rotations = 0;
  vel_full_rotations = 0;
}

float MT6701SSI::getSensorAngle() {
  uint16_t angle14 = zerohero::officialAngle((uint16_t)readRaw24());
  return ((float)angle14 / 16384.0f) * _2PI;
}

static float wrappedAngleDelta(float current, float previous) {
  float delta = current - previous;
  if (delta > 0.5f * _2PI) {
    delta -= _2PI;
  } else if (delta < -0.5f * _2PI) {
    delta += _2PI;
  }
  return delta;
}

void MT6701SSI::update() {
  float candidate = getSensorAngle();
  if (candidate < 0.0f) {
    return;
  }

  uint32_t now = micros();
  if (!_hasSample) {
    _angle = candidate;
    _velocityAngle = candidate;
    _velocityTimestamp = now;
    _hasSample = true;
    _hasVelocitySample = true;
    _velocity = 0.0f;
    _unwrapAngle = candidate;
    _unwrapPrevious = candidate;
    _unwrapHasSample = true;
  } else {
    uint32_t elapsed = now - _velocityTimestamp;
    if (_hasVelocitySample &&
        elapsed >= (uint32_t)(min_elapsed_time * 1000000.0f)) {
      _velocity = wrappedAngleDelta(candidate, _velocityAngle) /
                  ((float)elapsed * 1.0e-6f);
      _velocityAngle = candidate;
      _velocityTimestamp = now;
    }
    _angle = candidate;
  }

  // SimpleFOC Sensor pattern: separate integer revolutions and single-turn angle.
  full_rotations = zerohero::rotationsAfterSample(angle_prev, _angle, full_rotations);
  angle_prev = _angle;
  angle_prev_ts = now;
}

float MT6701SSI::getMechanicalAngle() {
  return _angle;
}

float MT6701SSI::getAngle() {
  return (float)full_rotations * _2PI + _angle;
}

double MT6701SSI::getPreciseAngle() {
  return (double)full_rotations * (double)_2PI + (double)_angle;
}

float MT6701SSI::getVelocity() {
  return _velocity;
}

int32_t MT6701SSI::getFullRotations() {
  return full_rotations;
}

void MT6701SSI::setCalibrationUnwrap(bool enabled) {
  _calibrationUnwrap = enabled;
  // Do not reset the continuous coordinate at the calibration/runtime boundary.
}

uint32_t MT6701SSI::readRaw24() {
  SPISettings settings(1000000, MSBFIRST, SPI_MODE2);

  _spiBus->beginTransaction(settings);
  digitalWrite(_csPin, LOW);

  uint16_t word = _spiBus->transfer16(0x0000);

  digitalWrite(_csPin, HIGH);
  _spiBus->endTransaction();

  return word; // Compatibility name readRaw24; actual transaction is 16 bits.
}

// ============================================================
// 编码器对象
// ============================================================

SPIClass spiBus(FSPI);

MT6701SSI sensorA(PIN_ENC_A_CS, &spiBus);
MT6701SSI sensorB(PIN_ENC_B_CS, &spiBus);
MT6701SSI sensorC(PIN_ENC_C_CS, &spiBus);
