#include "Globals.h"

// ============================================================
// MT6701 SSI 编码器类
// 作用：以 24bit SSI 帧读取 MT6701，取前 14bit 角度
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
  uint32_t raw = readRaw24();
  uint16_t angle14 = (raw >> 10) & 0x3FFF;
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
    if (_calibrationUnwrap) {
      if (_unwrapHasSample) {
        _unwrapAngle += wrappedAngleDelta(candidate, _unwrapPrevious);
      } else {
        _unwrapAngle = candidate;
        _unwrapHasSample = true;
      }
      _unwrapPrevious = candidate;
    }
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

  // Keep the base fields coherent for diagnostics, while never accumulating
  // full_rotations in this variant.
  angle_prev = _angle;
  angle_prev_ts = now;
  full_rotations = 0;
}

float MT6701SSI::getMechanicalAngle() {
  return _angle;
}

float MT6701SSI::getAngle() {
  return _calibrationUnwrap ? _unwrapAngle : _angle;
}

double MT6701SSI::getPreciseAngle() {
  return (double)getAngle();
}

float MT6701SSI::getVelocity() {
  return _velocity;
}

int32_t MT6701SSI::getFullRotations() {
  return 0;
}

void MT6701SSI::setCalibrationUnwrap(bool enabled) {
  _calibrationUnwrap = enabled;
  _unwrapAngle = _angle;
  _unwrapPrevious = _angle;
  _unwrapHasSample = true;
}

uint32_t MT6701SSI::readRaw24() {
  SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

  _spiBus->beginTransaction(settings);
  digitalWrite(_csPin, LOW);

  uint8_t b0 = _spiBus->transfer(0x00);
  uint8_t b1 = _spiBus->transfer(0x00);
  uint8_t b2 = _spiBus->transfer(0x00);

  digitalWrite(_csPin, HIGH);
  _spiBus->endTransaction();

  return ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;
}

// ============================================================
// 编码器对象
// ============================================================

SPIClass spiBus(FSPI);

MT6701SSI sensorA(PIN_ENC_A_CS, &spiBus);
MT6701SSI sensorB(PIN_ENC_B_CS, &spiBus);
