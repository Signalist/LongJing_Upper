#include "Globals.h"

// ============================================================
// 电机对象定义
// ============================================================

BLDCDriver3PWM driverA(PIN_A_U, PIN_A_V, PIN_A_W, PIN_A_SLEEP);
BLDCDriver3PWM driverB(PIN_B_U, PIN_B_V, PIN_B_W, PIN_B_SLEEP);

BLDCMotor motorA(MOTOR_A_POLE_PAIRS, MOTOR_A_PHASE_RESISTANCE, MOTOR_A_KV_RATING, MOTOR_A_LQ, MOTOR_A_LD);
BLDCMotor motorB(MOTOR_B_POLE_PAIRS, MOTOR_B_PHASE_RESISTANCE, MOTOR_B_KV_RATING, MOTOR_B_LQ, MOTOR_B_LD);

InlineCurrentSense currentSenseA(CURRENT_SHUNT_RESISTANCE, CURRENT_SENSE_GAIN,
                                 PIN_A_CURRENT_U, PIN_A_CURRENT_V);
InlineCurrentSense currentSenseB(CURRENT_SHUNT_RESISTANCE, CURRENT_SENSE_GAIN,
                                 PIN_B_CURRENT_U, PIN_B_CURRENT_V);

// 当前 FOC 最大输出电压，运行时可由 VF 指令修改，断电后恢复默认值
float Vfoc = FOC_VOLTAGE_LIMIT_DEFAULT;
float Ifoc = FOC_CURRENT_LIMIT_DEFAULT;

// PID gains are persisted in ESP32 NVS. The values in setupMotor() are used
// only when no saved value exists yet.
Preferences pidPrefs;
static bool pidPrefsReady = false;

// ============================================================
// 电机目标和模式变量
// ============================================================

uint8_t modeA = AXIS_MODE_VELOCITY;
uint8_t modeB = AXIS_MODE_VELOCITY;

float targetVelA = 0.0f;
float targetVelB = 0.0f;

float targetPosA = 0.0f;
float targetPosB = 0.0f;

// FOC current torque mode q-axis current commands, unit: A
float targetIqA = 0.0f;
float targetIqB = 0.0f;

// ============================================================
// sleep 状态
// ============================================================

bool sleepA = true;
bool sleepB = true;

// ============================================================
// 软限位状态
// ============================================================

bool limitAUpper = false;
bool limitALower = false;
bool limitBUpper = false;
bool limitBLower = false;

String axisModeToString(uint8_t mode) {
  if (mode == AXIS_MODE_VELOCITY) {
    return "VEL";
  }
  if (mode == AXIS_MODE_TORQUE) {
    return "TORQUE";
  }
  return "POS";
}


void applyVfoc() {
  driverA.voltage_limit = Vfoc;
  driverB.voltage_limit = Vfoc;

  motorA.updateVoltageLimit(Vfoc);
  motorB.updateVoltageLimit(Vfoc);
}


void setVfoc(float value, uint8_t source) {
  Vfoc = constrain(value, 0.2f, FOC_VOLTAGE_LIMIT_MAX);
  applyVfoc();

  String msg = "";
  msg += "OK, VF=";
  msg += String(Vfoc, 4);

  sendReply(source, msg);
}


String getVfocInfo() {
  String msg = "";
  msg += "Vfoc=";
  msg += String(Vfoc, 4);
  return msg;
}


void applyIfoc() {
  targetIqA = constrain(targetIqA, -Ifoc, Ifoc);
  targetIqB = constrain(targetIqB, -Ifoc, Ifoc);

  if (modeA == AXIS_MODE_TORQUE) motorA.target = targetIqA;
  if (modeB == AXIS_MODE_TORQUE) motorB.target = targetIqB;

  motorA.updateCurrentLimit(Ifoc);
  motorB.updateCurrentLimit(Ifoc);
}


void setIfoc(float value, uint8_t source) {
  Ifoc = constrain(value, 0.0f, FOC_CURRENT_LIMIT_MAX);
  applyIfoc();
  sendReply(source, "OK, IF=" + String(Ifoc, 4));
}


String getIfocInfo() {
  String msg = "Ifoc=";
  msg += String(Ifoc, 4);
  return msg;
}


static BLDCMotor* getAxisMotor(char axis) {
  if (axis == 'A') return &motorA;
  if (axis == 'B') return &motorB;
  return nullptr;
}


static PIDController* getAxisPid(BLDCMotor& motor, char loopType) {
  if (loopType == 'V') return &motor.PID_velocity;
  if (loopType == 'P') return &motor.PID_angle;
  return nullptr;
}


static void makePidKey(char* key, char loopType, char axis, char gainType) {
  key[0] = loopType;
  key[1] = axis;
  key[2] = gainType;
  key[3] = '\0';
}


static bool ensurePidPrefsReady() {
  if (pidPrefsReady) return true;
  pidPrefsReady = pidPrefs.begin("pid", false);
  return pidPrefsReady;
}


static void loadPidValuesForAxis(char loopType, char axis) {
  BLDCMotor* motor = getAxisMotor(axis);
  if (motor == nullptr) return;

  PIDController* pid = getAxisPid(*motor, loopType);
  if (pid == nullptr) return;

  char key[4];
  makePidKey(key, loopType, axis, 'P');
  pid->P = pidPrefs.getFloat(key, pid->P);
  makePidKey(key, loopType, axis, 'I');
  pid->I = pidPrefs.getFloat(key, pid->I);
  makePidKey(key, loopType, axis, 'D');
  pid->D = pidPrefs.getFloat(key, pid->D);
  pid->reset();
}


void loadPidValuesFromFlash() {
  if (!ensurePidPrefsReady()) {
    Serial.println("PID NVS open failed, using compiled defaults.");
    return;
  }

  loadPidValuesForAxis('V', 'A');
  loadPidValuesForAxis('V', 'B');
  loadPidValuesForAxis('P', 'A');
  loadPidValuesForAxis('P', 'B');
  Serial.println("PID values loaded from NVS.");
}


static bool savePidValuesForAxis(char loopType, char axis) {
  BLDCMotor* motor = getAxisMotor(axis);
  if (motor == nullptr) return false;

  PIDController* pid = getAxisPid(*motor, loopType);
  if (pid == nullptr) return false;

  char key[4];
  makePidKey(key, loopType, axis, 'P');
  bool savedP = pidPrefs.putFloat(key, pid->P) == sizeof(float);
  makePidKey(key, loopType, axis, 'I');
  bool savedI = pidPrefs.putFloat(key, pid->I) == sizeof(float);
  makePidKey(key, loopType, axis, 'D');
  bool savedD = pidPrefs.putFloat(key, pid->D) == sizeof(float);
  return savedP && savedI && savedD;
}


static String getSinglePidInfo(char loopType, char axis) {
  BLDCMotor* motor = getAxisMotor(axis);
  if (motor == nullptr) return "ERR, UNKNOWN_AXIS";

  PIDController* pid = getAxisPid(*motor, loopType);
  if (pid == nullptr) return "ERR, UNKNOWN_PID_LOOP";

  String msg = "";
  msg += loopType;
  msg += axis;
  msg += "_PID=";
  msg += String(pid->P, 4);
  msg += ",";
  msg += String(pid->I, 4);
  msg += ",";
  msg += String(pid->D, 4);
  return msg;
}


String getPidInfo(char loopType, char axis) {
  if (loopType != 'V' && loopType != 'P') {
    return "ERR, UNKNOWN_PID_LOOP";
  }

  if (axis == '\0') {
    String msg = getSinglePidInfo(loopType, 'A');
    msg += "\n";
    msg += getSinglePidInfo(loopType, 'B');
    msg += "\n";
    return msg;
  }

  return getSinglePidInfo(loopType, axis);
}


static bool applyPidValuesToAxis(char loopType, char axis, float p, float i, float d) {
  BLDCMotor* motor = getAxisMotor(axis);
  if (motor == nullptr) return false;

  PIDController* pid = getAxisPid(*motor, loopType);
  if (pid == nullptr) return false;

  pid->P = p;
  pid->I = i;
  pid->D = d;

  // Clear the old integral/derivative history so the new gain starts cleanly.
  pid->reset();
  return true;
}


bool setPidValues(char loopType, char axis, float p, float i, float d, uint8_t source) {
  if ((loopType != 'V' && loopType != 'P') ||
      (axis != '\0' && axis != 'A' && axis != 'B')) {
    sendReply(source, "ERR, PID_FORMAT, USE_VPID<P,I,D>_OR_VAPID<P,I,D>");
    return false;
  }

  bool ok = true;
  if (axis == '\0') {
    ok = applyPidValuesToAxis(loopType, 'A', p, i, d) &&
         applyPidValuesToAxis(loopType, 'B', p, i, d);
  } else {
    ok = applyPidValuesToAxis(loopType, axis, p, i, d);
  }

  if (!ok) {
    sendReply(source, "ERR, PID_SET_FAILED");
    return false;
  }

  if (!ensurePidPrefsReady()) {
    sendReply(source, "ERR, PID_NVS_OPEN_FAILED, RAM_UPDATED_NOT_SAVED");
    return false;
  }

  bool saved = true;
  if (axis == '\0') {
    bool savedA = savePidValuesForAxis(loopType, 'A');
    bool savedB = savePidValuesForAxis(loopType, 'B');
    saved = savedA && savedB;
  } else {
    saved = savePidValuesForAxis(loopType, axis);
  }

  if (!saved) {
    sendReply(source, "ERR, PID_NVS_WRITE_FAILED, RAM_UPDATED_NOT_FULLY_SAVED");
    return false;
  }

  String msg = "OK, ";
  msg += loopType;
  if (axis != '\0') {
    msg += axis;
  }
  msg += "_PID=";
  msg += String(p, 4);
  msg += ",";
  msg += String(i, 4);
  msg += ",";
  msg += String(d, 4);
  sendReply(source, msg);
  return true;
}


void sleepAxis(char axis) {
  if (axis == 'A') {
    targetVelA = 0.0f;
    targetIqA = 0.0f;
    if (driverA.initialized) motorA.disable();
    digitalWrite(PIN_A_SLEEP, LOW);
    sleepA = true;
  } else if (axis == 'B') {
    targetVelB = 0.0f;
    targetIqB = 0.0f;
    if (driverB.initialized) motorB.disable();
    digitalWrite(PIN_B_SLEEP, LOW);
    sleepB = true;
  }
}


static void syncPositionTargetAfterWake(char axis) {
  if (axis == 'A' && modeA == AXIS_MODE_POSITION) {
    motorA.updateTorqueControlType(TorqueControlType::foc_current);
    motorA.updateMotionControlType(MotionControlType::angle);
    targetPosA = getRelativePosA();
    motorA.target = getAbsoluteTargetA(targetPosA);
    motorA.PID_angle.reset();
    motorA.PID_velocity.reset();
  } else if (axis == 'B' && modeB == AXIS_MODE_POSITION) {
    motorB.updateTorqueControlType(TorqueControlType::foc_current);
    motorB.updateMotionControlType(MotionControlType::angle);
    targetPosB = getRelativePosB();
    motorB.target = getAbsoluteTargetB(targetPosB);
    motorB.PID_angle.reset();
    motorB.PID_velocity.reset();
  }
}


bool wakeAxis(char axis) {
  BLDCMotor* motor = getAxisMotor(axis);
  if (!motor || motor->motor_status != FOCMotorStatus::motor_ready || !canWakeNow()) return false;
  applyVfoc();
  motor->enable();
  delay(10); // MS8313 wake settling before FOC resumes.
  syncPositionTargetAfterWake(axis);
  if (axis == 'A') sleepA = false;
  else sleepB = false;
  return true;
}


void sleepAllAxis() {
  sleepAxis('A');
  sleepAxis('B');
}


bool wakeAllAxis() {
  bool okA = wakeAxis('A');
  bool okB = wakeAxis('B');
  return okA && okB;
}


String getAxisLimitText(bool upper, bool lower) {
  if (upper && lower) {
    return "BOTH";
  }

  if (upper) {
    return "UPPER";
  }

  if (lower) {
    return "LOWER";
  }

  return "NONE";
}


void applyPositionSoftLimit() {
  // A single-turn angle is cyclic. There is no physical upper/lower stop at
  // 0/2PI, so velocity and torque commands must be allowed to cross zero.
  limitAUpper = false;
  limitALower = false;
  limitBUpper = false;
  limitBLower = false;
  return;
}


void setupDriver(BLDCDriver3PWM& driver) {
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver.voltage_limit = Vfoc;
  driver.pwm_frequency = PWM_FREQUENCY;
  driver.enable_active_high = true;
  if (driver.init()) driver.disable();
}


bool setupMotor(BLDCMotor& motor,
                BLDCDriver3PWM& driver,
                MT6701SSI& sensor,
                InlineCurrentSense& currentSense,
                float maxVel,
                const float velocityPid[3],
                const float positionPid[3],
                const float currentPid[3]) {
  if (!driver.initialized) return false;
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  currentSense.linkDriver(&driver);
  if (!currentSense.init()) {
    return false;
  }
  motor.linkCurrentSense(&currentSense);

  motor.voltage_limit = Vfoc;
  motor.velocity_limit = maxVel;

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.current_limit = Ifoc;
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.PID_velocity.P = velocityPid[0];
  motor.PID_velocity.I = velocityPid[1];
  motor.PID_velocity.D = velocityPid[2];
  motor.PID_velocity.limit = Ifoc;
  motor.PID_velocity.output_ramp = 100.0f;

  motor.PID_angle.P = positionPid[0];
  motor.PID_angle.I = positionPid[1];
  motor.PID_angle.D = positionPid[2];
  motor.PID_angle.output_ramp = 100.0f;
  motor.PID_angle.limit = maxVel;

  motor.LPF_velocity.Tf = 0.02f;

  motor.PID_current_q.P = currentPid[0];
  motor.PID_current_q.I = currentPid[1];
  motor.PID_current_q.D = currentPid[2];
  motor.PID_current_q.limit = Vfoc;
  motor.PID_current_d.P = currentPid[0];
  motor.PID_current_d.I = currentPid[1];
  motor.PID_current_d.D = currentPid[2];
  motor.PID_current_d.limit = Vfoc;
  motor.LPF_current_q.Tf = CURRENT_LPF_TF;
  motor.LPF_current_d.Tf = CURRENT_LPF_TF;

  // Enable SimpleFOC initialization/error diagnostics (MOT: / ERR-MOT:).
  // Serial TX is configured as non-blocking in setup(), so an unopened USB
  // CDC terminal must not hold the motor-control startup path here.
  motor.useMonitoring(Serial);

  int initOk = motor.init();
  int focOk = 0;
  if (initOk) {
    // SimpleFOC expects a continuous angle while checking sensor direction
    // and pole pairs. Unwrap only for initFOC; runtime remains single-turn.
    sensor.setCalibrationUnwrap(true);
    Serial.println("ENC: calibration unwrap ON");
    focOk = motor.initFOC();
    sensor.setCalibrationUnwrap(false);
    Serial.println("ENC: calibration unwrap OFF");
  }
  // initFOC failures must also drive nSLEEP LOW.
  motor.disable();
  return initOk && focOk;
}

void runAxisControl(BLDCMotor& motor,
                    bool axisSleep,
                    uint8_t axisMode,
                    float velocityTarget,
                    float absolutePositionTarget,
                    float torqueCurrentTarget) {
  if (axisSleep || systemProtected) {
    return;
  }

  motor.loopFOC();

  if (axisMode == AXIS_MODE_VELOCITY) {
    motor.move(velocityTarget);
  } else if (axisMode == AXIS_MODE_TORQUE) {
    motor.move(torqueCurrentTarget);
  } else {
    float currentAngle = motor.shaftAngle();
    float error = absolutePositionTarget - currentAngle;
    while (error > 0.5f * _2PI) error -= _2PI;
    while (error < -0.5f * _2PI) error += _2PI;
    // Give SimpleFOC an equivalent target nearest to the measured angle so
    // its PID sees the circular shortest-path error, not a 2PI jump.
    motor.move(currentAngle + error);
  }
}


void setAxisTorqueCurrent(char axis, float value, uint8_t source) {
  // FOC current mode: move() receives the requested q-axis current in A.
  float iq = constrain(value, -Ifoc, Ifoc);

  if (axis == 'A') {
    modeA = AXIS_MODE_TORQUE;
    motorA.updateCurrentLimit(Ifoc);
    motorA.updateTorqueControlType(TorqueControlType::foc_current);
    motorA.updateMotionControlType(MotionControlType::torque);
    targetIqA = iq;
    motorA.target = targetIqA;
    sendReply(source, "OK, TA=" + String(targetIqA, 4));
  } else if (axis == 'B') {
    modeB = AXIS_MODE_TORQUE;
    motorB.updateCurrentLimit(Ifoc);
    motorB.updateTorqueControlType(TorqueControlType::foc_current);
    motorB.updateMotionControlType(MotionControlType::torque);
    targetIqB = iq;
    motorB.target = targetIqB;
    sendReply(source, "OK, TB=" + String(targetIqB, 4));
  } else {
    sendReply(source, "ERR, UNKNOWN_AXIS");
  }
}


void setAxisVelocity(char axis, float value, uint8_t source) {
  if (axis == 'A') {
    modeA = AXIS_MODE_VELOCITY;
    motorA.updateTorqueControlType(TorqueControlType::foc_current);
    motorA.controller = MotionControlType::velocity;
    targetIqA = 0.0f;
    targetVelA = constrain(value, -A_MAX_VEL, A_MAX_VEL);
    motorA.target = targetVelA;
    sendReply(source, "OK, VA=" + String(targetVelA, 4));
  } else if (axis == 'B') {
    modeB = AXIS_MODE_VELOCITY;
    motorB.updateTorqueControlType(TorqueControlType::foc_current);
    motorB.controller = MotionControlType::velocity;
    targetIqB = 0.0f;
    targetVelB = constrain(value, -B_MAX_VEL, B_MAX_VEL);
    motorB.target = targetVelB;
    sendReply(source, "OK, VB=" + String(targetVelB, 4));
  } else {
    sendReply(source, "ERR, UNKNOWN_AXIS");
  }
}


static void enterPositionMode(BLDCMotor& motor, uint8_t& axisMode) {
  bool modeChanged =
      axisMode != AXIS_MODE_POSITION ||
      motor.controller != MotionControlType::angle;

  motor.updateTorqueControlType(TorqueControlType::foc_current);
  motor.updateMotionControlType(MotionControlType::angle);
  axisMode = AXIS_MODE_POSITION;

  if (modeChanged) {
    motor.PID_angle.reset();
    motor.PID_velocity.reset();
  }
}


void setAxisPosition(char axis, float value, uint8_t source) {
  if (axis == 'A') {
    enterPositionMode(motorA, modeA);
    targetIqA = 0.0f;
    targetPosA = constrain(value, A_MIN_POS, A_MAX_POS);
    motorA.target = getAbsoluteTargetA(targetPosA);
    sendReply(source, "OK, PA=" + String(targetPosA, 4));
  } else if (axis == 'B') {
    enterPositionMode(motorB, modeB);
    targetIqB = 0.0f;
    targetPosB = constrain(value, B_MIN_POS, B_MAX_POS);
    motorB.target = getAbsoluteTargetB(targetPosB);
    sendReply(source, "OK, PB=" + String(targetPosB, 4));
  } else {
    sendReply(source, "ERR, UNKNOWN_AXIS");
  }
}


void setAxisDisplacement(char axis, float value, uint8_t source) {
  if (axis == 'A') {
    enterPositionMode(motorA, modeA);
    targetIqA = 0.0f;
    targetPosA = constrain(getRelativePosA() + value, A_MIN_POS, A_MAX_POS);
    motorA.target = getAbsoluteTargetA(targetPosA);
    sendReply(source, "OK, PA=" + String(targetPosA, 4));
  } else if (axis == 'B') {
    enterPositionMode(motorB, modeB);
    targetIqB = 0.0f;
    targetPosB = constrain(getRelativePosB() + value, B_MIN_POS, B_MAX_POS);
    motorB.target = getAbsoluteTargetB(targetPosB);
    sendReply(source, "OK, PB=" + String(targetPosB, 4));
  } else {
    sendReply(source, "ERR, UNKNOWN_AXIS");
  }
}


void allTargetZero(uint8_t source) {
  targetPosA = 0.0f;
  targetPosB = 0.0f;
  targetIqA = 0.0f;
  targetIqB = 0.0f;

  enterPositionMode(motorA, modeA);
  enterPositionMode(motorB, modeB);

  motorA.target = getAbsoluteTargetA(0.0f);
  motorB.target = getAbsoluteTargetB(0.0f);

  sendReply(source, "OK, ALL0");
}
