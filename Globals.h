#pragma once

#include "Config.h"
void runAxisControl(BLDCMotor& motor,
                    bool axisSleep,
                    uint8_t axisMode,
                    float velocityTarget,
                    float absolutePositionTarget,
                    float torqueCurrentTarget);

// ============================================================
// MT6701 SSI 编码器类声明
// 具体实现放在 Encoder.ino
// ============================================================

class MT6701SSI : public Sensor {
  public:
    MT6701SSI(uint8_t csPin, SPIClass* spiBus);
    void init() override;
    float getSensorAngle() override;
    void update() override;
    float getMechanicalAngle() override;
    float getAngle() override;
    double getPreciseAngle() override;
    float getVelocity() override;
    int32_t getFullRotations() override;
    void setCalibrationUnwrap(bool enabled);

  private:
    uint8_t _csPin;
    SPIClass* _spiBus;
    float _angle;
    float _velocity;
    float _velocityAngle;
    uint32_t _velocityTimestamp;
    bool _hasSample;
    bool _hasVelocitySample;
    bool _calibrationUnwrap;
    bool _unwrapHasSample;
    float _unwrapAngle;
    float _unwrapPrevious;

    uint32_t readRaw24();
};

// ============================================================
// 全局对象声明
// ============================================================

extern SPIClass spiBus;

extern MT6701SSI sensorA;
extern MT6701SSI sensorB;

extern BLDCDriver3PWM driverA;
extern BLDCDriver3PWM driverB;

extern BLDCMotor motorA;
extern BLDCMotor motorB;

extern InlineCurrentSense currentSenseA;
extern InlineCurrentSense currentSenseB;

extern Preferences zeroPrefs;

#if ENABLE_BLE
extern BLEServer* bleServer;
extern BLECharacteristic* txCharacteristic;
extern BLECharacteristic* rxCharacteristic;
extern bool bleDeviceConnected;
#endif

#if WIFI_MODE != WIFI_MODE_OFF
extern WiFiServer wifiServer;
extern WiFiClient wifiClient;
extern String wifiRxBuffer;
#endif

extern String serialRxBuffer;

// ============================================================
// Flash 零位运行变量
// ============================================================

extern float homeAOffset;
extern float homeBOffset;

// ============================================================
// 电机运行变量
// ============================================================

extern float Vfoc;
extern float Ifoc;

extern uint8_t modeA;
extern uint8_t modeB;

extern float targetVelA;
extern float targetVelB;

extern float targetPosA;
extern float targetPosB;

extern float targetIqA;
extern float targetIqB;

extern bool sleepA;
extern bool sleepB;

extern bool limitAUpper;
extern bool limitALower;
extern bool limitBUpper;
extern bool limitBLower;

// ============================================================
// Fault 运行变量
// ============================================================

extern bool currentFaultA;
extern bool currentFaultB;
extern bool currentVonFault;

extern bool latchFaultA;
extern bool latchFaultB;
extern bool latchVonFault;

extern uint64_t A_fault_time_ms;
extern uint64_t B_fault_time_ms;
extern uint64_t Von_fault_time_ms;

extern float Von_fault_value;
extern bool systemProtected;

// ============================================================
// 函数声明
// ============================================================

// Time.ino
extern bool timeSynced;
extern int64_t timeOffsetMs;
extern bool heartbeat;

String msToTimestamp(uint64_t ms);
uint64_t getCorrectedTimeMs();
String getTimestampText();
String getTimestampHeader();
void syncHostTimeMs(uint64_t hostTimeMs);
void syncHostTimeHms(uint8_t hh, uint8_t mm, uint8_t ss);
String getTimeInfo();

// Zero.ino
void loadZeroOffsetsFromFlash();
void saveZeroOffsetsToFlash();
void printZeroOffsetsAtBoot();
float getRelativePosA();
float getRelativePosB();
float getAbsoluteTargetA(float relativeTarget);
float getAbsoluteTargetB(float relativeTarget);
String getCalibrationInfo();
String setCurrentPositionAsZero(char axis);

// Fault.ino
float readVonVoltage();
bool hasAnyFaultLatch();
bool hasAnyCurrentFault();
String getFaultBriefText();
String getAxisFaultBriefText(char axis);
bool updateFaultLatch();
String getFaultInfo();
String clearFaultLatch();
bool canWakeNow();

// Motor.ino
String axisModeToString(uint8_t mode);
void applyVfoc();
void setVfoc(float value, uint8_t source);
String getVfocInfo();
void applyIfoc();
void setIfoc(float value, uint8_t source);
String getIfocInfo();
void loadPidValuesFromFlash();
String getPidInfo(char loopType, char axis);
bool setPidValues(char loopType, char axis, float p, float i, float d, uint8_t source);
void sleepAxis(char axis);
bool wakeAxis(char axis);
void sleepAllAxis();
bool wakeAllAxis();
String getAxisLimitText(bool upper, bool lower);
void applyPositionSoftLimit();
void setupDriver(BLDCDriver3PWM& driver);
bool setupMotor(BLDCMotor& motor,
                BLDCDriver3PWM& driver,
                MT6701SSI& sensor,
                InlineCurrentSense& currentSense,
                float maxVel,
                const float velocityPid[3],
                const float positionPid[3],
                const float currentPid[3]);
void setAxisVelocity(char axis, float value, uint8_t source);
void setAxisPosition(char axis, float value, uint8_t source);
void setAxisDisplacement(char axis, float value, uint8_t source);
void setAxisTorqueCurrent(char axis, float value, uint8_t source);
void allTargetZero(uint8_t source);

// Commu.ino
void bleSendText(const String& msg);
void setupBLE();
void setupWiFi();
void handleWiFiCommand();
void handleSerialCommand();

// Led.ino
void setupStatusLeds();
void updateStatusLeds();
void markCommunicationActivity();

// Protocol.ino
void sendReply(uint8_t source, const String& msg);
void broadcastMsg(const String& msg);
void broadcastRaw(const String& msg);
String getWifiBriefText();
String getStatusText();
String getSingleAxisStatus(char axis);
String getCommInfo();
String getLimitInfo();
String getCurrentInfo();
String getCxkText();
void handleCommand(String cmd, uint8_t source);
void broadcastRunningStatus();

// Can.ino; source 3 uses framed classic CAN text commands.
void setupCAN();
void handleCAN();
void canSendText(const String& msg);
String getCanInfo();
void handleBLECommand();
