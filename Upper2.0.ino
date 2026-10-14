// ============================================================
// 写在前面：
//    速度模式主要用于调试；正式运行前应确认速度限幅和安全策略。
//    OTA 未启用，固件通过 USB 下载。
//    POWER_SUPPLY_VOLTAGE 当前固定为 12 V，尚未使用 ADC 测量值补偿。
//    sleep 时不更新 FOC，wake 时会清空 PID。
//    电机零位会持久保存，用 1 的方法重新烧录后可能仍然存在。
//    TorqueMode测试中，堵转实验未完成
//
// 0、下载与重启
//   BOOT 接 GND，EN 触碰 V3 复位，进入下载模式。
//   BOOT 接 V33，EN 触碰 V3 复位，进入工作模式。
//
// 1、USB 串口
//    Baud Rate: 115200
//    Data Bits: 8
//    Stop Bits: 1
//    Parity: None
//    Flow Control: None
//
// 2、BLE
//    Device Name：Upper1.1-FOC
//    SERVICE_UUID：19161916-1916-1916-1916-aabbccdd0000
//    RX_CHAR_UUID：19161916-1916-1916-1916-aabbccdd0001（PC -> ESP32）
//    TX_CHAR_UUID：19161916-1916-1916-1916-aabbccdd0002（ESP32 -> PC）
//
// 3、WiFi
//    WiFi = 1 ROUTER，ESP32 TCP Server模式
//    Port：5090
//    WIFI_ROUTER_SSID: "CZ1916"
//    WIFI_ROUTER_PASS: "CZ19161916cz"
//
// 4、通信协议
//    命令以 '\n' 或 '\r' 结束。
//    BLE、Wi-Fi、USB命令响应将原路返回，广播消息将三路同时发送。
//    目前没有包序号，没有异常重发机制。
//    临时握手协议：发送 HELLO 5090，返回 Hello wxy。
//    事件消息带时间戳，长期日志由上位机保存（未实现）。
//    完整命令表见 COMMANDS.md。
//
// 5、启动广播
//    System all ready.
//    CalibrationInfo
//    LimitInfo
//    StatusText
//
// 6、心跳输出
//    使用 HEARTBEAT ON / HEARTBEAT OFF 控制心跳输出。
//    RUN_TIME_MS 周期输出 Running...；
//    STAT_TIME_MS 周期输出 StatusText。
//    默认关。
//
// 7、系统时间
//    输出信号均会带有时间戳，单位精确到毫秒。
//    Timestamp 上电后从零开始，可发送 TIME<HHMMSS> 同步，
//    例如 TIME235959。同步动作由上位机发起。
//
// 8、故障保护
//    FOC 失败的轴不会进入正常闭环控制。
//    出现 "Protection triggered, all motors sleep." 表示系统进入保护。
//    应先查询 FAULT?，故障消失后发送 CLEARFAULT，再执行 WAKE。
//    默认关。
//
// 9、数据单位
//    位置/角度：rad；速度：rad/s；电压：V；电流：A.
//
// 10、ZeroHero angle interfaces:
//    getSensorAngle(): hardware single-turn angle, [0, 2PI).
//    update(): sample sensor; getMechanicalAngle(): cached single-turn angle.
//    getAngle(): continuous during initFOC and runtime.
//    A legitimate angle of zero must not be discarded.
//      
// ============================================================



#include "Config.h"
#include "Globals.h"
bool heartbeat = 0;

void setup() 
{
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  // Hardware CDC can report "connected" as soon as Windows enumerates the
  // device, even when no terminal has opened the COM port.  Keep enough room
  // for all boot messages and never wait for a host reader.
  Serial.setTxBufferSize(8192);
  Serial.setTxTimeoutMs(0);
#endif
  setupStatusLeds();
  delay(100);
  Serial.println();
  Serial.println("ESP32-S3 WAKEUP");
  Serial.println("BLE OTA OFF.");
  Serial.println("FAULT N STOP OFF.");

  // GPIO35/36/37 are available only when Octal PSRAM is disabled. Do not
  // touch these shared pins if the board configuration enabled PSRAM.
  if (ESP.getPsramSize() != 0) {
    Serial.println("FATAL: PSRAM must be Disabled; GPIO35/36/37 are in use.");
    while (true) {
      updateStatusLeds();
      delay(1);
    }
  }
  Serial.println("PSRAM disabled; GPIO35/36/37 available.");
  
  // MS8313 管脚定义
  pinMode(PIN_A_SLEEP, OUTPUT);
  pinMode(PIN_B_SLEEP, OUTPUT);
  pinMode(PIN_C_SLEEP, OUTPUT);
  pinMode(PIN_A_FAULT, INPUT_PULLUP);
  pinMode(PIN_B_FAULT, INPUT_PULLUP);
  pinMode(PIN_C_FAULT, INPUT_PULLUP);
  digitalWrite(PIN_A_SLEEP, LOW);
  digitalWrite(PIN_B_SLEEP, LOW);
  digitalWrite(PIN_C_SLEEP, LOW);

  // 打印零位参数
  loadZeroOffsetsFromFlash();
  printZeroOffsetsAtBoot();

  // ADC分辨率12位，打印电压参数
  analogReadResolution(12);    
  delay(100);
  Serial.print("Initial VON=");
  Serial.print(readVonVoltage(), 2);
  Serial.println("V");

  // Encoder 串口通信与初始化，打印初始角度
  Serial.println("ZEROHERO: 03_MT6701_OfficialSSI");
  Serial.println("Initializing snesors...");
  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  delay(100);
  sensorA.init();
  sensorB.init();
  Serial.print("Sensor A single=");
  Serial.print(sensorA.getSensorAngle(), 6);
  Serial.print(", multi=");
  Serial.println(sensorA.getAngle(), 6);
  Serial.print("Sensor B single=");
  Serial.print(sensorB.getSensorAngle(), 6);
  Serial.print(", multi=");
  Serial.println(sensorB.getAngle(), 6);

  // Driver 初始化
  Serial.println("Initializing drivers...");
  digitalWrite(PIN_A_SLEEP, HIGH);
  digitalWrite(PIN_B_SLEEP, HIGH);
  digitalWrite(PIN_C_SLEEP, LOW);
  Serial.println("Drivers A/B wake; C disabled");
  delay(100);
  setupDriver(driverA);
  setupDriver(driverB);
  Serial.println("PWM available");

  // Motor 初始化
  Serial.println("Initializing motors...");

  // A performs the complete SimpleFOC calibration on every boot.
  motorA.voltage_sensor_align = MOTOR_A_ALIGN_VOLTAGE;
  Serial.print("A full init: PP=11 reference, align voltage=");
  Serial.print(motorA.voltage_sensor_align, 2);
  Serial.println("V, diagnostic build=ALIGN_MOVE_V1");
  // New SSI framing requires fresh direction/electrical-zero calibration.
  motorB.sensor_direction = Direction::UNKNOWN;
  motorB.zero_electric_angle = NOT_SET;
  currentSenseB.skip_align = true;

  bool motorAReady = setupMotor(motorA, driverA, sensorA, currentSenseA, A_MAX_VEL,
                                A_VELOCITY_PID_DEFAULT, A_POSITION_PID_DEFAULT,
                                A_CURRENT_PID_DEFAULT, true);
  bool motorBReady = setupMotor(motorB, driverB, sensorB, currentSenseB, B_MAX_VEL,
                                B_VELOCITY_PID_DEFAULT, B_POSITION_PID_DEFAULT,
                                B_CURRENT_PID_DEFAULT, true);
  bool motorCReady = false;
  loadPidValuesFromFlash();
  Serial.print("A position PID: P=");
  Serial.print(motorA.PID_angle.P, 4);
  Serial.print(", I=");
  Serial.print(motorA.PID_angle.I, 4);
  Serial.print(", D=");
  Serial.println(motorA.PID_angle.D, 4);
  Serial.print("B position PID: P=");
  Serial.print(motorB.PID_angle.P, 4);
  Serial.print(", I=");
  Serial.print(motorB.PID_angle.I, 4);
  Serial.print(", D=");
  Serial.println(motorB.PID_angle.D, 4);
  Serial.print("FOC ready: A=");
  Serial.print(motorAReady);
  Serial.print(", B=");
  Serial.print(motorBReady);
  Serial.print(", C=");
  Serial.println(motorCReady);
  Serial.println("A torque control: FOC_CURRENT (full init and ADC measured)");
  Serial.println("B torque control: FOC_CURRENT (ADC measured)");
  targetVelA = 0.0f;
  targetVelB = 0.0f;
  targetVelC = 0.0f;
  targetPosA = 0.0f;
  targetPosB = 0.0f;
  targetPosC = 0.0f;
  targetIqA = 0.0f;
  targetIqB = 0.0f;
  targetIqC = 0.0f;
  Serial.println("Target available");
  modeA = AXIS_MODE_POSITION;
  modeB = AXIS_MODE_POSITION;
  modeC = AXIS_MODE_POSITION;
  motorA.controller = MotionControlType::angle;
  motorB.controller = MotionControlType::angle;
  motorC.controller = MotionControlType::angle;
  sleepA = !motorAReady;
  sleepB = !motorBReady;
  sleepC = !motorCReady;
  Serial.println("Closed-loop available");
  
  // Start closed-loop control by holding the post-calibration position rather
  // than immediately chasing an old zero offset stored in flash.
  targetPosA = getRelativePosA();
  targetPosB = getRelativePosB();
  motorA.target = motorA.shaftAngle();
  motorB.target = motorB.shaftAngle();
  motorA.PID_angle.reset();
  motorB.PID_angle.reset();
  motorA.PID_velocity.reset();
  motorB.PID_velocity.reset();
  Serial.print("A position start: current=");
  Serial.print(targetPosA, 6);
  Serial.print(", target=");
  Serial.println(targetPosA, 6);
  Serial.print("B position start: current=");
  Serial.print(targetPosB, 6);
  Serial.print(", target=");
  Serial.println(targetPosB, 6);
  Serial.println("Motors under control");

  // 无线通信初始化
  Serial.println("Initializing communicators...");
  setupBLE();
  setupWiFi();

  // Same target operation as the ALL0 command, after calibration and NVS load.
  // initFOC failure must not start automatic homing.
  if (motorAReady && motorBReady) {
    allTargetZero(0);
    Serial.println("BOOT_ALL0: A/B mechanical-zero targets queued.");
  } else {
    sleepAllAxis();
    Serial.println("ERR, BOOT_ALL0_INIT_FAILED");
  }

   // 此处原本是做自启动控制，但发现 Sleep 状态无法完成 motor.initFOC()，遂废弃
// #if AUTO_WAKE_ON_BOOT
//   wakeAllAxis();
//   Serial.println("AUTO_WAKE_ON_BOOT=ON, motors enabled.");
// #else
//   sleepAllAxis();
//   Serial.println("AUTO_WAKE_ON_BOOT=OFF, motors sleeping. Send WAKE to enable.");
// #endif

  // 广播ready以及重要参数
  broadcastMsg("System all ready.");
  broadcastMsg(getCalibrationInfo());
  broadcastMsg(getLimitInfo());
  broadcastMsg(getStatusText());

}


void loop()
{
  static uint32_t lastDotTime = 0;
  static uint32_t lastRunTime = 0;
  static uint32_t lastStatTime = 0;

  uint32_t nowMs = millis();
  updateStatusLeds();

  bool faultNow = updateFaultLatch();
  if (faultNow && !systemProtected) {
    sleepAllAxis();
    systemProtected = true;
    broadcastMsg("Protection triggered, all motors sleep.");
    broadcastMsg(getFaultInfo());
  }

  applyPositionSoftLimit();
  runAxisControl(motorA,
                 sleepA,
                 modeA,
                 targetVelA,
                 getAbsoluteTargetA(targetPosA),
                 targetIqA);
  runAxisControl(motorB,
                 sleepB,
                 modeB,
                 targetVelB,
                 getAbsoluteTargetB(targetPosB),
                 targetIqB);

  handleBLECommand();
  handleSerialCommand();
  handleWiFiCommand();

  if (heartbeat) {
    if (nowMs - lastRunTime >= RUN_TIME_MS) {
      lastRunTime = nowMs;
      broadcastRaw("\n\nRunning");
    }

    if (nowMs - lastDotTime >= DOT_TIME_MS) {
      lastDotTime = nowMs;
      broadcastRaw(".");
    }

    if (nowMs - lastStatTime >= STAT_TIME_MS) {
      lastStatTime = nowMs;
      broadcastMsg("");
      broadcastMsg(getStatusText());
    }
  }
}
