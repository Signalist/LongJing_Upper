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
//    Device Name：Upper3.0-FOC
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
// 10、编码器角度接口说明：
//    getSensorAngle()：对应 CS 置 0 访问编码器硬件，返回当前的单圈机械角，范围 0～2π。
//    sensor.update()：调用 getSensorAngle，并根据前后两次读数更新单圈角、累计圈数以及角速度。
//    getMechanicalAngle()： 返回最近一次 sensor.update 后的单圈机械角，范围 0～2π。
//    getAngle() 基于 sensor.update 更新后的单圈角和圈数信息，返回连续的多圈机械角，可大于 2π 或小于 0。
//    编码器读数 0 或 2π 很可能是悬空带来的全 0 或全 1
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

  // Upper3.0 avoids GPIO35-37 and strapping GPIO45-46.
  // MS8313 nSLEEP has an external pulldown; HIGH still means awake.
  digitalWrite(PIN_A_SLEEP, LOW);
  digitalWrite(PIN_B_SLEEP, LOW);
  pinMode(PIN_A_SLEEP, OUTPUT);
  pinMode(PIN_B_SLEEP, OUTPUT);
  pinMode(PIN_A_FAULT, INPUT_PULLUP);
  pinMode(PIN_B_FAULT, INPUT_PULLUP);
  digitalWrite(PIN_A_SLEEP, LOW);
  digitalWrite(PIN_B_SLEEP, LOW);

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
  Serial.println("Initializing snesors...");
  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  delay(100);
  digitalWrite(PIN_ENC_A_CS, HIGH);
  digitalWrite(PIN_ENC_B_CS, HIGH);
  pinMode(PIN_ENC_A_CS, OUTPUT);
  pinMode(PIN_ENC_B_CS, OUTPUT);
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
  setupDriver(driverA);
  setupDriver(driverB);
  Serial.println("PWM available");

  // Motor 初始化
  Serial.println("Initializing motors...");

  bool motorAReady = setupMotor(motorA, driverA, sensorA, currentSenseA, A_MAX_VEL,
                                A_VELOCITY_PID_DEFAULT, A_POSITION_PID_DEFAULT,
                                A_CURRENT_PID_DEFAULT);
  bool motorBReady = setupMotor(motorB, driverB, sensorB, currentSenseB, B_MAX_VEL,
                                B_VELOCITY_PID_DEFAULT, B_POSITION_PID_DEFAULT,
                                B_CURRENT_PID_DEFAULT);
  // Keep both bridges asleep while communication services start.
  sleepAllAxis();
  loadPidValuesFromFlash();
  Serial.print("FOC ready: A=");
  Serial.print(motorAReady);
  Serial.print(", B=");
  Serial.println(motorBReady);
  targetVelA = 0.0f;
  targetVelB = 0.0f;
  targetPosA = 0.0f;
  targetPosB = 0.0f;
  targetIqA = 0.0f;
  targetIqB = 0.0f;
  Serial.println("Target available");
  modeA = AXIS_MODE_POSITION;
  modeB = AXIS_MODE_POSITION;
  motorA.controller = MotionControlType::angle;
  motorB.controller = MotionControlType::angle;

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
  Serial.println("Motors under control");

  // 无线通信初始化
  Serial.println("Initializing communicators...");
  setupBLE();
  setupWiFi();
  setupCAN();

  // Calibration needs an awake bridge; restore hold only after setup finishes.
  if (motorAReady) wakeAxis('A');
  if (motorBReady) wakeAxis('B');

  // 广播ready以及重要参数
  broadcastMsg(motorAReady && motorBReady ? "System all ready." : "System degraded: FOC initialization failed.");
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

  handleCAN();
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
