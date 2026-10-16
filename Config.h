#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <Preferences.h>
#include <WiFi.h>

// ============================================================
// 功能开关
// ============================================================

// 是否打开蓝牙
#define ENABLE_BLE                 1

// 1 = 启用 nFAULT / VON 故障检测与保护
// 0 = 关闭故障检测，程序忽略故障并持续运行
#define ENABLE_FAULT_CHECK         0

// 是否打开WIFI/模式选择
#define WIFI_MODE                  0
#define WIFI_MODE_OFF              1   // WIFI关闭
#define WIFI_MODE_ROUTER           1   // ESP32与上位机在同一路由器
#define WIFI_MODE_PHONE            2   // ESP32连接上位机热点
#define WIFI_MODE_AP               3   // 上位机连接ESP32热点

// 回传是否加上时间戳
#define IF_TIMESTAMP               1

// // 0 = 上电后保持 SLEEP，需要 WAKE 指令才允许运动
// // 1 = 上电后自动 WAKE
// #define AUTO_WAKE_ON_BOOT          0

// ============================================================
// 电机参数和控制限制
// ============================================================

// 电机极对数
#define MOTOR_A_POLE_PAIRS         11
#define MOTOR_B_POLE_PAIRS         7
#define MOTOR_C_POLE_PAIRS         11
#define ENABLE_C_AXIS              0

// Fixed calibration captured with axes unloaded and PP check passing.
// Recalibrate these values if phase wiring or encoder mounting changes.
#define MOTOR_A_SENSOR_DIRECTION   Direction::CCW
#define MOTOR_A_ZERO_ELEC_ANGLE    2.28f
#define MOTOR_B_SENSOR_DIRECTION   Direction::CCW
#define MOTOR_B_ZERO_ELEC_ANGLE    4.41f

// 
#define POWER_SUPPLY_VOLTAGE       15.0f
#define FOC_VOLTAGE_LIMIT_DEFAULT  12.0f      // 初始 VF
#define FOC_VOLTAGE_LIMIT_MAX      14.0f
#define PWM_FREQUENCY              25000

// 3510 motor parameters used by SimpleFOC current and motion control (axes A/C)
#define MOTOR_A_PHASE_RESISTANCE   8.80f      // Ohm
#define MOTOR_A_KV_RATING          45.0f      // rpm/V
#define MOTOR_A_LQ                 0.00395f   // H
#define MOTOR_A_LD                 0.00395f   // H
#define MOTOR_A_ALIGN_VOLTAGE      6.00f      // V, about 0.68 A at standstill (short calibration only)

// 2804 motor parameters used by SimpleFOC current and motion control (axis B)
#define MOTOR_B_PHASE_RESISTANCE   2.55f      // Ohm, phase resistance (line-to-line is about 5.1 Ohm)
#define MOTOR_B_KV_RATING          220.0f     // rpm/V
#define MOTOR_B_LQ                 0.00086f   // H
#define MOTOR_B_LD                 0.00086f   // H

#define MOTOR_C_PHASE_RESISTANCE   MOTOR_A_PHASE_RESISTANCE
#define MOTOR_C_KV_RATING          MOTOR_A_KV_RATING
#define MOTOR_C_LQ                 MOTOR_A_LQ
#define MOTOR_C_LD                 MOTOR_A_LD

#define FOC_CURRENT_LIMIT_DEFAULT  0.50f      // 初始 IF
#define FOC_CURRENT_LIMIT_MAX      1.00f      // A, maximum value accepted by IF command

// INA240A2 inline phase-current sensing: 20 mOhm shunt, gain 50 V/V.
#define CURRENT_SHUNT_RESISTANCE   0.020f
#define CURRENT_SENSE_GAIN         50.0f
#define CURRENT_LPF_TF             0.002f

// Default motion-loop PID values: {P, I, D}.
// Values saved in NVS take precedence over these defaults at startup.
static constexpr float A_VELOCITY_PID_DEFAULT[3] = {0.10f, 0.20f, 0.0f};
static constexpr float B_VELOCITY_PID_DEFAULT[3] = {0.10f, 0.20f, 0.0f};
static constexpr float C_VELOCITY_PID_DEFAULT[3] = {0.10f, 0.20f, 0.0f};

// Hardware-tuned cascade position loop: velocity PI supplies steady-state torque.
// High angle D amplifies encoder quantization; use a moderate pure P outer loop.
static constexpr float A_POSITION_PID_DEFAULT[3] = {16.0f, 0.0f, 0.0f};
static constexpr float B_POSITION_PID_DEFAULT[3] = {30.0f, 4.0f, 2.0f};
static constexpr float C_POSITION_PID_DEFAULT[3] = {6.0f, 0.0f, 0.0f};

static constexpr float A_CURRENT_PID_DEFAULT[3] = {3.0f, 300.0f, 0.0f};
static constexpr float B_CURRENT_PID_DEFAULT[3] = {3.0f, 300.0f, 0.0f};
static constexpr float C_CURRENT_PID_DEFAULT[3] = {3.0f, 300.0f, 0.0f};

// 速度限制，单位 rad/s
#define A_MAX_VEL                  6.28f
#define B_MAX_VEL                  6.28f
#define C_MAX_VEL                  6.28f

// Continuous position limits relative to the boot-selected mechanical home.
// Signed positions are distinct; historical turns are not restored.
// A limits were re-measured in the original PA frame (home 1.308486 rad).
// The temporary REBASEA+1 offset is no longer present; do not subtract it again.
// They are NOT absolute cable-travel protection after a power-cycle/rebase.
// Startup defaults; ALIMIT<lower,upper> updates the A limits in RAM only.
#define A_MIN_POS                  -7.2f
#define A_MAX_POS                   4.3f
#define B_MIN_POS                  -6.28318530718f
#define B_MAX_POS                   6.28318530718f
#define C_MIN_POS                   0.0f
#define C_MAX_POS                   6.28318530718f

// 母线电压保护阈值
#define VON_MIN                    8.0f
#define VON_MAX                    15.0f

// ADC 换算参数
#define ADC_REF_VOLTAGE            3.3f
#define ADC_MAX_VALUE              4095.0f
#define VON_R_HIGH                 56000.0f
#define VON_R_LOW                  10000.0f
#define VON_SCALE                  ((VON_R_HIGH + VON_R_LOW) / VON_R_LOW)

// ============================================================
// 自动广播时间设置，单位：ms
// ============================================================

#define DOT_TIME_MS                1000UL
#define RUN_TIME_MS                10000UL
#define STAT_TIME_MS               60000UL

// ============================================================
// BLE 参数，OTA 已删除，只保留普通控制通道
// ============================================================

#define DEVICE_NAME                "Upper2.0-FOC"

#define SERVICE_UUID               "19161916-1916-1916-1916-aabbccdd0000"
#define RX_CHAR_UUID               "19161916-1916-1916-1916-aabbccdd0001"
#define TX_CHAR_UUID               "19161916-1916-1916-1916-aabbccdd0002"

#if ENABLE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif

// ============================================================
// WiFi 参数
// ============================================================

#define WIFI_ROUTER_SSID           "CZ1916"
#define WIFI_ROUTER_PASS           "CZ19161916cz"

#define WIFI_PHONE_SSID            "YourPhoneHotspot"
#define WIFI_PHONE_PASS            "YourPhonePassword"

#define WIFI_AP_SSID               "Upper2.0-FOC"
#define WIFI_AP_PASS               "19168888"

#define WIFI_TCP_PORT              5090

// ============================================================
// MT6701 SSI / SPI 管脚
// ============================================================

#define PIN_SPI_SCK                4
#define PIN_SPI_MOSI               5
#define PIN_SPI_MISO               6

#define PIN_ENC_A_CS               17
#define PIN_ENC_B_CS               35
#define PIN_ENC_C_CS               36

// ============================================================
// MS8313 管脚
// ============================================================

// A 轴
#define PIN_A_U                    9
#define PIN_A_V                    10
#define PIN_A_W                    11
#define PIN_A_SLEEP                41
#define PIN_A_FAULT                42

// B 轴
#define PIN_B_U                    12
#define PIN_B_V                    13
#define PIN_B_W                    14
#define PIN_B_SLEEP                39
#define PIN_B_FAULT                40

// C 轴
#define PIN_C_U                    21
#define PIN_C_V                    47
#define PIN_C_W                    48
#define PIN_C_SLEEP                37
#define PIN_C_FAULT                38

// 母线电压采样
#define PIN_VON_ADC                18

// INA240A2 phase-current outputs (two shunts per axis)
#define PIN_A_CURRENT_U            1
#define PIN_A_CURRENT_V            2
#define PIN_B_CURRENT_U            7
#define PIN_B_CURRENT_V            8
#define PIN_C_CURRENT_U            15
#define PIN_C_CURRENT_V            16

// Status LEDs, active HIGH
#define PIN_LED1                   45
#define PIN_LED2                   3
#define LED1_TOGGLE_INTERVAL_MS    500UL
#define LED2_HOLD_TIME_MS          500UL

// ============================================================
// 运行模式定义
// ============================================================

#define AXIS_MODE_VELOCITY         0
#define AXIS_MODE_POSITION         1
#define AXIS_MODE_TORQUE           2   // FOC current torque mode: target is q-axis current Iq (A)
