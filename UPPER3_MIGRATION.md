# Upper3.0 双轴固件迁移

依据本地 `../source/Upper3.0.pdf`、`../source/Encoder3.1-2804.pdf` 和旧版
`../source/Upper2-board.pdf` 核对。用户所述 `Encoder3.1.pdf` 实际文件名带 `-2804`。
CAN 参考项目实际位置为 `F:/ARM/work/cool_robot_arm/cool_robot_arm`。

## 硬件与引脚

仅保留 A（3510，11 极对）和 B（2804，7 极对）。电阻、KV、电感、PID 默认值沿用迁移前
工作区的 A/B 配置；C 对象、采样、FOC、故障、零位及命令全部删除。

| 信号 | Upper2 / 原代码 GPIO | Upper3 GPIO |
|---|---:|---:|
| A PWM U/V/W | 9/10/11 | 9/10/11 |
| B PWM U/V/W | 12/13/14 | 12/13/14 |
| A nSLEEP / nFAULT | 41/42 | 41/42 |
| B nSLEEP / nFAULT | 39/40 | 39/40 |
| SSI SCK / MISO | 4/6 | 4/6 |
| MOSI（编码器未接入） | 5 | 5 |
| A 编码器 CSA | 17 | 17 |
| B 编码器 CSB | 35 | **38** |
| A 电流 U/V | 1/2 | 1/2 |
| B 电流 U/V | 7/8 | 7/8 |
| AD12 电压采样 | 18 | 18 |
| LED1 / LED2 | 45/3 | **21/47** |
| CAN1 TX / RX | 无 | **15/16** |

新图中 A 编码器并入主板。B 的 H2/CN1：1=VDD、2=CSB、3=SCK、4=MISO、5=GND、
6/7/8=OUTB_U/V/W，9/10 未连接。两个 MT6701 仍使用 SSI，初始化读取前先将两个 CS 拉高。
电流采样仍为 INA240A2、20mΩ、50V/V。

不再占用 GPIO35～37、45～46，删除旧版“检测到 PSRAM 就停机”的检查。
默认验证构建仍使用 PSRAM Disabled；如启用 N16R8 的 OPI PSRAM，应另行上板验证。

驱动电源由新图 +12V 稳压轨供电，`POWER_SUPPLY_VOLTAGE` 由 15 改为 12V，
`FOC_VOLTAGE_LIMIT_MAX` 由 14 改为 12V。AD12 仍是 56kΩ/10kΩ 分压，监测 +12V，
不是入口 +24V。若实际修改了电源反馈电阻，应按实测同步更新配置。

## nSLEEP 与启动

ENA/ENB 外部电阻改为下拉：**LOW=休眠，HIGH=唤醒，极性没有反转**。
两只 BLDCDriver3PWM 的 enable 引脚现在关联 nSLEEP，SimpleFOC disable 会同时拉低驱动。
先保持 nSLEEP LOW，再配置零占空 PWM；需要电机初始化/校准时由 SimpleFOC 唤醒。
校准结束后立即休眠，A 校准完成后也不会在等待 B 校准期间持续输出。
通信初始化完成后，成功初始化的轴恢复当前位置保持，延续原有自动运行意图。
初始化失败的轴保持休眠，WAKE 返回失败；排除硬件问题后重启重新校准，不对未完成初始化的
驱动或电流采样对象直接调用 initFOC。

零位和 PID 的 A/B NVS 键保持兼容，旧 C 数据不再读取或使用。旧三轴 coord_mask 只取 A/B 位。
串口速度循环脚本也已改为仅控制 A/B。

## CAN 的已知依据与假设

参考项目 `devices/upper_foc_board.py` 是旧三轴 **ASCII 串口**后端；
`devices/DM_CAN.py` 是达妙电机协议，经专有 USB 转 CAN 串口封装发送。921600 是该适配器
串口速率，不能当作 CAN 总线速率。参考项目没有定义 UpperFoc 的 CAN ID 或文本封装。
`motor-control-routine/Python_demo/u2canfd/damiao.py` 示例使用 1Mbps 仲裁段、5Mbps 数据段。

本固件据此选择 **可配置的经典 CAN 1Mbps** 作为默认假设，新增 UpperFoc 文本封装；
这不表示已确认现场总线速率，也不宣称兼容达妙 MIT 报文。
ESP32-S3 内置 TWAI 不支持 CAN-FD，不能直接接入正在发送 FD 帧的同一总线。
依据：[Espressif ESP32-S3 TWAI 文档](https://docs.espressif.com/projects/esp-idf/en/v5.4.2/esp32s3/api-reference/peripherals/twai.html)。

`Config.h` 中可配置：

| 参数 | 默认值 | 含义 |
|---|---|---|
| ENABLE_CAN | 1 | 启用原生 TWAI + SIT1051 |
| CAN_BITRATE | 1000000 | 支持 1000000 / 500000 / 250000 |
| CAN_COMMAND_ID | 0x590 | 主机发给本板，11 位标准数据帧 |
| CAN_REPLY_ID | 0x591 | 本板回复和广播，11 位标准数据帧 |
| CAN_FRAGMENT_TIMEOUT_MS | 100 | 接收分片最大间隔 |
| CAN_TX_TIMEOUT_MS | 500 | 每条待发送消息的入队/发送等待上限 |

0x590/0x591 是本次新增的本地约定，需要与主机和现场其他节点统一，**不是**从参考工程
提取出的 ID。文本 `HELLO 5090` 不变；十进制 5090 超出标准 CAN ID 范围，不能直接作为 ID。
仅允许一个主机向本板连续发送分片；一条命令发完再发下一条。

## CAN 分片格式

每个 CAN 帧 DLC 为 3～8，最多携带 6 个 UTF-8 文本字节。

| 字节 | 含义 |
|---|---|
| data[0] bit7 | START：新消息首片 |
| data[0] bit6 | END：本消息末片 |
| data[0] bit5..0 | 序号：首片为 0，随后递增，模 64 |
| data[1] | 消息号，主机每条命令递增（模 256） |
| data[2..DLC-1] | 文本数据，无 NUL、无填充；无需 CR/LF |

单帧 `STAT?`，消息号 1：`C0 01 53 54 41 54 3F`，DLC=7，CAN ID=0x590。
`PA1.234`，消息号 2：第一帧 `80 02 50 41 31 2E 32 33`，第二帧 `41 02 34`。
本板把完整文本交给与 USB/BLE/WiFi 相同的 `handleCommand(..., 3)`。

回复使用同样格式，消息号由本板独立生成，不回显请求消息号；回复与广播共享 0x591，
主机需要解析文本识别响应。命令最多 192 字节，回复最多 2048 字节。
重复/缺失/乱序分片、消息号不一致、NUL、溢出和超时都会丢弃正在拼接的命令，
后续必须从新 START 开始。完整命令的重复发送不做去重，因此不要自动重试 D 位移命令。
单帧丢失后不会执行收到的命令后缀。

收发均使用零等待 TWAI 调用，每次主循环最多读取 4 帧、执行 1 条 CAN 命令并提交 1 帧。
TX 应用队列最多 8 条消息，满队列/超时会丢弃消息，可通过 USB 的 `COM?` 查看统计。
CAN bus-off 时清理待发送消息与接收状态并启动恢复；恢复后不重放应用队列。
CAN 故障不自动改变电机目标，运动超时停车策略尚未定义。
通讯栈非阻塞并不意味着 NVS 写入、WAKE 延时等命令本身是硬实时操作。

主机示例工具 `upper_can.py` 使用 python-can：

```powershell
python -m pip install python-can
python upper_can.py --interface slcan --channel COM21 'STAT?'
```

按实际适配器修改 interface/channel；示例不会自动打开或控制本地硬件。
达妙专有 USB 转 CAN 串口协议不是 slcan，需另外实现其适配层；参考项目仍使用旧串口后端，
本次没有修改外部 `cool_robot_arm` 项目。接线为 CANH/CANL/GND；仅在总线端点闭合
SW2 接入板载 R13=120Ω 终端电阻。

## 命令与其他修正

支持原 A/B 命令：PA/PB、VA/VB、DA/DB、TA/TB，单位分别为 rad、rad/s、rad、A（Iq）。
SLEEP/WAKE、ZERO!/ZERO?、ALL0、VPID/PPID、VF/IF 及状态查询均只作用于 A/B。
STATA?/STATB?、ENC?、CUR?、COM?、FAULT?、LIMIT? 保留，C 命令返回错误。
所有通道拒绝非法数值、NaN、Inf，PID 拒绝负增益；USB/WiFi 接收长度受限且不会执行溢出后缀。
BLE 回调通过 FreeRTOS 队列把命令交给主循环，避免与 FOC 并发修改电机对象。
WiFi OFF=0、ROUTER=1，修正原来两个宏同为 1 的错误，广播设备名统一 Upper3.0-FOC。

原有 `ENABLE_FAULT_CHECK=0` 保留，当前 nFAULT/电压保护默认未启用。
原有单圈位置及绕零运行策略保留；这些不是机械行程限位。

## 验证

最终默认配置已完整编译通过（退出码 0）：程序占用 798557 / 3145728 字节（25%），
全局变量占用 36220 / 327680 字节（11%）。C++ CAN 分片回归及 6 项 Python 测试通过，
`git diff --check` 通过。编译日志为 `tmp/build-final.log`，固件位于 `tmp/build/`。

使用本地 Arduino-ESP32 3.3.10 和本仓库修改版 SimpleFOC（不能换成未含本项目 API 的
其他副本）。构建命令：

```powershell
& 'F:/ARM/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe' compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled' --libraries ./libraries --build-path ./tmp/build .
g++ -std=c++11 -Wall -Wextra -Werror tests/can_transport_test.cpp -o tmp/can_transport_test.exe
./tmp/can_transport_test.exe
python -m unittest discover -s tests -p 'test_*.py'
```

C++ 回归覆盖正常分片、重复末片、丢片、跨消息拼接、坏 DLC、NUL、溢出、超时计时回绕和
长消息序号回绕。Python 回归覆盖主机编码/解码及引脚映射和 C 轴清理。
尚未烧录或连接实际电机/CAN 总线；电流零偏、编码器方向、PWM、nSLEEP 复位电平、CAN
收发与 bus-off 恢复仍需上板测量。
