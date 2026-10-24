# ArmPi Ultra 上位机控制软件开发

Windows 上位机（浏览器控制页 + Python 串口服务）开发目录。控制对象为幻尔（Hiwonder）
STM32F103RB 机械臂控制板，6 个总线舵机。

本仓库只放上位机代码与文档。STM32 固件源码在仓库外，路径见下。

## 参考资料位置

固件工程：`树莓派对应机械臂控制/ArmPiUltra/ArmPiUltra/`

| 文件 | 用途 |
| --- | --- |
| `Hiwonder/Misc/packet.c` / `packet.h` | AA55 帧协议状态机、功能号枚举 |
| `Hiwonder/Misc/checksum.c` | CRC8 表（反射多项式 0x8C） |
| `Hiwonder/System/packet_handle.c` | 上位机指令分发（子命令 → 舵机动作） |
| `Hiwonder/System/packet_reports.h` | 回包结构体 |
| `Hiwonder/Portings/Src/packet_porting.c` | 协议绑到 USART1 |
| `Hiwonder/Arm/src/robot_arm.c` | 机械臂应用层、角度↔位置换算 |
| `Hiwonder/Arm/Inc/kinematics.h` | 连杆长度、关节限位 |
| `Hiwonder/Peripherals/Inc/serial_servo.h` | 总线舵机指令码（板内私有，上位机不直接用） |
| `MDK-ARM/ArmPiUltra.hex` | 已编译固件，说明本机 Keil 环境可用 |

## 硬件链路

上位机 ↔ **USART1**（PA9 TX / PA10 RX）**@ 1,000,000 bps，8N1**

- USART2（PA2/PA3 + PA1 BUS_EN 方向控制）@ 115200：板内半双工总线舵机，上位机不碰。
- USART3 @ 9600、I2C1：固件里初始化了但没有任何代码使用，闲置。
- SPI1 + PB14 CS：W25Q64 外部 Flash，存动作组与舵机偏差。
- ADC1 CH13（PC3）：电池电压。TIM4 CH3（PB8）：蜂鸣器。LED PB9 / PB15。
- 烧录口：SWD（PA13/PA14）。
- `global.h` 里的 `CONTROL_MODE`（PC/蓝牙）和 `SERVO_TYPE` 是编译期宏，**全工程无引用，不影响行为**。

注意：1 Mbps 对 USB 转串口芯片有要求，CH340 / CH9102 可以，老 CP2102 / PL2303 容易乱码。

## 通信协议

### 帧格式

```
0xAA 0x55 | function | length | data[0..length-1] | crc8
```

- CRC-8，**反射多项式 0x8C，初值 0x00，无输出异或**（查表法表头 `0,94,188,226,...` 已核对）。
- 计算范围 = `function + length + data`，共 `length + 2` 字节，**不含帧头**。
- `length` 为 data 段字节数，0~255。
- 功能号必须 `< 14`（`PACKET_FUNC_NONE`），否则状态机丢弃整帧。

### 已验证样例

让 6 号舵机（底座）在 1000 ms 内运动到位置 500：

```
AA 55 05 07 01 E8 03 01 06 F4 01 8F
帧头      功能 长度 子命令 时间=1000ms  ID=6  位置=500  CRC=0x8F
```

### 功能号（`enum PACKET_FUNCTION`）

`0 SYS` `1 LED` `2 BUZZER` `3 MOTOR` `4 PWM_SERVO` `5 BUS_SERVO` `6 KEY` `7 IMU`
`8 GAMEPAD` `9 SBUS` `10 OLED` `11 RGB` `12 CONVEYOR` `13 STEPPER_MOTOR`

固件**实际注册的分发器只有 4 个**：`0 SYS`（改电池报警阈值）、`1 LED`、`2 BUZZER`、
`5 BUS_SERVO`。其余（含 PWM 舵机、RGB）在 `packet_handle.c` 里被注释掉了。

### BUS_SERVO 子命令（功能号 5，本项目主用）

data 段第 1 字节恒为子命令。

| 子命令 | 含义 | data 段布局 | 是否回包 |
| --- | --- | --- | --- |
| `0x01` | 设置位置（多舵机） | `01 \| duration u16LE \| servo_num \| (id, position i16LE) × n` | 否 |
| `0x03` | 停止舵机 | `03 \| servo_num \| id × n` | 否 |
| `0x05` | 读位置 | `05 \| servo_id` | 是，5 字节 |
| `0x07` | 读输入电压 | `07 \| servo_id` | 是，5 字节 |
| `0x09` | 读温度 | `09 \| servo_id` | 是，4 字节 |
| `0x0B` / `0x0C` | 卸力 / 加载 | `0B 或 0C \| servo_id` | 否 |
| `0x0D` | 读加载状态 | `0D \| servo_id` | 是，4 字节 |
| `0x10` / `0x12` | 写 ID / 读 ID | `10 \| servo_id \| new_id` | 读时 4 字节 |
| `0x20` / `0x22` / `0x24` | 设偏差 / 读偏差 / 保存偏差 | `20 \| servo_id \| dev i8` | 读时 4 字节 |
| `0x30` / `0x32` | 设 / 读角度限位 | `30 \| servo_id \| min u16LE \| max u16LE` | 读时 7 字节 |
| `0x34` / `0x36` | 设 / 读电压限位 | `34 \| servo_id \| min u16LE \| max u16LE` | 读时 7 字节 |
| `0x38` / `0x3A` | 设 / 读温度上限 | `38 \| servo_id \| limit u8` | 读时 4 字节 |

`position` 范围 0~1000（固件只裁上限，不裁下限），`duration` 0~30000 ms。

### 回包格式

同样是 AA55 帧，`function = 5`，data 段为：

```
servo_id | sub_command | success(i8) | args...
```

`success` 非 0 表示读取成功。位置/电压是 2 字节小端，温度与状态是 1 字节。

### 解析时的两个坑

1. **每帧尾部多 1 个垃圾字节**：`packet.c` 的 `transmit()` 写 `data_len + 6` 字节，实际帧长是
   `data_len + 5`，多发 1 个未初始化字节。上位机必须用帧头状态机解析，不能按定长切包。
2. **收到校验通过的帧会闪 PB15 的 LED**，可以当作"协议层已通"的硬件指示灯。

## 舵机映射

| ID | 关节 | 角度→位置 | 复位位置 |
| --- | --- | --- | --- |
| 1 | 夹爪 | `pos = 700 − 5.556 × 张开角(0~90°)`；700 全闭、200 全开；固件硬性限幅 ≤700 防堵转 | 226 |
| 2 | 腕旋转 | `pos = 500 − 4.167 × 角(−90~90°)`，方向相反 | 500 |
| 3 | 手腕俯仰 = knot3 | `pos = 500 + 4.167 × 角` | 110 |
| 4 | 小臂 = knot2 | 同上 | 800 |
| 5 | 大臂 = knot1 | `pos = 500 + 4.167 × (角 − 90°)`（结构补偿 90°） | 600 |
| 6 | 底座回转 = knot0 | `pos = 500 + 4.167 × 角` | 500 |

`SERIAL_ANGLE_FACTOR = 1000/240 ≈ 4.16667`，即 **位置 0~1000 对应 ±120°，1 单位 = 0.24°**。

逆解只涉及 knot0~3（ID 6/5/4/3）；腕旋和夹爪是独立角度控制。

## 坐标系与限位（`kinematics.h`）

- 连杆：`L1=2.89`，`L2=9.50`，`L3=9.5`，`L4=16.9`，**单位 cm**
  （`robot_arm.c` 注释写 mm 是错的，X 上限 20 只能是 cm）。
- 工作空间：X 10~20，Y −10~10，Z 0~25；默认点 `(15, 0, 2)`。
- 关节限位：knot3 ±120.2°，knot4 −200.2~20.2°，knot5 −180.2~0.2°，knot6 ±120.2°。
- 夹爪张开角 0~90°，腕旋转 −90~90°。

## 已知固件缺口

`robot_arm_coordinate_set()`（坐标控制）、`action_group_run()`（动作组）、
`robot_arm_emergency_stop()`（急停）、`fkine/ikine` 在固件里**没有任何调用者**——
编译进去了，但没挂到串口命令上。当前上位机只能逐关节发原始位置。

- **方案 A**：在 `packet_handle.c` 增加分发分支接上这些函数，Keil 重编译、SWD 烧录。
  限位/急停/动作组在板内执行，上位机掉线不影响安全。
- **方案 B**：不改固件，上位机自己做逆解，用 `0x01` 多舵机指令下发。零硬件风险，立即可跑。
  注意正逆解在闭源 `LeArm.lib` 里，需要按上面的连杆参数自行实现并与实机校准。

## 开发环境

- Python 3.11.9（`C:\Users\Administrator\AppData\Local\Programs\Python\Python311\python.exe`）
- 尚未安装 `pyserial`
- Git 2.52，仓库在本目录，尚未产生任何提交

## 约定

- 真实 COM 口、串口参数写进 `config.local.json`（已在 `.gitignore` 里排除），
  仓库只保留 `config.example.json`。
- 任何会让舵机通电运动的代码，必须先在文档里说明限位与急停路径，再实机测试。
