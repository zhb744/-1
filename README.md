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

## 实机自检结果（2026-10-25，COM4）

`python tools/selfcheck.py --port COM4` 输出：**30 次探测全部 OK，0 CRC 失败，0 杂帧**。
三段链路（Windows ↔ USART1 @ 1 Mbps ↔ 舵机总线 @ 115200）确认打通，6 个舵机 ID 1~6 全部在线。

实机读数：

- 电压原始值 7725~7869，六个舵机高度一致 → 单位应是 **mV**（约 7.8 V 母线），仍需与电池 ADC 对账。
- 温度 31~32 °C，全总线一致，合理。
- 加载状态全部为**卸力**（`load_unload = 0`）→ 舵机当前不带保持力矩，位置会随手漂移；
  开始做运动控制前必须逐条下发 `0x0C` 加载，或确认这是有意为之。
- 复位位置（226/500/110/800/600/500）与实机关节位置不一致，说明臂停在自定义姿态上。

### 端口号会漂

同一块 CH340 板子，本次会话里先后出现过 **COM9 和 COM4**。CH340 的 COM 号由 Windows 按
"USB 序列号 + 上次占用"分配，换 USB 口或换线就可能变。**每次开机先跑 `--list`，别照搬旧号码**。

### 错位回包（实机踩到的坑，已修）

固件 `transmit()` 每帧尾部多写 1 个未初始化字节。它偶尔正好是 `0xAA`，于是会把下一帧的真帧头
吞掉，导致之后**所有回包整体错位一帧** —— 第一次实机自检就出现了"请求舵机 4 的 ID，回包自称 3"
"温度读到 127 °C"这类假数据。

修法（`tools/selfcheck.py` 的 `exchange()` 已实现，控制层必须沿用）：

1. 每条请求前先 `reset_input_buffer()` 清空系统接收缓冲，并 `parser.resync()` 复位状态机；
2. 收到的回包必须核对 **功能号 + 子命令 + 舵机 ID** 三项都对得上才算本次结果，
   对不上就丢弃计数并继续等，绝不能"读到一帧就用"。

## 仿真资源与 URDF 现状

`树莓派对应机械臂控制/3 功能包文件/simulations/simulations/` 是 **ROS 2（Humble）功能包三件套**：

| 包 | 内容 |
| --- | --- |
| `armpi_ultra_description` | `armpi_ultra.urdf` / xacro + 17 个 STL 网格 + RViz 配置 |
| `armpi_ultra_moveit_config` | MoveIt Setup Assistant 生成的 SRDF、joint_limits、kinematics、ros2_control |
| `robot_gazebo` | Gazebo/Ignition 世界与 xacro |

URDF 直接照搬去仿真会出问题，已核对出四处必须先处理：

1. **`joint1`（底座回转）被导出成 `type="fixed"`**，但它对应真机舵机 ID 6。不改回 `revolute`，
   仿真里底座根本转不动。`r_joint`、`gripper_joint` 等也是 fixed。
2. **网格路径全是 `package://44 001/meshes/...`**，而包名是 `armpi_ultra_description`，
   SolidWorks 导出器把零件名（带空格）写进了路径 → RViz/Gazebo 里模型是空的，需要批量替换。
3. **夹爪在 URDF 里是 4 个独立关节**（`l_joint` / `l_in_joint` / `r_in_joint` / `l_out_joint`），
   真机只有 1 个舵机 ID 1，仿真必须加 mimic 联动，否则拖动结果与真机不一致。
4. **限位三处不同源**：URDF 每个关节都写 ±2.09 rad（±119.75°）、固件 `kinematics.h` 是
   非对称的 −200.2~20.2 等值、`global.h` 的复位位置又与两者对不上。桥接层目前统一用
   位置全域 ±120°，标定表留 `config.local.json` 的 `calibration.servo_angle_limits`，
   `tests/test_servos.py::test_three_limit_sources_disagree_and_need_calibration` 专门盯着这个矛盾。

关节对应关系（由轴向判断，已写进 `armcomm/servos.py`）：
`joint1→ID6 底座`、`joint2→ID5 大臂`、`joint3→ID4 小臂`、`joint4→ID3 腕俯仰`、
`joint5→ID2 腕旋转`、`l_joint 等 4 个→ID1 夹爪`。

## 代码结构

```
armcomm/protocol.py   AA55 帧编解码、CRC-8、回包解码、只读安全闸门
armcomm/servos.py     舵机 ID/URDF 关节映射、角度↔位置换算、限位
tools/selfcheck.py    只读串口自检（不发任何运动指令）
tests/                106 条单元测试，含固件 CRC 表逐项核对与黄金帧
config.local.json     真实 COM 口等本地配置（已 gitignore）
config.example.json   配置模板
```

## 仿真页面怎么用

```
python tools/serve.py --com COM4        # 启动，默认 http://127.0.0.1:8300/
python tools/selfcheck.py --list        # 端口不确定时先查
```

浏览器打开后：

- **默认是纯仿真**，怎么拖都不会动真机。要在"真机连接"里选端口并点"开启联动"，
  服务端才会独占那个 COM 口；关掉或断开立刻回到仿真。
- 六个滑块就是六个舵机（1 夹爪 / 2 腕旋转 / 3~6 主臂），单位是**舵机角度**，
  服务端按 `armcomm/servos.py` 换算成 0~1000 位置，六个关节永远打包在**同一帧** `0x01`
  里同步下发。
- "读取真机姿态"会把实机位置回读成角度并同步到滑块，仿真从真机当前姿态开始，
  避免视觉位置和实际位置对不上。
- "急停"发子命令 `0x03`；"加载力矩/卸力"发 `0x0C`/`0x0B`。卸力状态下臂不带保持力矩，
  会随手漂移，此时回读到的角度只代表那一瞬间。
- 调试：控制台里 `__armSim.state` 能看到模型、关节句柄与当前目标角度。

前端依赖（three.js r128 + STLLoader + OrbitControls）已固化在 `web/vendor/`，
页面不依赖外网 CDN —— 本机网络只放通 `api.github.com`，需要更新时用
`python tools/vendor_web_deps.py` 重新拉取。

坐标系提醒：URDF 是 Z 轴朝上，three.js 是 Y 轴朝上，机器人挂在
`web/index.html` 里一个 `rotation.x = -90°` 的组上；改渲染时别把这个翻转去掉，
否则模型会"躺着"长出来。

## 开发环境

- Python 3.11.9（`C:\Users\Administrator\AppData\Local\Programs\Python\Python311\python.exe`）
- 尚未安装 `pyserial`
- Git 2.52，仓库在本目录，尚未产生任何提交

## 约定

- 真实 COM 口、串口参数写进 `config.local.json`（已在 `.gitignore` 里排除），
  仓库只保留 `config.example.json`。
- 任何会让舵机通电运动的代码，必须先在文档里说明限位与急停路径，再实机测试。
