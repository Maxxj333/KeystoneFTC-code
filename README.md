# KeystoneFTC-code

[中文](#中文) | [English](#english)

FTC 队伍 **#37502**（北京市鼎石学校高中部 / Keystone Academy）机器人源代码。
FTC Team **#37502** (Keystone Academy, Beijing) robot source code.


---

## 中文

### 概览

- 底盘：四电机麦克纳姆底盘（`DcMotorEx` × 4）+ IMU 姿态 + 里程计
- 视觉/定位：Limelight 3A（AprilTag）+ 里程计 + EKF 融合，带场地 Y 轴翻转修正
- 机构：双电机 Shooter + Feeder、Intake、双 Servo
- 自主：轨迹跟踪（`path_*.txt`）+ `events.txt` 脚本 DSL（顺序/并行/竞速/视觉条件分支）
- 遥控：场地中心 / 机器人中心切换、Shooter、Intake、Servo 控制

### 目录结构

```text
KeystoneFTC-code/
├── README.md                 # 本文件（中英双语）
├── LICENSE
└── teamcode/                 # 复制到 FTC SDK TeamCode 模块 / OnBot 即可使用
    ├── RedAuto.java          # Autonomous: "Red Auto"
    ├── RedTeleop.java        # TeleOp: "Red TeleOp"
    ├── config/
    │   └── RobotConfig.java  # 全部硬件名、几何、PID、Shooter、视觉、EKF 参数集中配置
    ├── subsystems/           # 子系统（每个机构一个类）
    │   ├── ShooterSubsystem.java
    │   ├── IntakeSubsystem.java
    │   └── ServoSubsystem.java
    ├── commands/             # 自主命令系统
    │   ├── AutoCommand.java      # 命令基类
    │   ├── CommandGroups.java    # SEQ / PAR / RACE 组合
    │   ├── CommandParser.java    # 解析 events.txt
    │   ├── PathCommand.java      # 轨迹插值与目标下发
    │   ├── ConditionalVision.java# IF_VISION 条件分支
    │   ├── TagFilter.java        # AprilTag ID 过滤（支持 34-37 等写法）
    │   ├── VisionSource.java     # 视觉数据源接口
    │   └── OtherCommands.java    # WAIT / SHOOT / INTAKE / SERVO 等
    ├── RED_1/                # 路径批次 1：events.txt + path_01~04.txt
    └── RED_2/                # 路径批次 2（默认，DEFAULT_PATH_BATCH）
```

> **为什么 `RED_1/`、`RED_2/` 放在 `teamcode/` 根目录下？**
> `RedAuto` 在机器人上运行时会扫描 `TEAMCODE_DIR`
> （`/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/`）的直属子目录来找批次，
> 所以批次文件夹必须直接放在 `teamcode/` 下面，不能再套一层目录。

### 硬件配置（`config/RobotConfig.java`）

| 名称 | 用途 |
|---|---|
| `FL` / `FR` / `BL` / `BR` | 四个底盘电机 |
| `imu` | IMU（REV Hub，Logo 向下） |
| `limelight-dolphin` | Limelight 3A，pipeline 0 |
| `ShooterRight` / `ShooterLeft` | Shooter 双电机 |
| `Feeder` | 送料电机 |
| `Intake` | 进料电机 |
| `LeftServo` / `RightServo` | 两个舵机 |

几何、PID、Shooter 转速、视觉融合、翻转修正等参数全部集中在 `RobotConfig.java`，调参改这里即可。

### OpMode

| OpMode | 文件 | 说明 |
|---|---|---|
| **Red Auto** | `RedAuto.java` | 自主：启动时用手柄 D-pad ↑/↓ 选择路径批次，按 START 开始；执行 `events.txt`，无 `events.txt` 时自动按顺序跑所有 `path_*.txt` |
| **Red TeleOp** | `RedTeleop.java` | 遥控，见下表 |

#### TeleOp 操作（Gamepad 1）

| 操作 | 按键 |
|---|---|
| 移动 | 左摇杆 |
| 旋转 | 右摇杆 X |
| 场地中心 / 机器人中心切换 | `X` / `Y` |
| Shooter 送料 | 右扳机 `RT` |
| Intake（斜坡升速） | 右保险杠 `RB` |
| Servo 切到 ACTIVE / 回到 DEFAULT | `A` / `B` |

### 自主脚本 `events.txt`

`events.txt` 是缩进敏感的文本 DSL，`#` 或 `//` 为注释。常用关键字：

```text
SHOOT 3.0                 # 射击（参数为送料时长，秒）
PATH path_01              # 跟踪 path_01.txt
WAIT 3.0                  # 等待
INTAKE / INTAKE 3.0       # 进料（可选时长）
SERVO_ACTIVE / SERVO_DEFAULT

SEQ                       # 顺序执行子命令
PAR                       # 并行执行子命令
RACE                      # 竞速：先完成者结束，其余取消

IF_VISION 34-37           # 视野中出现任一 Tag 即走 THEN，否则超时走 ELSE
  THEN
    ...
  ELSE
    ...
```

示例（`RED_1/events.txt`）：

```text
SHOOT 3.0
PATH path_01
SERVO_ACTIVE

RACE
  SEQ
    PATH path_02
    WAIT 3.0
  INTAKE

IF_VISION 34-37
  THEN
    PAR
      PATH path_03
      SERVO_DEFAULT
  ELSE
    PAR
      PATH path_04
      SERVO_DEFAULT

SHOOT 3.0
```

`path_*.txt` 为 CSV 轨迹文件，列为
`Time_s, X_m, Y_m, Theta_rad, Vx_m_s, Vy_m_s, Omega_rad_s, Ax_m_s2, Ay_m_s2, Alpha_rad_s2, Jx_m_s3, Jy_m_s3, Jalpha_rad_s3`。

### 部署

1. 将本仓库的 `teamcode/` 整个文件夹复制到 FTC SDK 的 TeamCode 模块：
   `.../java/src/org/firstinspires/ftc/teamcode/`（或 OnBot Java 的对应目录）。
2. 确保 `RED_1/`、`RED_2/` 直接位于 `teamcode/` 下。
3. 在 FTC App 中 Build 并安装，运行 **Red Auto** 或 **Red TeleOp**。

---

## English

### Overview

- Drive: 4-motor mecanum chassis (`DcMotorEx` × 4) + IMU + odometry
- Localization: Limelight 3A (AprilTags) + odometry + EKF fusion, with field Y-flip correction
- Mechanisms: dual-motor shooter + feeder, intake, dual servos
- Autonomous: trajectory following (`path_*.txt`) + an `events.txt` script DSL (sequential / parallel / race / vision-conditional branching)
- TeleOp: field-/robot-centric drive switching, shooter, intake, and servo control

### Repository Layout

```text
KeystoneFTC-code/
├── README.md                 # This file (bilingual)
├── LICENSE
└── teamcode/                 # Copy this folder into the FTC SDK TeamCode module / OnBot
    ├── RedAuto.java          # Autonomous: "Red Auto"
    ├── RedTeleop.java        # TeleOp: "Red TeleOp"
    ├── config/
    │   └── RobotConfig.java  # All hardware names, geometry, PID, shooter, vision, and EKF params
    ├── subsystems/           # One class per mechanism
    │   ├── ShooterSubsystem.java
    │   ├── IntakeSubsystem.java
    │   └── ServoSubsystem.java
    ├── commands/             # Autonomous command system
    │   ├── AutoCommand.java      # Command base class
    │   ├── CommandGroups.java    # SEQ / PAR / RACE composition
    │   ├── CommandParser.java    # Parses events.txt
    │   ├── PathCommand.java      # Trajectory interpolation & target sink
    │   ├── ConditionalVision.java# IF_VISION conditional branch
    │   ├── TagFilter.java        # AprilTag ID filter (supports e.g. 34-37)
    │   ├── VisionSource.java     # Vision source interface
    │   └── OtherCommands.java    # WAIT / SHOOT / INTAKE / SERVO, etc.
    ├── RED_1/                # Path batch 1: events.txt + path_01~04.txt
    └── RED_2/                # Path batch 2 (default, DEFAULT_PATH_BATCH)
```

> **Why do `RED_1/` and `RED_2/` sit directly under `teamcode/`?**
> At runtime, `RedAuto` scans the direct subdirectories of `TEAMCODE_DIR`
> (`/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/`) to discover batches,
> so batch folders must be placed directly under `teamcode/`, with no extra nesting.

### Hardware Configuration (`config/RobotConfig.java`)

| Name | Purpose |
|---|---|
| `FL` / `FR` / `BL` / `BR` | Four drive motors |
| `imu` | IMU (REV Hub, logo facing down) |
| `limelight-dolphin` | Limelight 3A, pipeline 0 |
| `ShooterRight` / `ShooterLeft` | Dual shooter motors |
| `Feeder` | Feeder motor |
| `Intake` | Intake motor |
| `LeftServo` / `RightServo` | Two servos |

Geometry, PID, shooter RPM, vision fusion, and Y-flip correction parameters are all centralized in `RobotConfig.java`.

### OpModes

| OpMode | File | Description |
|---|---|---|
| **Red Auto** | `RedAuto.java` | Autonomous: select a path batch with D-pad up/down before START; runs `events.txt`, or all `path_*.txt` in sequence if no `events.txt` exists |
| **Red TeleOp** | `RedTeleop.java` | TeleOp, see below |

#### TeleOp Controls (Gamepad 1)

| Action | Button |
|---|---|
| Translate | Left stick |
| Rotate | Right stick X |
| Field-centric / robot-centric | `X` / `Y` |
| Shooter feed | Right trigger (`RT`) |
| Intake (ramped) | Right bumper (`RB`) |
| Servo to ACTIVE / back to DEFAULT | `A` / `B` |

### Autonomous Script `events.txt`

`events.txt` is an indentation-sensitive text DSL; `#` or `//` starts a comment. Common keywords:

```text
SHOOT 3.0                 # Shoot (arg = feed duration in seconds)
PATH path_01              # Follow path_01.txt
WAIT 3.0                  # Wait
INTAKE / INTAKE 3.0       # Intake (optional duration)
SERVO_ACTIVE / SERVO_DEFAULT

SEQ                       # Sequential
PAR                       # Parallel
RACE                      # Race: first to finish ends the group

IF_VISION 34-37           # If any listed tag is visible -> THEN, else timeout -> ELSE
  THEN
    ...
  ELSE
    ...
```

Example (`RED_1/events.txt`):

```text
SHOOT 3.0
PATH path_01
SERVO_ACTIVE

RACE
  SEQ
    PATH path_02
    WAIT 3.0
  INTAKE

IF_VISION 34-37
  THEN
    PAR
      PATH path_03
      SERVO_DEFAULT
  ELSE
    PAR
      PATH path_04
      SERVO_DEFAULT

SHOOT 3.0
```

`path_*.txt` files are CSV trajectories with columns
`Time_s, X_m, Y_m, Theta_rad, Vx_m_s, Vy_m_s, Omega_rad_s, Ax_m_s2, Ay_m_s2, Alpha_rad_s2, Jx_m_s3, Jy_m_s3, Jalpha_rad_s3`.

### Deployment

1. Copy the `teamcode/` folder from this repository into the FTC SDK TeamCode module:
   `.../java/src/org/firstinspires/ftc/teamcode/` (or the equivalent OnBot Java directory).
2. Make sure `RED_1/` and `RED_2/` are directly under `teamcode/`.
3. Build and install from the FTC App, then run **Red Auto** or **Red TeleOp**.
