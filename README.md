# GELLO / CR3A / O6 Operator

新操作界面入口是 `scripts/run_operator.sh`。启动状态始终为 **OFFLINE**，
不自动连接硬件、开启相机、使能机器人或发送 ServoJ。
当前交付用于离线验收和后续真机联调；实际接口稳定性需要人工完成统一硬件验收，
不能由 pytest 结果代替。

## 项目结构

- `src/gello_cr/`：遥操作应用、设备适配、录制和界面代码。
- `vendor/`：随项目管理的设备 SDK 与旧版 FT300 驱动，见 [vendor 说明](vendor/README.md)。
- `tests/`：pytest 自动化回归测试，不是运行时驱动目录。
- `teleop_runtime.py`、`lerobot_recorder.py`：仍被新界面工厂使用的根目录兼容运行时。
- `TEST_INEXBOT.py`：旧版操作界面入口；新界面从 `scripts/run_operator.sh` 启动。

## 安装与启动

仓库自带 NRC SDK 是 Linux x86-64 / Python 3.12 版本。新建独立 Python 3.12
环境，安装应用依赖；原有硬件/训练环境可以继续使用，不需替换。

```bash
cd /path/to/gello_CR
python3.12 -m venv .venv
.venv/bin/python -m pip install -e '.[ui,dev]'
scripts/run_operator.sh --check-only
scripts/run_operator.sh
```

启动器默认使用项目内 `.venv/bin/python`；没有 `.venv` 时使用系统
`python3.12`。其他兼容环境可显式指定：

```bash
GELLO_CR_PYTHON=/path/to/python3.12 scripts/run_operator.sh --check-only
```

启动器管理应用源码路径和 Python 动态库目录（包含 venv 的 base_prefix），
无需手工设置 PYTHONPATH。NRC SDK 默认从仓库加载；如需使用其他 SDK，设置
`NRC_SDK_ROOT` 或配置 `robot.sdk_root`。

硬件环境另需 GELLO 的 `dynamixel-sdk`、O6 SDK 对应的 `pymodbus==3.5.1`、
D435 的 `pyrealsense2`。O6 源码位于 Git 子模块；新克隆仓库先执行
`git submodule update --init --recursive`。随项目携带的设备库集中在 `vendor/`：
GELLO、NRC、LinkerHand 子模块和旧版 Robotiq FT300 传感器驱动。这些驱动只在人工连接时使用，
离线测试不加载真实设备 SDK。

LeRobot 在 `dataset.worker_python` 指定的独立环境中运行。该环境需要提供
`LeRobotDataset.create`、
`RGBEncoderConfig` 和 streaming encoding；本地实际写盘已用该版本验证。
迁移项目时需在配置中指定兼容的 LeRobot Python 环境；应用通过 worker 进程调用，
不导入 LeRobot 源码。也可通过 `GELLO_CR_DATA_PYTHON=/path/to/lerobot/python`
在启动时覆盖配置路径。不要仅凭同名 PyPI 包已安装就认定接口兼容。

可选 DAgger 部署代码来自独立 OpenPI 项目。使用时在启动前设置
`GELLO_CR_OPENPI_ROOT=/path/to/openpi_cr3_o6`；不使用 DAgger 无需安装 OpenPI。

## 配置与数据含义

启动后、连接设备前，可点击右上角 **⚙ 设置**，分组编辑机械臂速度与保护、
GELLO 参数、O6 动作库及开合选择、数采相机和控制器连接参数。
保存会备份原配置，**关闭并重启程序后生效**。参数保存在
`config/roarm_cr5_teleop.json`。

`gello.software_root` 和 `gello.kinematics_urdf` 使用相对配置文件的路径，
会解析到本项目内的 GELLO 副本。迁移到其他设备时仍需配置串口 by-id、相机序列号、
数据根目录和 `dataset.worker_python`。仓库已包含只读 Dynamixel 实现和 FK URDF。

当前配置为 joint 跟随，J1–J6 顺序不变，倍率 1.1；offsets/signs 原样保留。
`safety_max_command_speed_rad_s` 从 20 调整为 40；`servoj_vmax/amax/jmax` 从
80/195/120 调整为 100/200/180。V2 关节使用 rad，NRC 发送值转换为 deg。
安全参数是否适用于实际机械臂需在硬件验收前复核；测试通过不证明阈值适合现场。

- `observation.state`：18 维，6 个关节 rad + TCP xyz/rpy（m/rad）+ 6 个 O6 原始位置。
- `action`：12 维，TCP delta（m/rad）+ 6 个 O6 目标；不是绝对关节目标。
- Base RGB → `observation.images.base_0_rgb`。
- Wrist RGB → `observation.images.left_wrist_0_rgb`。
- Base ROI → `observation.images.right_wrist_0_rgb`（历史兼容键名，并非第二腕部相机）。
- 三路输出均为 224×224 RGB 视频。

每个 Episode 的 `meta/collection/episode_XXXXXX.json` 保留 outcome、task、
配置上下文、质量摘要、逐帧 host monotonic timestamp、camera age/skew 和流名称。
LeRobot `timestamp` 是按 fps 生成的 Episode 时间；真实采样时刻保存在 sidecar。
这些时间是主机接收时间，不是硬件同步曝光时间。

## 操作与退出

人工按 CONNECT → PREPARE_DEVICES → POWER_ON → START_TELEOP 推进；
录制前另行 START_CAMERAS。所有按钮和 F12 软件紧急停止通过 CommandPort /
ApplicationService，状态机和 readiness 在执行端继续校验。

POWER_ON 只使能，不启动跟随。FAULT/ESTOP 后停止输出；RESET 成功只回到
ROBOT_ENABLED，必须再次人工 START_TELEOP。F12 是软件停止，不能代替控制柜物理急停。

STOP_EPISODE 只停止采样并保留 pending frames；随后明确选择 SAVE_SUCCESS、
SAVE_FAILURE 或 DISCARD。相机 stale/skew 超限禁止开始 Episode；录制中的质量超限
作为 needs_review 元数据，不替代独立运动安全保护。

active/pending Episode 阻止关闭窗口，不会静默保存成功或丢弃。先处理 Episode
再退出。执行中的命令也需等待完成，F12 仍可用。正常退出停止新命令和运动、
处理机器人下电、断开控制器、关闭主设备/O6/相机、收尾录制 worker 和线程。
关闭失败会继续尝试其他资源并报告错误；窗口保留，允许重试未完成的关闭步骤。

## 离线测试与数据校验

```bash
.venv/bin/python -m pytest
.venv/bin/python -m compileall -q src teleop_runtime.py lerobot_recorder.py
 git diff --check
```

测试自动阻断 TCP connect、串口打开和真实 NRC/RealSense/Dynamixel SDK 导入；
协议测试使用本地 Unix socketpair。GUI 测试使用 offscreen Qt。
完整数据测试环境还需 `.[data-validation]` 和兼容的 LeRobot；缺少这些依赖时对应
测试会显示 skipped，不能算完整数据验收通过。

数据校验是只读操作，需要已正常 finalize 的 **LeRobot v3** 数据集：

```bash
GELLO_CR_DATA_PYTHON=/path/to/lerobot/python \
  scripts/validate_dataset.sh '/path/to/finalized/dataset'
```

校验状态/动作维度与 NaN/Inf、task/metadata、逐帧时间、相机 age/skew、质量信息、
各类帧计数，并完整解码每一路视频检查大小、PTS 和 Episode 时间区间。
损坏或缺失数据会直接失败；needs_review Episode 会单独列出。

## 常见故障

- `NRC Python module missing`：目录必须直接包含 wrapper 和对应扩展。
- `libpython3.12.so.1.0 not found`：检查启动解释器及动态库路径；不是 PYTHONPATH 问题。
- CONNECT 失败：检查配置中的实际 IP（当前为 `192.168.2.14`）、6001/7000、控制柜状态和其他端口占用者。
- 主设备未 ready：检查 FTDI by-id、GELLO 反馈和 O6 RS485 通信；O6 hand_id=39 是 Modbus 地址。
- 相机 ERROR/stale：检查各自序列号、其他进程占用和新帧到达时间。
- 保存超时：保留目录和日志，先判断落盘状态，不要反复提交保存或自动丢弃。
- 退出报错：保留异常信息，确认未完成资源和机器人实际电源状态，再重试关闭。

Legacy 的大规模拆分和正式硬件验证版本仍以真机验收为前提；本轮只改必要的
生命周期边界，保留已验证 ServoJ 核心。
