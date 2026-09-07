# GELLO / CR3A / O6 Operator

新操作界面入口是 `scripts/run_operator.sh`。启动状态始终为 **OFFLINE**，
不自动连接硬件、开启相机、使能机器人或发送 ServoJ。
当前交付用于离线验收和后续真机联调；实际接口稳定性需要完成
[统一硬件验收表](docs/HARDWARE_ACCEPTANCE.md)，不能由 pytest 结果代替。

## 安装与启动

仓库自带 NRC SDK 是 Linux x86-64 / Python 3.12 版本。新建独立 Python 3.12
环境，安装应用依赖；原有硬件/训练环境可以继续使用，不需替换。

```bash
cd ~/cyf/gello_CR
python3.12 -m venv .venv
.venv/bin/python -m pip install -e '.[ui,dev]'
GELLO_CR_PYTHON="$PWD/.venv/bin/python" scripts/run_operator.sh --check-only
GELLO_CR_PYTHON="$PWD/.venv/bin/python" scripts/run_operator.sh
```

本机已有解释器可用于纯启动检查：

```bash
GELLO_CR_PYTHON=/home/ace/cyf/TEST_PY12/bin/python3.12 scripts/run_operator.sh --check-only
```

启动器管理应用源码路径和 Python 动态库目录（包含 venv 的 base_prefix），
无需手工设置 PYTHONPATH。SDK 目录优先级：`NRC_SDK_ROOT` 环境变量 >
配置 `robot.sdk_root` > 仓库 `TESTRobot_INEXBOT`。
详见 [NRC 加载与动态库排错](docs/NRC_SDK_LOADING.md)。

硬件环境另需 GELLO 的 `dynamixel-sdk`、O6 SDK 对应的 `pymodbus==3.5.1`、
D435 的 `pyrealsense2`。O6 源码位于 Git 子模块；新克隆仓库先执行
`git submodule update --init --recursive`。这些驱动只在人工连接时使用，
离线测试不加载真实设备 SDK。

LeRobot 在 `dataset.worker_python` 指定的独立环境中运行。
本机配置为 `/home/ace/miniconda3/envs/lerobot/bin/python`，其安装源码在
`/home/ace/lerobot`。所需接口包括 `LeRobotDataset.create`、
`RGBEncoderConfig` 和 streaming encoding；本地实际写盘已用该版本验证。
不要仅凭同名 PyPI 包已安装就认定接口兼容。

## 配置与数据含义

配置文件：`config/roarm_cr5_teleop.json`。本轮没有修改机器人参数或标定。
部署到其他目录前检查 `gello.software_root`、`gello.kinematics_urdf`、串口 by-id、
相机序列号、数据根目录和 worker_python。当前两个 GELLO 资源路径仍指向原工作区，
应在迁移机器时明确指向实际安装目录；仓库已包含只读 Dynamixel 实现和 FK URDF。

当前配置为 joint 跟随，J1–J6 顺序不变，倍率 1.1；offsets/signs 原样保留。
`servoj_vmax/amax/jmax = 80/195/120`，没有在本轮调参。
V2 关节使用 rad，NRC 发送值转换为 deg。首次目标、速度、步长和跟踪误差保护
继续使用现有逻辑。现有安全阈值也原样保留，其是否适用于实际机械臂需在硬件
验收前复核；测试通过不证明阈值适合现场。

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
GELLO_CR_DATA_PYTHON=/home/ace/miniconda3/envs/lerobot/bin/python \
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
