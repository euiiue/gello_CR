# DOBOT CR5 + RoArm-M2-Pro / Inverse3 + LinkerHand O6 项目交接文档

更新时间：2026-08-20

本文用于把当前实机、Qt 遥操作、LeRobot 数据采集与 OpenPI 部署工作的边界一次交代清楚。它是背景和状态交接，不代替操作前的现场安全检查。

## 1. 当前目标

系统最终目标是：

1. 可选择 RoArm-M2-Pro 或 Haply Inverse3 + VerseGrip 主臂遥操作一台 CR5 从臂和 LinkerHand 右手 O6；
2. 在 Qt 上位机内直接采集适配 PI0.5/OpenPI 的 LeRobot Episode，不再以 ROSBag 作为主采集链路；
3. 使用采集数据完成 PI0.5 微调；
4. 用独立、可迁移的 OpenPI 硬件接口部署模型。

当前阶段仍应视为“遥操作和数据采集软件已实现，正在进行实机验证和数据质量验证”，不能视为已经完成训练或部署。

## 2. 实机组成

### 2.1 CR5 从臂

- 机械臂：DOBOT CR5。
- 当前实际控制入口：纳博特/NRC 控制柜及其 `nrc_interface` SDK，不与 Dobot TCP SDK 混用。
- 控制柜地址：`192.168.1.245`（当前现场 CR3 配置）。
- 6001：命令、状态、MoveJ、逆解等接口。
- 7000：ServoJ 跟踪接口。
- 当前单机器人编号：`robotNum = 1`。
- 数据集只使用 CR5 J1～J6；NRC 某些接口返回的第七个数不作为第七个 CR5 关节训练。

CR5 伺服状态含义：

- `0`：停止；
- `1`：就绪；
- `2`：报警；
- `3`：运行。

Qt 运行时的伺服状态读取、清错、上/下使能和运行状态查询已经统一使用 `*_robot(..., robotNum, ...)`，目标为机器人 1。

### 2.2 RoArm-M2-Pro 主臂

- 控制板：Waveshare，USB 串口直连 Ubuntu 主机。
- 共 5 个物理舵机，但用于 4 个逻辑机械自由度加一个开合输入：
  - M1：底座旋转，ST3235；
  - M2、M3：两个 ST3235 镜像联动，共同构成一个连杆俯仰自由度；
  - M4：ST3235，末端连杆俯仰；
  - M5：ST3215HS，作为灵巧手开合输入。
- 正常遥操作时 M1～M4 可徒手拖动；主动回示教点时临时上力，到位后的具体保持/恢复行为由当前操作状态决定。

### 2.2.1 Haply Inverse3 主臂（新增）

- 接口：Haply Hub 内置 Inverse Service 3.5.x，WebSocket `ws://127.0.0.1:10001`；不直接抢占 Hub 管理的 `/dev/ttyACM*`。
- 通信只使用官方监控命令 `probe_position` 和 `probe_orientation`，不向 Inverse3 下发力、位置或关节力矩。
- 连接前要求 `in_use=false`、`control_mode=idle`，并逐帧检查 VerseGrip 的 ready/connected/awake/error 状态；其他 Haply 控制会话占用、手柄休眠或掉线都会快速失败。
- Inverse3 本体只提供球头中心 XYZ（m）；自由 RPY 必须来自有线或无线 VerseGrip 的 `orientation` 四元数。
- `body_orientation` 是 Inverse3 机身姿态，不能代替 VerseGrip 工具姿态。缺少 VerseGrip 时允许用只读 `probe_position` 检查 XYZ，但禁止启动 6D 跟随，不会静默锁住 RPY。
- 当前配置选择设备 ID `06DA`，沿用原 XYZ 倍率 `[1.5, 1.5, 1.5]`、位移范围 `[290, 275, 300] mm` 和 `150 mm/s`。
- 新增姿态软件上限：倍率 `[1,1,1]`、每轴相对范围 `±0.35 rad`、最高 `0.1 rad/s`；这些尚未经过 CR5 单轴实机标定。
- 当前要求 Haply 会话 `basis=XYZ`，连接时只读核验，不自动改变 Hub 的会话配置；设备安装方向、XYZ 轴对应、ABC 正负号和组合旋转必须在以后接入 CR5 时逐轴低速验收。
- Inverse3 只读模式不支持主动锁定或回位，因此 RoArm 的同步 HOME/A/B/C/D 功能在选择 Inverse3 时禁用。
- VerseGrip 按钮到 O6 的语义尚未定义；当前只保留 O6 独立快捷动作，不猜测按钮映射。
- 2026-08-20 最新只读盘点：Hub 已识别 Inverse3 `06DA` 和无线 VerseGrip Stylus `1792`；两者均为 `basis=XYZ`，手柄 `connected=true`、`awake=true`、`ready=true`。新适配器已真实读取 XYZ `(13.251, -44.467, 197.468) mm` 与四元数 `(w=.1027,x=.0065,y=.9771,z=.1861)`；CR5 尚未连接，未发送从臂命令。

### 2.3 LinkerHand 右手 O6

- 型号：LinkerHand right O6。
- 当前控制量：6 路位置目标，原始范围 `0..255`。
- 通信：Modbus RTU / RS485，115200 baud。
- 当前从站地址：`0x27`（十进制 39）。
- 当前串口：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。
- 当前动作：张开手、中指、轻微抓取、小拇指、抓取。

O6 SDK 没有被确认存在通用的“单电机重新使能”寄存器。Qt 的单电机恢复是安全重发当前位置、速度和非零力矩，不应把它描述成固件级清错。

### 2.4 相机

- CAMERA1 腕部 D435：序列号 `317222074617`。
- CAMERA2 基座 D435：序列号 `254622075848`。
- 基座 ROI：归一化 `[0.37, 0.56, 0.55, 0.79]`。
- 在 640×480 下约为 `(left=237, top=269, right=352, bottom=379)`。
- 当前 Episode 输入统一缩放到 224×224 RGB；不保存 640×480 原图和深度图。

## 3. 两种控制合同必须区分

### 3.1 人工主从遥操作

人工跟随只使用 RoArm TCP 的 XYZ 变化：

```text
CR5_target_xyz = CR5_start_xyz + scale_xyz * (RoArm_xyz - RoArm_start_xyz)
CR5_target_rpy = CR5_start_rpy
```

- 每次开始 XYZ 跟随时重新记录主从零点；
- CR5 RPY 固定为开始跟随瞬间的姿态；
- M5 单独映射 O6 张开和闭合，不作为 CR5 关节；
- 修改 XYZ 倍率时会重建映射零点，避免目标突跳。

选择 Inverse3 时使用另一条人工 6D 合同：

```text
CR5_target_xyz = CR5_start_xyz + scale_xyz * (Inverse3_xyz - Inverse3_start_xyz)
q_delta = VerseGrip_q * inverse(VerseGrip_start_q)
q_target = scale_in_nrc_xyz(q_delta) * quaternion(CR5_start_abc)
CR5_target_abc = nearest_nrc_xyz_euler(q_target)
```

- 只使用 VerseGrip 姿态，不使用 Inverse3 `body_orientation`；
- 纳博特 TCP ABC 原生单位为 rad，欧拉顺序为内禀 `X'-Y'-Z'`；
- 每次启动和修改倍率时重建位置/姿态零点；
- RPY 范围、单帧跳变和角速度均独立限幅；
- Inverse3 模式不自动控制 O6，且不改变原 RoArm 固定 RPY/M5 路径。

2026-08-19 同时修正了采集端的单位错误：旧代码曾把 NRC 原生 rad 再执行一次 `math.radians()`。修正只影响此后新采集的数据；此前 Episode 的 TCP RPY/ΔRPY 是否缩小约 57.3 倍需要单独审计，未审计前不要与新数据混合训练。

### 3.2 OpenPI 模型控制

模型动作不锁定 RPY。物理动作是 12 维：

```text
[CR5 delta XYZ (m) x3,
 CR5 delta RPY (rad) x3,
 O6 position target (0..255) x6]
```

OpenPI 部署接口会对 XYZ、RPY、工作空间和 O6 单步变化做限制，然后执行 IK、ServoJ 和 O6 RS485 命令。

因此，“人工遥操作固定 RPY”和“模型输出允许 delta RPY”不是同一条控制合同，不应互相覆盖。

## 4. 代码和目录职责

### 4.1 Qt 遥操作与采集：`/home/ace/cyf/TEST_PY12`

- `TEST_INEXBOT.py`
  - Qt 主窗口、页面整合、快捷键、双相机预览；
  - 设备连接、Episode 操作和用户日志；
  - 只负责协调，不应把新的硬件状态机继续堆在按钮回调里。
- `teleop_runtime.py`
  - `NrcRobotAdapter`：NRC 6001/7000 串行化访问；
  - `RoArmSerialController`：RoArm 串口反馈和力矩控制；
  - `Inverse3Controller`：Haply Service 只读位置/VerseGrip 姿态 probe；
  - `O6Controller`：O6 Modbus 工作线程；
  - `TeleopEngine`：跟随、示教点、主臂自由、Replay、急停和数据状态/动作生成。
- `lerobot_recorder.py`
  - Qt Python 3.12 与 LeRobot 环境之间的 Unix socket bridge；
  - 20 Hz 采样；
  - 三路 224×224 RGB 实时 H.264 编码，CRF 28；
  - 直接写 LeRobot v3，不需要先录 ROSBag 再转换。
- `config/roarm_cr5_teleop.json`
  - 当前实机串口、`robot_num`、倍率、安全范围、O6 参数、Replay 参数、数据集参数、快捷键和 HOME/A/B/C/D 示教点。
  - 此文件含真实示教位置，覆盖前必须确认。
- `config/joint_replay.json`、`config/joint_replay_2.json`、`config/joint_replay_3.json`
  - 三段独立的 CR5 + O6 同步关节轨迹；第一段沿用原文件。
- `TELEOP_README.md`
  - 详细操作说明。
- `test_teleop_runtime.py`
  - 无硬件单元测试。
- `run_test_inexbot.sh`
  - 正确启动入口。

`TEST_PY12` 当前不是 Git 仓库；修改没有正常的提交历史保护，做大改动前应先建立可恢复备份或纳入版本控制。

### 4.2 OpenPI 训练与部署：`/home/ace/cyf/openpi`

- 当前分支：`cr5-o6-pi05`。
- `origin`：`https://github.com/euiiue/openpi-for-diversity-robot.git`。
- `examples/Dobot/interface.py`
  - 独立、可迁移的 CR5 + O6 + 双 D435 硬件接口；
  - 自带 `vendor/nrc_linux_x86_64`，不依赖 `TEST_PY12` 或 `teleop_runtime.py`；
  - 状态 18 维、物理动作 12 维；
  - `connect()` 只连接，不自动上使能或运动；
  - `start_control()` 检查工作空间、伺服状态和 O6 后才打开 ServoJ。
- `examples/Dobot/config/{robot,camera,roi}.yaml`
  - 部署机器相关配置。
- `src/openpi/policies/Dobot_policy.py`
  - 数据、模型输入输出和 12 维物理动作合同。
- `src/openpi/training/config.py`
  - `pi05_dobot_cr5_o6_roi_lora` 训练配置。
- `examples/Dobot/main.py`
  - 只读延迟 benchmark 和 baseline 客户端入口。
- `examples/Dobot/main_rtc.py`
  - RTC 控制客户端。

OpenPI 工作区当前有多项未提交修改。不要使用 `git reset --hard` 或覆盖这些文件；先检查 diff 并提交到用户自己的分支。

## 5. LeRobot / OpenPI 数据合同

### 5.1 `observation.state`：18 维

```text
CR5 J1..J6                    6 维，rad
CR5 TCP XYZ                  3 维，m
CR5 TCP RPY                  3 维，rad
O6 六路实际位置反馈           6 维，原始 0..255
合计                         18 维
```

### 5.2 `action`：12 维

```text
CR5 实际下发的 delta TCP XYZ  3 维，m
CR5 实际下发的 delta TCP RPY  3 维，rad
O6 六路实际下发目标           6 维，原始 0..255
合计                         12 维
```

应记录经过限幅、量化后真正下发的动作，而不是未执行的原始提议。

### 5.3 图像键

当前 Qt 采集器写入：

| LeRobot/OpenPI 键 | 实际画面 |
|---|---|
| `observation.images.base_0_rgb` | CAMERA2 基座完整图 |
| `observation.images.left_wrist_0_rgb` | CAMERA1 腕部完整图 |
| `observation.images.right_wrist_0_rgb` | CAMERA2 基座 ROI |

第三个键只是为了适配 PI0.5 的三图像输入槽，实际是基座 ROI，不是真正的右腕相机。训练、部署和可视化必须始终使用相同语义。

OpenPI 模型内部 `action_dim=32`，但只有前 12 维是本系统的物理动作；策略变换负责 12↔32 维适配，硬件接口只能接收 12 维。

## 6. 当前 Qt 行为

### 6.1 推荐流程

1. 确认硬件急停可用、工作区无人；
2. “一键准备”启动双相机并连接 CR5、RoArm、O6，不运动；
3. 单独点击“CR5 上使能”，不启动跟随；
4. 单独点击“开始 XYZ 跟随”；
5. 填写任务文字和数据目录后开始 Episode；
6. 成功则保存并开始下一条，失败则丢弃并重录；
7. 采集结束后执行“结束数据集”，完成视频和索引收尾。

### 6.2 示教点和 Replay

- `Shift+A/B/C/D`：保存示教点；Episode 录制中禁止覆盖。
- `A/B/C/D`：回示教点；Episode 录制可继续采样。
- `HOME`：独立初始位；成功到位后清除 O6 独立手势保持，下一次 XYZ 跟随由 M5 控制 O6。
- CR5、RoArm、O6 进入示教点容差后会主动结束 CR5 MoveJ 占用，并在最多 5 秒内确认停止，不再继续占用 90 秒回位等待上限。
- 只有在操作前已经处于 XYZ 跟随，示教点/Replay 完成后才自动重建零点并恢复跟随。
- 从空闲状态触发示教点/Replay，完成后仍保持空闲，不应自行启动跟随。
- Replay 可以只选择某些 CR5 关节；未选关节保持触发 Replay 时的实际位置。
- 界面提供轨迹 1/2/3 三组独立录制、保存和回放按钮；快捷键 `R` 仍对应轨迹 1。
- Episode 内插入式 Replay 不停止 Episode 采样。

### 6.3 O6 快捷动作

- 快捷动作只控制 O6，正常情况下不停止 CR5 XYZ 跟随；
- 执行快捷动作后会屏蔽 M5，避免手势立即被 M5 覆盖；
- 点击“恢复 M5 控制 O6”或成功回到 HOME 后重新启用实时开合映射。

### 6.4 CR5 清错并上使能

- 按钮：`CR5 清错并上使能（机器人 1，不跟随）`；
- 执行前先确认 6001 端口连接正常并读取伺服状态；
- 报警(2)：依次调用 `clear_error_robot(command_fd, 1)`、等待 300 ms、再次读取并确认已脱离报警(2)、`set_servo_poweroff_robot(command_fd, 1)`、等待停止(0)/就绪(1)、`set_servo_state_robot(command_fd, 1, 1)`、确认就绪(1)、`set_servo_poweron_robot(command_fd, 1)`，最终确认运行(3)；
- 清错后仍为报警(2)会立即失败，不继续下电或上电；
- 停止(0)：设置就绪(1)后上电；就绪(1)：直接上电；运行(3)：不重复操作；
- 不启动 ServoJ、不启动 XYZ 跟随、不下发运动命令；
- Episode 或运动任务进行中禁止执行。

如果从站、EtherCAT、急停或硬件故障仍存在，软件清错可能立即再次报警。

## 7. 当前数据与训练状态

`/home/ace/datasets/cr5_o6` 下当前发现 4 个会话：

- `cr5_o6_20260812_045533_499776703`：0 Episode / 0 帧；
- `cr5_o6_20260813_013058_612040831`：1 Episode / 59 帧；
- `cr5_o6_20260813_013126_070064078`：0 Episode / 0 帧；
- `cr5_o6_20260813_032540_026702293`：1 Episode / 1505 帧。

这里只确认了元数据计数，没有检查图像内容、动作同步、任务是否成功或异常帧，所以两个非空会话也不能直接认定为训练合格数据。

`/home/ace/outputs/pi05_cr5_o6` 当前没有发现训练 checkpoint。OpenPI 训练配置中的 `repo_id="local/dobot_cr5_o6"` 仍是占位值，正式训练前需要指向实际数据集。

推荐下一阶段门槛：

1. 可视化并人工验收现有非空 Episode；
2. 检查三路图像语义、帧数、state/action 维度和时间连续性；
3. 丢弃失败、卡顿、遮挡或不同步 Episode；
4. 统一任务文字和操作成功标准后批量采集；
5. 计算训练所需统计量；
6. 先做单 Episode 过拟合，证明数据合同和训练链路成立；
7. 再做完整训练、checkpoint 部署和只读推理延迟测试；
8. 根据实测 P95 RTT 决定 RTC `delay_steps`，不能长期沿用占位值 4。

## 8. 已完成的软件验证

2026-08-13 当前检查结果：

- `teleop_runtime.py`、`TEST_INEXBOT.py`、`test_teleop_runtime.py` 编译通过；
- 45 个 runtime/recorder 单元测试通过；
- Qt 离屏启动 8 秒无初始化 traceback；
- 编号版 NRC 伺服状态机测试验证所有相关调用都带 `robotNum=1`。

测试命令：

```bash
cd /home/ace/cyf
PYTHONPATH=/home/ace/cyf \
LD_LIBRARY_PATH=/home/ace/cyf/TEST_PY12/lib \
/home/ace/cyf/TEST_PY12/bin/python3.12 \
  -m unittest -v TEST_PY12.test_teleop_runtime
```

Qt 无运动启动检查：

```bash
cd /home/ace/cyf/TEST_PY12
timeout 8s env QT_QPA_PLATFORM=offscreen ./run_test_inexbot.sh
```

无 traceback 且退出码为 124，表示 `timeout` 结束了仍正常运行的 Qt 事件循环。

上述验证没有连接或移动实机，不能证明 CR5、RoArm、O6 或相机当前硬件状态正常。

## 9. 明确未完成和待处理项

### 高优先级

1. **OpenPI 独立接口尚未同步 robotNum**  
   `TEST_PY12` 已使用编号版 NRC 伺服接口，但 `openpi/examples/Dobot/interface.py` 当前仍使用无编号的 `get_servo_state`、`clear_error`、`set_servo_*` 和 `get_robot_running_state`。部署前应给 `InterfaceConfig`/YAML 增加 `robot_num: 1`，并与 Qt 采用同一套编号版状态机。

2. **编号版接口尚未实机验证**  
   单元测试已经覆盖调用参数，但尚未在本轮对真实控制柜执行清错、上使能或运动。

3. **数据质量尚未验收**  
   目前只有两个非空单 Episode 会话，且未做逐帧回放和成功/失败复核。

4. **训练链路尚未闭环**  
   实际 `repo_id`、统计量、单 Episode 过拟合、正式训练、checkpoint 和部署测试都未完成。

### 实机确认项

- CR5 控制柜和从站是否无报警；
- 机器人 1 的伺服状态能否稳定到 3；
- 6001/7000 同时连接且不会被其他上位机占用；
- O6 串口权限、地址、反馈和故障码；
- RoArm 串口身份和 M2/M3 镜像方向；
- 两台 D435 序列号与腕部/基座物理位置仍匹配；
- ROI 是否仍覆盖当前任务区域；
- OpenPI 工作空间边界和 20 mm 内缩是否经过现场重新确认；
- 模型 RPY 动作的单步限幅和方向是否经过低速实机验证。

## 10. 安全边界

- 软件急停不能代替控制柜硬件急停；
- 连接成功不等于允许运动；
- 清错并上使能成功不等于允许开始 XYZ 跟随或模型控制；
- 上使能成功不等于允许开始 XYZ 跟随或模型控制；
- 自动重连只恢复通信，不应自动恢复上使能、HOME、Replay 或跟随；
- 测试新映射、Replay、模型 RPY 或工作空间边界时，必须低速、单步、工作区无人并保持硬件急停可触达；
- 不要同时让 Qt 遥操作和 OpenPI 部署进程打开同一 NRC、O6 或相机设备。

## 11. 下一位开发者建议从这里开始

1. 阅读本文件和 `TELEOP_README.md`；
2. 先运行 45 个无硬件测试；
3. 检查 `config/roarm_cr5_teleop.json`，不要覆盖现有 HOME/A/B/C/D；
4. 在不使能的情况下启动 Qt，核对两个相机和三个设备连接；
5. 现场确认报警、急停和工作空间后，再逐项验证清错、上使能、XYZ 跟随、示教点和 Replay；
6. 可视化两条非空 LeRobot Episode，决定保留或丢弃；
7. 将 `robot_num=1` 同步到独立 OpenPI 接口并补无硬件测试；
8. 完成单 Episode 过拟合前，不进入长时间训练或实机模型闭环。
