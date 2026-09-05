# RoArm-M2-Pro / Haply Inverse3 → CR5 → LinkerHand O6 遥操

## 启动

纳博特 `_nrc_host.so` 依赖项目内的 Python 3.12 和 `libpython3.12.so`，请使用：

```bash
cd /home/ace/cyf/TEST_PY12
./run_test_inexbot.sh
```

不要直接用系统 `python3`（当前是 Python 3.10）运行。

## 连接顺序

1. 确认硬件急停可随时触发，CR5 工作区无人。
2. 在“机器人”页点击连接，建立 `192.168.1.245:6001` 和 `:7000` 两个连接。
3. 在“高级遥操设置”选择主臂：
   - `Inverse3 + VerseGrip（XYZ + RPY）`：连接本机 Haply Inverse Service `ws://127.0.0.1:10001`；
   - `RoArm-M2-Pro（XYZ，锁定 RPY）`：继续使用原 USB 串口路径。
4. 未连接 CR5/O6 时可先点击“只读连接主臂”：只有 Inverse3 本体时可观察 XYZ，检测到 VerseGrip 后同时显示 ABC；完整系统再点击“连接主臂 + O6”。缺少 VerseGrip 时“开始 6D 跟随”会明确失败，不会退回锁定 RPY。
   - 若先以 XYZ-only 连接，之后才打开/接入 VerseGrip，请再次点击“只读连接主臂”重新发现姿态设备。
5. 首次实机跟随把 X/Y/Z 倍率都设为 `0.5`，确认方向后再逐步放大。
6. 使能 CR5，点击“开始主从跟随”。

## 映射逻辑

### RoArm（保留的原路径）

每次开始跟随时，程序同时记录 RoArm TCP 和 CR5 TCP：

```text
CR5_target_xyz = CR5_start_xyz + scale_xyz * (RoArm_xyz - RoArm_start_xyz)
CR5_target_rpy = CR5_start_rpy
```

录制中可以修改 XYZ 倍率和安全范围并点击“保存并立即应用 XYZ 映射”。倍率变化时会以当前主从位置重建倍率零点，因此不会把 CR5 目标瞬间跳到新倍率对应的位置；安全范围仍始终相对本次 XYZ 跟随启动时的 CR5 原点计算。

RoArm M5 使用实测张开/闭合端点 `0.7977 / 3.1447 rad`，线性插值到 O6 六个执行器：

- 张开：`[253, 253, 253, 253, 253, 253]`
- 安全闭合：`[90, 99, 50, 50, 50, 50]`

### Inverse3 + VerseGrip（新增 6D 路径）

- XYZ 来自 `inverse3.state.cursor_position`，由 m 转为 mm 后继续使用原有 XYZ 倍率、范围和 `150 mm/s` 速度参数。
- 姿态只来自 VerseGrip `state.orientation` 四元数；`inverse3.state.body_orientation` 是机身姿态，禁止当作末端 RPY。
- 每次开始跟随记录主手四元数 `q0` 和 CR5 当前 ABC；运行时计算 `q_delta = q_current * inverse(q0)`，按纳博特内禀 `X'-Y'-Z'` 约定缩放后与 CR5 初始四元数组合，最后转换为最接近上一命令的合法 ABC(rad)。
- 默认 RPY 倍率为 `[1, 1, 1]`，相对范围为每轴 `±0.35 rad`，最高角速度 `0.1 rad/s`。这些是软件保护上限，不是实机标定结果。
- 当前软件要求 Haply 会话 `basis=XYZ`，连接时只读核验且不替用户修改 Hub 配置；不一致会直接失败。安装方向、XYZ 对应关系和 ABC 正负号仍须在接入 CR5 后逐轴低速确认，确认前不能视为实机适配完成。
- 只向 Haply Service 发送官方只读 `probe_position` / `probe_orientation`，不发送力、位置或力矩控制；停止跟随后 Inverse3 仍保持自由。
- 连接时要求 Inverse3 `in_use=false` 且 `control_mode=idle`，并检查 VerseGrip `ready/connected/awake`；GEL 或其他 Haply 控制会话仍占用设备时会拒绝连接。
- VerseGrip 按钮尚未定义为 O6 开合输入，Inverse3 模式使用现有 O6 快捷动作；程序不会猜测 A/B/C 的含义。

纳博特 TCP 的 XYZ 单位是 mm，ABC 原生单位是 rad。采集时只对 XYZ 做 mm→m；ABC 不再重复做 degree→rad 转换。

## 示教点

以下同步示教点属于可主动锁定/回位的 RoArm 路径。Inverse3 适配使用只读 probe，不会偷偷切换为 Haply 位置控制，因此在 Inverse3 模式下 HOME/A/B/C/D 同步保存与回位按钮会禁用。

- `Shift+A/B/C/D`：同时保存 RoArm 四个逻辑关节、CR5 七元关节/TCP 和 O6 六个位置。
- `A/B/C/D`：CR5 用 MoveJ、RoArm 和 O6 同时主动回位；三者进入示教点容差后会主动结束 MoveJ 占用，并在确认 CR5 停止后重建零点。只有执行前本来就在 XYZ 跟随才恢复跟随；从空闲状态回位则继续保持空闲。
- Episode 录制中仍可按 `A/B/C/D`：采样不会停止，回位阶段记录 MoveJ 的目标 TCP，完成后无空档恢复 XYZ 跟随。录制期间不允许 `Shift+A/B/C/D` 覆盖示教点。
- 输入框正在编辑时，A/B/C/D 不会触发运动。

示教点和参数保存在 `config/roarm_cr5_teleop.json`。

## 主臂自由与初始位

- “从臂保持 / 主臂自由”：停止 CR5 运动命令并保持当前位置，O6 保持当前手势，RoArm 全部卸力供徒手拖动。
- “停止当前动作并锁住主臂”：停止跟随、示教点回位、O6 恢复或轨迹重放，并让 RoArm 在当前反馈位置重新上力矩。
- “保存当前为初始位”：保存独立 `HOME` 位，包含 RoArm、CR5 和 O6。
- “主从臂一键回初始位”：三者同时恢复 `HOME`；CR5 使用 MoveJ，到位后保持，不自动启动跟随。HOME 成功后同时退出 O6 独立手势保持，下一次 XYZ 跟随由 RoArm M5 控制 O6。
- “回位等待上限”默认 `90s`，表示 HOME/A/B/C/D 最长允许等待时间；它不会改变 MoveJ 实际速度，可在界面设置为 `10～300s`。

## CR5 连接与上使能

- 当前单 CR5 在 `config/roarm_cr5_teleop.json` 中明确配置为 `robot_num: 1`。伺服状态读取、清错、上/下使能和运行状态查询统一使用 NRC 的 `*_robot(..., robot_num, ...)` 接口。
- 采集操作台的“CR5 清错并上使能（机器人 1，不跟随）”按钮先确认 6001 端口连接并读取伺服状态。报警(2)时依次调用 `clear_error_robot(..., 1)`、等待 300 ms、再次读取并确认已脱离报警(2)、`set_servo_poweroff_robot(..., 1)` 释放占用、等待停止(0)/就绪(1)、`set_servo_state_robot(..., 1, 1)`、确认就绪(1)、`set_servo_poweron_robot(..., 1)`，最后确认运行(3)。清错后仍为报警(2)会立即失败，不继续下电或上电。停止(0)先切换就绪(1)再上电；就绪(1)直接上电；运行(3)不重复操作。该按钮不会启动 ServoJ、XYZ 跟随或下发运动命令。
- 采集操作台把两步明确分开：“CR5 上使能”只把伺服切换到运行(3)，不会释放 RoArm、不会打开 ServoJ；“开始 XYZ 跟随”只启动跟随，不会自动连接或自动上使能。
- 默认快捷键为 `Ctrl+F6` 单独上使能、`F6` 单独开始 XYZ 跟随；均可在快捷键绑定页修改。
- “开始 XYZ 跟随”不再被界面每 `200ms` 的伺服显示缓存禁用；点击后由遥操作运行时直接读取控制柜实时状态，不满足运行(3)时在日志中快速失败并说明原因。
- 连接控制柜时会参考 `ros/nrc_interface_ros2` 驱动，分别等待 6001 示教端口和 7000 跟踪端口真正就绪，再将界面标记为已连接。
- 点击“连接纳博特 CR5”时会先检查现有 6001/7000 fd；两路都为 `0` 且伺服状态可读才复用连接。否则会丢弃遥操作引擎中的旧适配器、关闭两个旧 fd，再新建两路连接。
- 连接后每 `200ms` 在后台检查两路 SDK 状态。检测到掉线或返回 `-2` 时会停止跟随/回放命令源、锁住 RoArm、保持 O6，并把界面改为未连接；随后每 `5s` 在后台尝试重连。
- 自动重连只恢复通信，不会自动上使能、HOME、恢复 XYZ 跟随或恢复 Replay。重连成功后必须由操作者检查现场并主动恢复运行。
- “上使能”使用统一伺服状态机：停止(0)先切换就绪(1)再上电；报警(2)先清错、下电释放占用，再切换就绪并上电；运行(3)不重复操作。
- 上电命令返回后必须再次读取并确认状态为运行(3)，否则 HOME、A/B/C/D 和 XYZ 跟随都不会下发运动。
- SDK 错误码会显示含义，例如 `-1` 为接收控制柜响应失败、`-4` 为当前状态不允许操作。

## 安全停止

下列任一情况会停止 CR5 ServoJ、保持 O6，并要求人工重新开始；RoArm 会锁定当前位置，Inverse3 只读模式则保持自由：

- 主臂/O6 反馈超时或通信故障；
- O6 任一执行器报错；
- 主臂 XYZ 单帧跳变超限；
- VerseGrip 姿态丢失或单帧角度跳变超限；
- 任一 XYZ 相对位移超限；
- 任一 ABC 相对转角超限；
- CR5 逆解或 ServoJ 下发失败。

“软件紧急停止”会中断跟随、预设回位、O6 恢复和轨迹重放，并让三台设备保持当前位置；后台任务不会在急停后重新发布空闲/跟随状态。软件停止不能代替控制柜的硬件急停。

## O6 单电机受阻恢复

- 在“O6 电机故障恢复”中选择电机；默认 `3 - 食指弯曲`，编号从 1 开始。
- 移除障碍并确认手指周围无人后，点击“恢复所选 O6 电机（安全重发）”。程序会先停止跟随，把该电机目标覆盖为当前反馈位置，再重发配置中的速度和非零力矩。
- O6 RS485 SDK 没有独立的使能/清故障接口，因此该按钮不会写未知寄存器，也不能保证清除固件锁存故障。故障码仍非 0 时请断电检查，不要继续遥操作。
- 温度过高、编码器错误、过压/欠压以及未知故障会被程序拒绝重新上力；恢复成功后也不会自动启动 XYZ 跟随。

## O6 快捷动作

“RoArm 遥操作”页提供五个只控制灵巧手的动作按钮：

- 张开手：`[250, 250, 250, 250, 250, 250]`
- 中指：`[91, 132, 0, 250, 0, 0]`
- 轻微抓取：`[107, 90, 129, 250, 250, 250]`
- 小拇指：`[30, 250, 40, 40, 40, 250]`
- 抓取：`[78, 85, 123, 250, 250, 250]`

点击任一动作只下发 O6 六维目标，不会停止 CR5 的 XYZ 跟随。程序会进入“O6 独立保持”模式并屏蔽 RoArm M5 对灵巧手的控制，因此手势到位后不会立刻被 M5 覆盖；点击“恢复 M5 控制 O6”或成功回到 HOME 会重新启用 M5 开合映射。动作目标、速度与 `action_timeout_s` 保存在 `config/roarm_cr5_teleop.json`，可直接调整；修改后需重启程序。

## CR5 + O6 同步关节轨迹录制与 Replay

- 界面提供三组独立按钮：`录制1/停止并保存1/回放1`、`录制2/停止并保存2/回放2`、`录制3/停止并保存3/回放3`。同一时间只允许录制或回放其中一段。
- 任一“录制”按钮都以配置的 `sample_period_s`（默认 `0.05s`）同步记录 CR5 七元关节反馈和 O6 六元实际位置反馈；可以在 XYZ 跟随过程中录制。O6 动作无论来自 M5 还是快捷手势都会被记录。
- 三段轨迹分别原子写入 `config/joint_replay.json`、`config/joint_replay_2.json`、`config/joint_replay_3.json`。轨迹 1 沿用原文件路径；只有点击“停止并保存1”或达到录制时限时才会用新轨迹替换该文件。默认最长录制 `300s`，达到上限会自动停止并保存当前段。
- “回放1/2/3”只读取对应文件，先停止 XYZ 跟随并锁住 RoArm；选中的 CR5 关节以点击时的实际位置为零点，重现“当前帧减录制首帧”的相对位移，未选关节始终保持点击时的位置，不再 MoveJ 跳回旧录制姿态。O6 仍从对应轨迹第一帧开始同步回放。
- CR5 的 20 Hz 录制帧会线性插值为 `0.01s` 的 ServoJ 下发周期；示教点 MoveJ 必须等待控制柜确认运动完全停止并经过模式稳定时间后，才允许恢复 XYZ/ServoJ。
- 回放按钮不再弹出二次确认；默认 `R` 立即执行回放 1（Episode 内则插入回放 1），回放 2/3 使用各自按钮，`Shift+R` 提前停止当前回放。两个快捷键都可在“键盘快捷键绑定”中修改或清空。
- 无论是否正在录制 Episode，只要轨迹重放前处于 XYZ 跟随，重放完成后都会以当前主从臂位姿重建零点并自动恢复跟随；如果重放前本来处于空闲/锁定状态，完成后仍保持锁定。
- 当 LeRobot Episode 正在录制时，同一按钮会变为“插入重放（Episode 继续录制）”。Episode 采样线程不停止：选中的 CR5 关节相对当前姿态重放，其他关节固定在点击时的实际位置，O6 同步重放。
- XYZ 跟随中的指令优先级为：硬件/软件停止最高；A/B/C/D 回位和轨迹重放会先抢占并停止当前 XYZ 跟随；O6 快捷手势只抢占 M5 对灵巧手的控制，CR5 XYZ 跟随仍继续。A/B/C/D 只有在抢占跟随时才会在完成后重建零点并恢复；从空闲状态回位不会自行启动跟随。HOME 仍作为停留在初始位的安全回位。
- 重放过程中点专用的“停止重放”会提前结束轨迹，并按重放前状态决定是否恢复 XYZ 跟随；点操作台的“停止当前动作并锁住主臂”或“软件紧急停止”则不会自动恢复运动。
- 重放结束后 O6 保持轨迹最后一帧手势，M5 仍处于屏蔽状态；点击“恢复 M5 控制 O6”才恢复实时开合映射。
- 轨迹文件会校验 CR5 七维关节、O6 六个 `0..255` 位置、严格递增时间戳和 `max_joint_speed_deg_s` 速度上限。旧版不含 `o6_position` 的轨迹必须重新录制。
- “停止重放”和“软件紧急停止”都会先停止 CR5 轨迹命令并保持 RoArm/O6；前者可按重放前状态恢复跟随，后者始终保持停止。软件按钮不能替代控制柜硬件急停。

## 无运动测试

```bash
cd /home/ace/cyf
TEST_PY12/bin/python3.12 -m unittest -v TEST_PY12/test_teleop_runtime.py
```
## LeRobot / PI0.5 Episode 采集

采集已直接接入 `TEST_INEXBOT.py` 的“RoArm 遥操”页，不需要先录 ROSBag。

启动方式不变：

```bash
cd /home/ace/cyf/TEST_PY12
./run_test_inexbot.sh
```

每个 Episode 的操作顺序：

1. 在采集操作台点击“启动全部相机”，确认 CAMERA1 腕部完整图、CAMERA2 基座完整图和 CAMERA2 ROI 都正常。
2. 连接 CR5、RoArm 和 O6，单独上使能 CR5，再启动 `XYZ 跟随`。
3. 在 `LeRobot / PI0.5 Episode 采集` 区域填写任务文字和保存根目录。
4. 点击 `开始 Episode`，完成一次完整操作。
5. 成功时点击 `成功并保存`；失败时点击 `失败并丢弃`。
6. 全部 Episode 完成后点击 `结束数据集`，这一步会写完 Parquet 和视频索引。

记录格式为 LeRobot v3：

- `observation.state`：CR5 J1～J6（rad）+ TCP XYZ/RPY（m/rad）+ O6 六路位置反馈，18 维。
- `action`：从当前 TCP 反馈到实际下发目标的 ΔTCP（m/rad）+ O6 实际下发目标，12 维。这与 `openpi/examples/Dobot/interface.py` 的部署合同一致，Episode 内插入关节轨迹时也能记录正确的腕部旋转增量。
- `observation.images.base_0_rgb`：CAMERA2 基座完整图。
- `observation.images.left_wrist_0_rgb`：CAMERA1 腕部完整图。
- `observation.images.right_wrist_0_rgb`：CAMERA2 基座 ROI；这是第三个模型输入槽，不是真正的右腕相机。

三路图像都是 `224x224 uint8 RGB`。RealSense 画面先按 OpenPI 的等比例、居中黑边方式缩放，再用 H.264 CRF 28 实时编码为 MP4；不记录深度图和 640x480 原图。

LeRobot 写入 worker 默认使用
`/home/ace/miniconda3/envs/lerobot/bin/python`，而 Qt/NRC 仍使用
`/home/ace/cyf/TEST_PY12/bin/python3.12`。两者通过本地 Unix socket 通信，不会重复打开机械臂或 O6 端口。

录制结束后，从数据集目录的 `meta/info.json` 读取 `repo_id`，然后可以先查看 Episode 0：

```bash
/home/ace/miniconda3/envs/lerobot/bin/lerobot-dataset-viz \
  --repo-id=ace/cr5_o6_时间戳 \
  --root=/home/ace/datasets/cr5_o6/cr5_o6_时间戳 \
  --episode-index=0 \
  --display-compressed-images
```

本地 PI0.5 expert-only 训练命令模板：

```bash
cd /home/ace/lerobot
TRANSFORMERS_OFFLINE=1 HF_HUB_OFFLINE=1 \
/home/ace/miniconda3/envs/lerobot/bin/lerobot-train \
  --policy.path=/home/ace/models/lerobot/pi05_base \
  --policy.device=cuda \
  --policy.dtype=float32 \
  --policy.train_expert_only=true \
  --policy.freeze_vision_encoder=true \
  --policy.push_to_hub=false \
  --dataset.repo_id=ace/cr5_o6_时间戳 \
  --dataset.root=/home/ace/datasets/cr5_o6/cr5_o6_时间戳 \
  --dataset.use_imagenet_stats=false \
  --dataset.return_uint8=true \
  --batch_size=1 \
  --num_workers=0 \
  --steps=30000 \
  --output_dir=/home/ace/outputs/pi05_cr5_o6
```

`pi05_base` 预训练配置期望 `base_0_rgb`、`left_wrist_0_rgb` 和
`right_wrist_0_rgb`。当前数据集已经提供全部三个键，但第三个键的实际语义是基座 ROI；训练和部署必须保持同一映射。
