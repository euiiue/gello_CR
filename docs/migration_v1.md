# 迁移到 v1.0

v1.0 是代码与发布准备版本，现场验收尚未签收。正式 HMI 入口统一为 `scripts/run_operator.sh`，配置统一为 `config/workbench.json`。迁移不需要连接硬件，也不改写已发布训练数据。

1. 保留旧工作副本、标定和录制目录。把实际使用的旧 `config/roarm_cr5_teleop.json` 内容复制到 `config/workbench.json`；仓库内已完成该改名。不要用默认配置覆盖现场偏置、方向、HOME、O6 动作或序列号。
2. 对照 [配置说明](configuration.md) 核对 Profile、串口、控制器、模式与阈值。相对路径现统一相对于配置文件目录；尤其检查 Worker Python、数据/轨迹目录和 SDK 路径。新字段会合并缺省值，但部署身份与运动限制仍需人工核定。
3. 建立 Python 3.12 应用环境和各格式独立 Worker。使用 `tools/diagnostics.py` 检查解析后的身份与路径；它不连接设备。先离线预览，再进行 [现场验收](release_v1.md)。

## 旧轨迹与数据

历史 JSON 输入移到 `examples/legacy_trajectories/`，包括四个 `joint_replay*.json` 文件。UI 可直接只读导入；批量一次性转换使用相同 `LegacyJsonAdapter`：

```bash
.venv/bin/python tools/migrate_legacy_trajectory.py --help
.venv/bin/python tools/migrate_legacy_trajectory.py \
  --config config/workbench.json \
  --root /path/to/new_trajectory_store \
  examples/legacy_trajectories/*.json
```

工具只读源 JSON，校验版本 1/2/3、有限关节值和时间顺序，将 deg 转为 rad，输出 UUID 目录中的 `trajectory.parquet` 与 `meta.json`。省略 `--root` 使用配置的轨迹目录；显式相对 `--root` 按当前工作目录解析，部署建议使用绝对路径。重复执行会产生新的 UUID 副本，应按需转换一次。旧第七轴没有导入为六轴轨迹通道，需检查导入预览与所选关节。

```bash
.venv/bin/python scripts/replay_trajectory.py examples/legacy_trajectories/joint_replay.json --preview
.venv/bin/python scripts/replay_trajectory.py /path/to/lerobot_dataset --episode-index 0 --preview
```

默认 CLI 仅预览；`--execute` 会明确连接所选设备、使能所选 Arm 并调用统一 Player，执行前必须完成现场确认。TCP 数据集导入回放采用实测关节，不把 TCP delta 当作绝对关节目标。旧数据保持只读；未完成新批次通过数据集管理页处理 Partial 并恢复到新 Segment。

## 旧入口变化

根目录 `teleop_runtime.py`、`lerobot_recorder.py`、`TEST_INEXBOT.py`、`run_test_inexbot.sh` 与旧 TELEOP_README 不再作为运行入口。合成预览在 `examples/preview_ui.py`；功能实现归入 `src/gello_cr/`，测试归入 `tests/`。旧 A/B/C/D 运动命令由规划预览和明确执行取代，原 pose 数据保留。详细去向见 [Legacy Audit](legacy_audit.md)。
