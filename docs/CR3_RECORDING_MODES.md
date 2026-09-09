# CR3 LeRobot 记录模式

右上角「设置」→「数采与相机」→「LeRobot 记录模式」自由选择 TCP 或 CR3 关节模式，保存后重启生效。沿用设置页的重启机制；录制中不可切换。同一个数据集不能混用两种模式。旧配置缺少 `dataset.recording_mode` 时默认 TCP。

| 模式 | observation.state | action |
| --- | --- | --- |
| `tcp`（原格式） | 18 维：关节 rad ×6、TCP XYZ m ×3、RPY rad ×3、O6 原始反馈 ×6 | 12 维：已下发 TCP 目标减当前反馈（m/rad，姿态最短角差）×6、O6 原始目标 ×6 |
| `joint` | 12 维：CR3 当前 q1–q6 rad、O6 原始反馈 ×6 | 12 维：CR3 已下发的绝对 q1–q6 rad、O6 原始目标 ×6 |

关节模式读取跟随线程同次遥测的反馈和最后成功提交的目标，度转弧度，不记录 GELLO 原始角，不把目标减反馈当作关节动作。第七个 SDK 占位轴不进入数据。元数据使用 `cr3.q1.rad` 至 `cr3.q6.rad` 和 `cr3.target_q1.rad` 至 `cr3.target_q6.rad`。TCP 的历史 `cr5.*` 字段名称保持兼容。

关节模式必须先启动 GELLO 关节跟随，并至少提交过一次目标，才能开始 Episode；停止跟随、进入预设或回放时记录明确报错停止，不能继续保存为成功 Episode。先结束 Episode，再停止跟随。三路 RGB、O6、质量统计和保存／丢弃流程保持原有机制。

新关节数据需要匹配的训练和部署 action/state 配置；现有 TCP 模型不能直接使用这一契约。

## 验证

离线测试覆盖 90°→π/2、负角与超过 180°的绝对目标（不折返角度）、反馈／已下发目标的区分、设置双向切换保存、跨模式帧拒绝。真实 LeRobot 子进程以合成图像和状态验证两种模式的 Episode 保存／失败标记／丢弃／收尾，并解码 MP4、回读 Parquet 检查帧数、维度、时间戳和模式。测试不访问机器人或相机。

采集结束并收尾后可检查真实数据：

```bash
GELLO_CR_DATA_PYTHON=/home/ace/miniconda3/envs/lerobot/bin/python \
  ./scripts/validate_dataset.sh /实际数据集目录
```

`valid: true` 表示文件结构检查通过；仍需检查 `needs_review_episodes`，并用首次短采集确认真机反馈与操作者动作一致。
