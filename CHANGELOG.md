# Changelog

## 1.0.0 — 发布准备（2026-10-16，现场验收待签收）

- 统一 CR3A + O6 + GELLO 工作台：正式 HMI 入口、六页界面、统一命令策略，启动保持 OFFLINE。
- Teleop、HOME、轨迹回放、Joint / TCP / Waypoint 规划共享 MotionCoordinator 和 lease；回放及规划轨迹使用同一 Player。STOP / FAULT / ESTOP 抢占后不自动恢复运动。
- 轨迹使用 UUID / Parquet Store；兼容只读导入旧 JSON 和 LeRobot Episode，增加一次性迁移工具。
- 采集抽象为 CaptureFrame / Session / Writer，支持独立环境中的 LeRobot V2.1、V3 和 HDF5；增加批次、Segment、journal、Partial 决策、恢复与校验后原子发布。
- 配置统一为 `config/workbench.json`，集中校验、原子保存和部署路径解析；保留现场标定、HOME、O6 动作与历史输入。
- 拆分应用服务，明确后台任务、反馈线程与 Worker 的归属、关闭顺序和失败重试；增加统一日志、领域错误和无硬件诊断。
- 清理根目录旧入口、兼容桥接与重复 UI 策略；保留已配置的 RoArm / Inverse3 和 GELLO 控制模式；新增开发/部署文档与离线 CI。

验证记录和未完成的现场安全、性能、外部 DAgger 联调见 [发布清单](docs/release_v1.md)。版本号不代表真机验收通过。

### 2026-10-17 整体结构审查

- 将现有 P0–P9 工作区作为统一基线审查，验证应用服务、命令通道、运动与录制的边界及退出重试。
- 保留 GELLO 物理采样时间戳与反馈失效处理，以及统一界面主题和对话框中的软件急停快捷键。
- 当前完整官方格式离线回归 834 项通过，无跳过；生产代码 lint、格式、编译和 diff 检查通过。真机签收仍按发布清单进行。
