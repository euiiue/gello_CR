# NRC SDK loading（阶段 1）

SDK 目录必须包含同一发行版的 `nrc_interface.py` 和 `_nrc_host.so`。
选择优先级为 `NRC_SDK_ROOT` 环境变量 > `robot.sdk_root` 配置 >
仓库 `TESTRobot_INEXBOT`。路径支持 `~`；相对路径按进程工作目录解析，
部署时建议用绝对路径。显式路径错误会直接失败，不回退到另一份 SDK。
已加载另一目录的 SDK 时必须重启进程，避免 Python 模块缓存混用。

运行时工厂会传递 `robot.sdk_root`，例如在现有 robot 配置内添加：

```json
"sdk_root": "/absolute/path/to/sdk"
```

构造设备和启动 UI 不加载 SDK，不连接控制器，也不启动相机。
只有显式 CONNECT 才进入 SDK 加载与连接流程。

## 本机证据与启动

仓库 SDK：`/home/ace/cyf/gello_CR/TESTRobot_INEXBOT`。
`_nrc_host.so` 为 x86-64 ELF，依赖 `libpython3.12.so.1.0`，其
RUNPATH 为本机不存在的 `/usr/local/python3.12/lib`。
系统默认环境 `ldd` 报该库 not found；本机可用库位于
`/home/ace/cyf/TEST_PY12/lib`，对应解释器为该环境的 `bin/python3.12`。
没有替换供应商二进制或修改机器人参数。

```bash
cd ~/cyf/gello_CR
GELLO_CR_PYTHON=/home/ace/cyf/TEST_PY12/bin/python3.12 scripts/run_operator.sh --check-only
GELLO_CR_PYTHON=/home/ace/cyf/TEST_PY12/bin/python3.12 scripts/run_operator.sh
```

启动脚本默认使用 `python3.12`，可由 `GELLO_CR_PYTHON` 指定解释器。
脚本在启动 Python 前加入该解释器的 `sys.prefix/lib` 到动态库搜索路径，
并设置应用源码路径。无需手工配置 PYTHONPATH；NRC 路径由设备加载器管理。
使用不同 SDK 时可设置 `NRC_SDK_ROOT=/absolute/sdk/path`。

## 两类错误

- Python module missing/lookup failed：检查 SDK 目录、Python wrapper 和扩展文件。
  `LD_LIBRARY_PATH` 不能修复 Python 模块路径。
- Native SDK loading failed：保留原始异常，检查 Python ABI、CPU 架构及
  `ldd /path/to/_nrc_host.so`。额外依赖目录需在启动前加入 `LD_LIBRARY_PATH`；
  `PYTHONPATH` 不能修复共享库缺失。

本阶段行为测试仅使用假 SDK；独立导入检查只加载真实 SDK，未调用任何
控制器 API。`--check-only` 已验证 OFFLINE、cameras stopped 和无硬件连接。
真实 6001/7000 与反馈验证属于下一人工 Gate，本阶段未执行。
