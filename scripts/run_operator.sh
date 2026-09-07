#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# 优先使用用户显式指定的 Python。
# 当前机器默认使用已经验证过的 Python 3.12 环境。
default_python="${HOME}/cyf/TEST_PY12_MOVEJ_LOW_LATENCY_TEST/bin/python3.12"

if [[ -n "${GELLO_CR_PYTHON:-}" ]]; then
    python_bin="${GELLO_CR_PYTHON}"
elif [[ -x "${default_python}" ]]; then
    python_bin="${default_python}"
else
    python_bin="python3.12"
fi

python_prefix="$(
    "$python_bin" -c \
    'import sys; print(sys.prefix)'
)"

# NRC SDK:
# 显式 NRC_SDK_ROOT 优先；
# 否则尝试当前 Python 环境旁边的 TESTRobot_INEXBOT。
if [[ -z "${NRC_SDK_ROOT:-}" ]]; then
    candidate="${python_prefix}/TESTRobot_INEXBOT"

    if [[ -d "${candidate}" ]]; then
        export NRC_SDK_ROOT="${candidate}"
    fi
fi

# NRC native extension / Python runtime shared libraries。
export LD_LIBRARY_PATH="${python_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# 项目源码。
export PYTHONPATH="${repo_root}/src:${repo_root}${PYTHONPATH:+:${PYTHONPATH}}"

# 启动后仍保持 OFFLINE。
# 不自动连接机器人、不上使能、不启动 ServoJ。
exec "$python_bin" -m gello_cr.ui \
    --repo-root "$repo_root" \
    "$@"
