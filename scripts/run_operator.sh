#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# 优先使用用户显式指定的 Python，其次使用本项目虚拟环境。
default_python="${repo_root}/.venv/bin/python"

if [[ -n "${GELLO_CR_PYTHON:-}" ]]; then
    python_bin="${GELLO_CR_PYTHON}"
elif [[ -x "${default_python}" ]]; then
    python_bin="${default_python}"
else
    python_bin="python3.12"
fi

python_paths="$(
    "$python_bin" -c \
    'import sys; print(sys.prefix); print(sys.base_prefix)'
)"
python_prefix="$(sed -n '1p' <<<"${python_paths}")"
python_base_prefix="$(sed -n '2p' <<<"${python_paths}")"

# NRC SDK 默认由本项目定位；仅保留显式 NRC_SDK_ROOT 覆盖。
# 原生扩展可能依赖 venv 基础解释器中的 libpython。
export LD_LIBRARY_PATH="${python_base_prefix}/lib:${python_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# 项目源码。
export PYTHONPATH="${repo_root}/src:${repo_root}${PYTHONPATH:+:${PYTHONPATH}}"

# 启动后仍保持 OFFLINE。
# 不自动连接机器人、不上使能、不启动 ServoJ。
exec "$python_bin" -m gello_cr.ui \
    --repo-root "$repo_root" \
    "$@"
