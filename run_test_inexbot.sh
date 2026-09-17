#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
default_python="${repo_root}/.venv/bin/python"

if [[ -n "${GELLO_CR_PYTHON:-}" ]]; then
    python_bin="${GELLO_CR_PYTHON}"
elif [[ -x "${default_python}" ]]; then
    python_bin="${default_python}"
else
    python_bin="python3.12"
fi

python_paths="$(
    "${python_bin}" -c \
    'import sys; print(sys.prefix); print(sys.base_prefix)'
)"
python_prefix="$(sed -n '1p' <<<"${python_paths}")"
python_base_prefix="$(sed -n '2p' <<<"${python_paths}")"
export LD_LIBRARY_PATH="${python_base_prefix}/lib:${python_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${repo_root}/src:${repo_root}${PYTHONPATH:+:${PYTHONPATH}}"

exec "${python_bin}" "${repo_root}/TEST_INEXBOT.py" "$@"
