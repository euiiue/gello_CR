
"""Executable entry point for the new PySide6 operator UI.

Examples:

    PYTHONPATH=src python -m gello_cr.ui --check-only
    PYTHONPATH=src python -m gello_cr.ui

Neither mode automatically connects hardware.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from gello_cr.bootstrap import build_operator_application
from gello_cr.core.state_machine import WorkflowState


def _default_repo_root() -> Path:
    configured = os.environ.get("GELLO_CR_REPO_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()

    cwd = Path.cwd().resolve()
    if (cwd / "config" / "roarm_cr5_teleop.json").is_file():
        return cwd

    package_root = Path(__file__).resolve().parents[3]
    return package_root


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="GELLO / CR3A / O6 PySide6 operator UI",
    )
    parser.add_argument(
        "--repo-root",
        default=str(_default_repo_root()),
        help="gello_CR repository root",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help=(
            "construct the runtime/backend and exit without importing "
            "PySide6 or connecting hardware"
        ),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    repo_root = Path(args.repo_root).expanduser().resolve()

    operator = build_operator_application(repo_root)
    try:
        if operator.backend.service.state is not WorkflowState.OFFLINE:
            raise RuntimeError("new operator application must start OFFLINE")
        if operator.cameras.running:
            raise RuntimeError("camera service unexpectedly started")

        if args.check_only:
            runtime = operator.runtime
            master_type = str(runtime.store.data["master"]["type"])
            print(
                "Phase 6.5 operator bootstrap check: PASS\n"
                f"repo_root={repo_root}\n"
                f"workflow={operator.backend.service.state.name}\n"
                f"master={master_type}\n"
                "cameras=stopped\n"
                "hardware_connections=not_started"
            )
            return 0

        try:
            from PySide6.QtWidgets import QApplication
        except ImportError as exc:
            raise SystemExit(
                "PySide6 is not installed. Install the project UI extra "
                "before launching the new operator window."
            ) from exc

        from gello_cr.ui.session import create_operator_window

        qt_app = QApplication(sys.argv if argv is None else [sys.argv[0], *argv])
        window = create_operator_window(operator.backend)

        dataset_cfg = operator.runtime.store.data["dataset"]
        window.task_edit.setText(str(dataset_cfg["task"]))
        window.root_edit.setText(str(dataset_cfg["root"]))
        window.statusBar().showMessage(
            "OFFLINE · cameras stopped · no hardware auto-connect",
            15000,
        )

        qt_app.aboutToQuit.connect(operator.close)
        window.show()
        return int(qt_app.exec())
    finally:
        if args.check_only:
            operator.close()


if __name__ == "__main__":
    raise SystemExit(main())
