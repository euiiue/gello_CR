
"""No-hardware preview launcher for the Phase 6 PySide6 shell.

This preview deliberately does not import or instantiate NRC/GELLO/O6/cameras.
Command requests are logged to stdout and do not change application state.
"""

from __future__ import annotations

import sys

from gello_cr.app import ApplicationService
from gello_cr.ui.command_port import CallbackCommandPort
from gello_cr.ui.main_window import OperatorMainWindow
from gello_cr.ui.presenter import OperatorUiPresenter


def main() -> int:
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError as exc:
        raise SystemExit(
            "PySide6 is not installed. Install the project UI extra before "
            "running the Phase 6 preview."
        ) from exc

    app = QApplication(sys.argv)
    service = ApplicationService()

    presenter = OperatorUiPresenter(
        service,
        runtime_snapshot=lambda: {"state": "idle"},
        recorder_snapshot=lambda: {
            "session_active": False,
            "episode_active": False,
            "buffered_frames": 0,
            "saved_episodes": 0,
            "error": "",
            "quality": {"needs_review": False},
        },
    )

    def log_request(request) -> None:
        print(
            f"[Phase6 preview] command={request.command.name} "
            f"payload={dict(request.payload)}"
        )

    window = OperatorMainWindow(
        presenter,
        CallbackCommandPort(log_request),
    )
    window.setWindowTitle(
        "GELLO · CR3A · O6 Operator — Phase 6.1 Preview (No Hardware)"
    )
    window.show()
    return int(app.exec())


if __name__ == "__main__":
    raise SystemExit(main())
