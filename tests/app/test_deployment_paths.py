from types import SimpleNamespace

import pytest
from PySide6 import QtWidgets

from gello_cr.app.config_store import TeleopConfigStore
from gello_cr.bootstrap.runtime_factory import Cr3aLifecycle, RuntimeFactoryPaths
from gello_cr.core.state_machine import WorkflowState
from gello_cr.ui import __main__ as entrypoint
from gello_cr.ui import session


@pytest.mark.parametrize("edit_root", [False, True], ids=["display", "save"])
def test_collection_root_uses_config_directory_from_another_cwd(
    tmp_path, monkeypatch, edit_root
):
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    store = TeleopConfigStore(config_dir / "workbench.json")
    store.save()
    elsewhere = tmp_path / "launch" / "nested"
    elsewhere.mkdir(parents=True)
    monkeypatch.chdir(elsewhere)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = QtWidgets.QMainWindow()
    window.collection = SimpleNamespace(
        task_edit=QtWidgets.QLineEdit(window),
        description_edit=QtWidgets.QLineEdit(window),
        root_edit=QtWidgets.QLineEdit(window),
    )
    operator = SimpleNamespace(
        runtime=SimpleNamespace(store=store),
        backend=SimpleNamespace(service=SimpleNamespace(state=WorkflowState.OFFLINE)),
        cameras=SimpleNamespace(running=False),
        close=lambda: None,
    )
    monkeypatch.setattr(entrypoint, "build_operator_application", lambda root: operator)
    monkeypatch.setattr(session, "create_operator_window", lambda *a, **kw: window)
    monkeypatch.setattr(QtWidgets, "QApplication", lambda argv: app)

    def interact():
        if edit_root:
            window.collection.root_edit.setText("../dataset/selected")
            window.collection.root_edit.editingFinished.emit()
            expected = str(tmp_path / "dataset/selected")
            saved = TeleopConfigStore.from_file(store.path)
            assert saved.data["dataset"]["root"] == expected
        else:
            expected = str(tmp_path / "dataset/train")
        assert window.collection.root_edit.text() == expected
        return 0

    monkeypatch.setattr(app, "exec", interact)
    try:
        assert entrypoint.main(["--repo-root", str(tmp_path)]) == 0
    finally:
        app.aboutToQuit.disconnect(operator.close)
        window.close()


def test_cr3a_sdk_root_uses_config_directory_from_another_cwd(tmp_path, monkeypatch):
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    store = TeleopConfigStore(config_dir / "workbench.json")
    store.data["robot"]["sdk_root"] = "../vendor/custom-nrc"
    elsewhere = tmp_path / "launch" / "nested"
    elsewhere.mkdir(parents=True)
    monkeypatch.chdir(elsewhere)
    lifecycle = Cr3aLifecycle(
        store=store,
        teleop_engine=None,
        paths=RuntimeFactoryPaths.from_repo(tmp_path),
    )

    assert lifecycle._robot_config().sdk_root == str(tmp_path / "vendor/custom-nrc")
    assert store.data["robot"]["sdk_root"] == "../vendor/custom-nrc"
