
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_new_bootstrap_has_no_qt_dependency() -> None:
    for name in ('runtime_factory.py', 'sample_source.py'):
        source = (ROOT / 'src/gello_cr/bootstrap' / name).read_text(encoding='utf-8')
        assert 'PyQt5' not in source
        assert 'PySide6' not in source
        assert 'TEST_INEXBOT' not in source


def test_factory_uses_lazy_legacy_module_loading() -> None:
    source = (
        ROOT / 'src/gello_cr/bootstrap/runtime_factory.py'
    ).read_text(encoding='utf-8')
    assert "importlib.import_module('teleop_runtime')" in source
    assert "importlib.import_module('lerobot_recorder')" in source


def test_cr3a_lifecycle_is_explicit_not_constructor_side_effect() -> None:
    source = (
        ROOT / 'src/gello_cr/bootstrap/runtime_factory.py'
    ).read_text(encoding='utf-8')
    init_block = source.split('class Cr3aLifecycle:', 1)[1].split(
        '    def _event(', 1
    )[0]
    assert '.connect(' not in init_block
    assert '.power_on(' not in init_block


def test_recording_sample_source_replaces_legacy_qt_sample_provider_contract() -> None:
    source = (
        ROOT / 'src/gello_cr/bootstrap/sample_source.py'
    ).read_text(encoding='utf-8')
    assert "sample = self._teleop_engine.dataset_sample()" in source
    assert "sample['image_base_rgb']" in source
    assert "sample['image_wrist_rgb']" in source
    assert "sample['image_roi_rgb']" in source
