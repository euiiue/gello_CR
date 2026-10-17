"""Print deployment diagnostics without constructing a runtime or loading SDKs."""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from gello_cr.app.config_store import TeleopConfigStore
from gello_cr.diagnostics import diagnostic_report


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "config/workbench.json")
    args = parser.parse_args()
    config = TeleopConfigStore.from_file(args.config)
    print(
        json.dumps(diagnostic_report(config.deployment_data(), root), ensure_ascii=False, indent=2)
    )


if __name__ == "__main__":
    main()
