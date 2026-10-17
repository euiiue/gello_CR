"""One-time conversion into TrajectoryStore; never connects or rewrites inputs."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from gello_cr.app.config_store import TeleopConfigStore
from gello_cr.trajectory import TrajectoryStore
from gello_cr.trajectory.adapters import LegacyJsonAdapter


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", type=Path)
    parser.add_argument(
        "--config", type=Path, default=Path(__file__).resolve().parents[1] / "config/workbench.json"
    )
    parser.add_argument("--root", type=Path, help="Override the trajectory store output directory")
    args = parser.parse_args(argv)
    config = TeleopConfigStore.from_file(args.config)
    root = args.root or config.resolve_path(config.data["trajectory"]["root"])
    store = TrajectoryStore(root)
    for path in args.files:
        print(store.save(LegacyJsonAdapter.load(path)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
