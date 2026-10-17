#!/usr/bin/env python3
"""Source-checkout entry point for the shared replay CLI."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from gello_cr.cli.replay_trajectory import main

if __name__ == "__main__":
    raise SystemExit(main())
