# Phase 3.3 - GELLO J7 -> O6 mapping

This phase extracts and integrates both existing J7 policies in one step.

## Joint mode

- J7 < 0.5 -> 张开手
- J7 >= 0.5 -> 抓取

## Cartesian GELLO modes

- clamp J7 to [0, 1]
- interpolate O6 `open` -> `closed`
- Python `round()`
- send only when max motor delta >= `command_deadband`

New module:

`src/gello_cr/control/hand_mapping.py`
