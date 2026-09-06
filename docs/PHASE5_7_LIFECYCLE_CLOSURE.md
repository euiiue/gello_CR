# Phase 5.7 - Lifecycle closure

Phase 5.7 completes the core ApplicationService lifecycle callbacks for the
legacy Qt bridge:

- DISCONNECT
- POWER_OFF
- RESET_FAULT
- RESET_ESTOP

No new NRC primitive is introduced.

## POWER_OFF

Application `POWER_OFF` uses the existing `NrcRobotSession.power_off()` state
machine. Before power-off, TeleopEngine performs its existing software stop so
legacy preset/replay command producers cannot remain active.

The command is only legal from `ROBOT_ENABLED`.

## RESET_FAULT / RESET_ESTOP

The callback uses the existing validated NRC `power_on()` sequence, which
already clears servo alarm when state=2 and finishes in servo state=3.

Only after servo state 3 is verified does the callback call:

`TeleopEngine.acknowledge_fault()`

That new runtime method only clears the local fault latch and returns the
runtime to `idle`. It sends no motion command and never calls `start_follow()`.

Application reset then transitions to its safe recovery target. For a fault or
E-stop that interrupted TELEOP/RECORDING, that target remains ROBOT_ENABLED.

## DISCONNECT

The application disconnect callback:

1. detaches the expected NRC adapter from TeleopEngine;
2. closes `Cr3aDevice` (best-effort stop + 6001/7000 disconnect);
3. clears the legacy Qt connection references and flags.

No automatic reconnect occurs.

## Legacy robot-tab buttons

The older robot tab is now also application-gated for connect, power-on,
power-off, clear/reset, start-follow and software stop so it cannot bypass the
new workflow state.
