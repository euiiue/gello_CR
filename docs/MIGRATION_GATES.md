# V2 Migration Gates

## Offline gates

- legacy tests still pass
- V2 state-machine tests pass
- data contract remains 18D / 12D / three RGB / 20 Hz
- UI does not import vendor SDKs
- device adapters do not import UI

## Hardware gates

1. GELLO J1..J7 read-only feedback
2. O6 read-only position/fault feedback
3. NRC 6001/7000 connect only
4. CR3A clear-error and power-on
5. low-speed single-joint ServoJ
6. GELLO J1..J6 relative mapping
7. GELLO J7 -> O6
8. 60 s full teleop
9. episode success/failure/discard
10. five saved episodes at 20 Hz, all three RGB streams
11. old-vs-new dataset schema equivalence
12. communication/camera fault safe-stop behavior
13. E-stop reset must not auto-resume ServoJ
