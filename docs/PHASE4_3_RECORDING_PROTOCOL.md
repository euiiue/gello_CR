# Phase 4.3 - LeRobot Unix-socket protocol extraction

Phase 4.3 moves the packet wire format from `lerobot_recorder.py` to:

`src/gello_cr/recording/protocol.py`

The wire format is unchanged:

1. 4-byte unsigned big-endian JSON header length
2. UTF-8 compact JSON
3. optional raw byte payload

Limits are unchanged:

- JSON header <= 4 MiB
- raw payload <= 16 MiB

`raw_size` remains a transport-only JSON field and is removed by the receiver
before the payload is returned.

The root recorder imports the moved functions as `_read_exact`,
`_send_packet`, and `_receive_packet`, preserving the old private names.

Not changed:

- `socket.socketpair()`
- worker subprocess ownership
- request locking
- socket timeout handling
- permanent communication-error latch after timeout/protocol failure
- Episode lifecycle
- 20 Hz sample loop
- frame content/order
- LeRobot schema
