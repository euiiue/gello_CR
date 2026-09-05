"""Frozen LeRobot/OpenPI data contract during V2 migration."""

STATE_DIM = 18
ACTION_DIM = 12
FPS = 20
IMAGE_SIZE = (224, 224)
CAMERA_KEYS = ("base_rgb", "wrist_rgb", "roi_rgb")

STATE_FIELDS = (
    "cr5.j1.rad",
    "cr5.j2.rad",
    "cr5.j3.rad",
    "cr5.j4.rad",
    "cr5.j5.rad",
    "cr5.j6.rad",
    "cr5.tcp.x.m",
    "cr5.tcp.y.m",
    "cr5.tcp.z.m",
    "cr5.tcp.roll.rad",
    "cr5.tcp.pitch.rad",
    "cr5.tcp.yaw.rad",
    "o6.thumb_flex.position",
    "o6.thumb_yaw.position",
    "o6.index_flex.position",
    "o6.middle_flex.position",
    "o6.ring_flex.position",
    "o6.little_flex.position",
)

ACTION_FIELDS = (
    "cr5.delta_tcp.x.m",
    "cr5.delta_tcp.y.m",
    "cr5.delta_tcp.z.m",
    "cr5.delta_tcp.roll.rad",
    "cr5.delta_tcp.pitch.rad",
    "cr5.delta_tcp.yaw.rad",
    "o6.thumb_flex.command",
    "o6.thumb_yaw.command",
    "o6.index_flex.command",
    "o6.middle_flex.command",
    "o6.ring_flex.command",
    "o6.little_flex.command",
)

assert len(STATE_FIELDS) == STATE_DIM
assert len(ACTION_FIELDS) == ACTION_DIM
