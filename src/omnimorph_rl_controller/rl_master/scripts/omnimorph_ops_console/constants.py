from __future__ import annotations

TOPIC_ROBOT_STATE = "/omnimorph/rl/state"
TOPIC_TELEOP = "/omnimorph/rl/teleop"
TOPIC_MODE_CONTROL = "/omnimorph/rl/mode_control"
TOPIC_CAMERA_COLOR = "/omnimorph/camera/color/image_raw"
TOPIC_CAMERA_DEPTH = "/omnimorph/camera/depth/image_raw"
TOPIC_CAMERA_FEATURES = "/omnimorph/camera/features"

PROTOCOL_V2_MAGIC = 240426
PROTOCOL_VERSION = 2
PAYLOAD_ROBOT_STATE = 2
STATE_HEADER_COUNT = 4
BASE_STATE_TAIL_COUNT = 3 + 4 + 3

CTRL_START_LC = 10
CTRL_STOP = 11
CTRL_ZERO = 12
CTRL_ESTOP = 13
CTRL_START_MODE_BASE = 1000
CTRL_SET_MODE_BASE = 2000
