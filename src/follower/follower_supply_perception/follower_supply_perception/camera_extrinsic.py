"""Fixed Follower camera mounting and optical-frame transform definitions."""

from dataclasses import dataclass
from math import isfinite, pi
from typing import Sequence, Tuple


@dataclass(frozen=True)
class FixedTransform:
    """One repository-owned static transform expressed as xyz and fixed-axis RPY."""

    parent_frame: str
    child_frame: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]

    def validate(self) -> None:
        """Reject malformed transforms before they reach the ROS graph."""
        if not self.parent_frame or not self.child_frame:
            raise ValueError("Static TF frame names must not be empty")
        if self.parent_frame == self.child_frame:
            raise ValueError("A static TF parent and child must be different")
        if not all(isfinite(value) for value in self.xyz + self.rpy):
            raise ValueError("Static TF values must be finite")

    def publisher_arguments(self) -> Tuple[str, ...]:
        """Return ROS 2 Humble ``static_transform_publisher`` arguments."""
        self.validate()
        return (
            "--x",
            str(self.xyz[0]),
            "--y",
            str(self.xyz[1]),
            "--z",
            str(self.xyz[2]),
            "--roll",
            str(self.rpy[0]),
            "--pitch",
            str(self.rpy[1]),
            "--yaw",
            str(self.rpy[2]),
            "--frame-id",
            self.parent_frame,
            "--child-frame-id",
            self.child_frame,
        )


BASE_FRAME = "base_link"
CAMERA_BODY_FRAME = "follower/follower_camera_link"
CAMERA_OPTICAL_FRAME = "follower/follower_camera_optical_frame"

# Measured camera-body origin in base_link. The camera is provisionally treated
# as level and forward-facing in robot body coordinates.
BASE_TO_CAMERA_BODY = FixedTransform(
    parent_frame=BASE_FRAME,
    child_frame=CAMERA_BODY_FRAME,
    xyz=(0.042, 0.01, 0.120),
    rpy=(0.0, 0.0, -0.10),
)

# REP-103 camera convention: body +X forward/+Y left/+Z up becomes optical
# +X right/+Y down/+Z forward. Keep this rotation separate from the measured
# mounting translation.
CAMERA_BODY_TO_OPTICAL = FixedTransform(
    parent_frame=CAMERA_BODY_FRAME,
    child_frame=CAMERA_OPTICAL_FRAME,
    xyz=(0.0, 0.0, 0.0),
    rpy=(-pi / 2.0, 0.0, -pi / 2.0),
)

FOLLOWER_CAMERA_TRANSFORMS = (BASE_TO_CAMERA_BODY, CAMERA_BODY_TO_OPTICAL)


def validate_transform_chain(transforms: Sequence[FixedTransform]) -> None:
    """Validate child uniqueness and an ordered parent-to-child chain."""
    if not transforms:
        raise ValueError("Static TF chain must not be empty")
    child_frames = set()
    for index, transform in enumerate(transforms):
        transform.validate()
        if transform.child_frame in child_frames:
            raise ValueError("Each static TF child must have exactly one publisher")
        child_frames.add(transform.child_frame)
        if index > 0 and transform.parent_frame != transforms[index - 1].child_frame:
            raise ValueError("Static transforms must form one ordered chain")


validate_transform_chain(FOLLOWER_CAMERA_TRANSFORMS)
