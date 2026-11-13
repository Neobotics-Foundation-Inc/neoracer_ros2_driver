"""
Speeds for the calibration drives, stated in m/s and sent as /drive commands.

The calibration tools publish /drive, which reaches the ESP32 through the mux,
the throttle node and the controller. The throttle node scales /drive by
max_speed_forward (max_speed_backward when reversing) onto /motor, and the
controller sends /motor times max_speed_mps to the ESP32 as a speed target in
m/s. The ESP32's speed loop has no floor, so the target is the speed the car
drives: on the dev car a 0.5 m/s target held about 0.54 m/s on the floor
(2026-11-12).

Upstream's tools give their speeds as /drive values for a firmware that maps
any command onto 0.5 to 1.0 m/s. Those values mean something else here, so
each NeoRacer tool states its speed in m/s and converts it with the car's own
max_speed_mps and throttle caps, read from the config the stack loaded with
the car profile on top.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from neoracer_ros2_driver import car_profile

# The nodes' own fallbacks, used when a YAML file does not set the value.
DEFAULT_MAX_SPEED_MPS = 6.0
DEFAULT_MAX_FORWARD = 0.17
DEFAULT_MAX_BACKWARD = 0.24


@dataclass(frozen=True)
class DriveLimits:
    """How a /drive speed becomes the ESP32's speed target on this car."""

    max_speed_mps: float  # controller_node: /motor 1.0 is this many m/s
    max_forward: float  # throttle_node: /drive to /motor scale going forward
    max_backward: float  # throttle_node: the same in reverse

    def mps_per_command(self, forward: bool) -> float:
        """Return the m/s that a /drive speed of 1.0 gives in this direction."""
        return self.max_speed_mps * (self.max_forward if forward else self.max_backward)


def drive_limits(config_dir: Path | None = None) -> DriveLimits:
    """Return the car's limits from controller.yaml and throttle.yaml plus the profile."""
    if config_dir is None:
        from calibrate_common import install_config_dir, source_config_dir

        config_dir = install_config_dir() or source_config_dir()
    controller = car_profile.node_params('controller', 'controller_node', config_dir)
    throttle = car_profile.node_params('throttle', 'throttle_node', config_dir)
    return DriveLimits(
        float(controller.get('max_speed_mps', DEFAULT_MAX_SPEED_MPS)),
        float(throttle.get('max_speed_forward', DEFAULT_MAX_FORWARD)),
        float(throttle.get('max_speed_backward', DEFAULT_MAX_BACKWARD)),
    )


def speed_command(mps: float, limits: DriveLimits) -> float:
    """
    Return the /drive speed that makes the controller send `mps` to the ESP32.

    Raises ValueError when the car's limits cannot reach `mps`.
    """
    if mps == 0.0:
        return 0.0
    scale = limits.mps_per_command(mps > 0.0)
    if scale <= 0.0:
        raise ValueError(f'this car cannot drive {mps:+g} m/s: the throttle cap is 0')
    command = mps / scale
    if abs(command) > 1.0:
        raise ValueError(
            f'{mps:+g} m/s needs /drive {command:+.3f}, beyond 1.0 with '
            f'max_speed_mps {limits.max_speed_mps:g} and the throttle caps'
        )
    return command


def drive_segments(
    segments_mps: Sequence[tuple[float, float, float]], limits: DriveLimits
) -> list[tuple[float, float, float]]:
    """Return (speed, steering, seconds) segments with each speed as a /drive value."""
    return [
        (speed_command(speed, limits), steering, seconds)
        for speed, steering, seconds in segments_mps
    ]
