"""Command-range helpers shared by the drive-path nodes."""


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    return max(low, min(high, value))
