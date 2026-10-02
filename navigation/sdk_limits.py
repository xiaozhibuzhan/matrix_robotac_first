"""ZSL-1 move() deadbands documented by the bundled official SDK."""
import math

MIN_FORWARD_SPEED=0.05
MIN_YAW_RATE=0.02


def validate_sdk_velocity(v,w):
    """Reject unsupported commands; never increase speed in the SDK transport."""
    if not all(math.isfinite(n) for n in (v,w)):
        raise ValueError('Nonfinite SDK velocity')
    if v<0 or (v!=0 and v<MIN_FORWARD_SPEED):
        raise ValueError(f'SDK vx={v}: expected 0 or >= {MIN_FORWARD_SPEED} m/s')
    if w!=0 and abs(w)<MIN_YAW_RATE:
        raise ValueError(f'SDK yaw={w}: expected 0 or abs >= {MIN_YAW_RATE} rad/s')
