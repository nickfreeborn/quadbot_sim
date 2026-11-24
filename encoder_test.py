import numpy as np

from sensors.encoder import DualEncoder

# Robot configuration
WHEEL_RADIUS = 0.0325
ENCODER_CPR = 360
GEAR_RATIO = 30.0

encoder = DualEncoder(
    wheel_radius=WHEEL_RADIUS,
    counts_per_rev=ENCODER_CPR,
    gear_ratio=GEAR_RATIO,
    noise_std=0.0,
)

dt = 0.002  # 500 Hz

for i in range(10):
    time = i * dt

    # Example wheel rotation
    left_angle = 2.0 * time
    right_angle = 2.0 * time

    measurements = encoder.update(
        left_angle,
        right_angle,
        dt,
    )

    print(
        f"Position: {measurements['centre_position']:.4f} m | "
        f"Velocity: {measurements['centre_velocity']:.4f} m/s"
    )
