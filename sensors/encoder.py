
"""
sensors/encoder.py

Simulated incremental wheel encoders for a two-wheeled
self-balancing robot.

Features:
    - Left and right wheel encoder simulation
    - Configurable counts per revolution
    - Gearbox ratio support
    - Wheel angular position estimation
    - Linear displacement estimation
    - Velocity estimation
    - Optional measurement noise

Coordinate convention:
    Positive wheel rotation produces positive forward displacement.

Encoder resolution:
    COUNTS_PER_REV represents the number of decoded counts per
    motor shaft revolution before applying the gearbox ratio.

    For quadrature encoders, ensure this matches the decoding
    convention (x1, x2 or x4).
"""

import numpy as np


class Encoder:
    def __init__(
        self,
        wheel_radius,
        counts_per_rev=360,
        gear_ratio=1.0,
        noise_std=0.0,
    ):
        """
        Initialise an encoder model.

        Args:
            wheel_radius: Wheel radius in metres.
            counts_per_rev: Encoder counts per motor shaft revolution.
            gear_ratio: Motor revolutions per wheel revolution.
            noise_std: Standard deviation of position noise in metres.
        """

        if wheel_radius <= 0:
            raise ValueError("wheel_radius must be positive")

        if counts_per_rev <= 0:
            raise ValueError("counts_per_rev must be positive")

        if gear_ratio <= 0:
            raise ValueError("gear_ratio must be positive")

        if noise_std < 0:
            raise ValueError("noise_std cannot be negative")

        self.wheel_radius = wheel_radius
        self.counts_per_rev = counts_per_rev
        self.gear_ratio = gear_ratio
        self.noise_std = noise_std

        # Encoder counts per wheel revolution
        self.wheel_counts_per_rev = counts_per_rev * gear_ratio

        # Distance travelled per encoder count
        self.distance_per_count = (
            2.0 * np.pi * wheel_radius
            / self.wheel_counts_per_rev
        )

        # Internal encoder state
        self.count = 0
        self.previous_count = 0

        self.position = 0.0
        self.velocity = 0.0

    def update(self, wheel_angle, dt):
        """
        Update the simulated encoder from the actual wheel angle.

        Args:
            wheel_angle: Actual wheel angle in radians.
            dt: Simulation timestep in seconds.

        Returns:
            Dictionary containing:
                count: Quantised encoder count
                angle: Estimated wheel angle (rad)
                position: Estimated linear displacement (m)
                velocity: Estimated linear velocity (m/s)
        """

        if dt <= 0:
            raise ValueError("dt must be positive")

        # Convert wheel angle to encoder counts
        raw_count = (
            wheel_angle
            / (2.0 * np.pi)
            * self.wheel_counts_per_rev
        )

        # Quantise to integer encoder counts
        self.count = int(np.round(raw_count))

        # Convert counts back to wheel angle
        measured_angle = (
            self.count
            / self.wheel_counts_per_rev
            * 2.0 * np.pi
        )

        # Convert angular position to linear displacement
        measured_position = measured_angle * self.wheel_radius

        # Calculate velocity from position difference
        measured_velocity = (
            measured_position - self.position
        ) / dt

        # Add optional measurement noise
        if self.noise_std > 0:
            measured_position += np.random.normal(
                0.0, self.noise_std
            )

        self.velocity = measured_velocity
        self.position = measured_position

        return {
            "count": self.count,
            "angle": measured_angle,
            "position": measured_position,
            "velocity": measured_velocity,
        }

    def reset(self):
        """Reset encoder state to zero."""

        self.count = 0
        self.previous_count = 0
        self.position = 0.0
        self.velocity = 0.0


class DualEncoder:
    """
    Simulates left and right wheel encoders.
    """

    def __init__(
        self,
        wheel_radius,
        counts_per_rev=360,
        gear_ratio=1.0,
        noise_std=0.0,
    ):

        self.left = Encoder(
            wheel_radius=wheel_radius,
            counts_per_rev=counts_per_rev,
            gear_ratio=gear_ratio,
            noise_std=noise_std,
        )

        self.right = Encoder(
            wheel_radius=wheel_radius,
            counts_per_rev=counts_per_rev,
            gear_ratio=gear_ratio,
            noise_std=noise_std,
        )

    def update(self, left_angle, right_angle, dt):
        """
        Update both encoders.

        Args:
            left_angle: Left wheel angle in radians.
            right_angle: Right wheel angle in radians.
            dt: Simulation timestep in seconds.

        Returns:
            Dictionary containing left and right measurements.
        """

        left_data = self.left.update(left_angle, dt)
        right_data = self.right.update(right_angle, dt)

        # Average wheel displacement and velocity
        centre_position = (
            left_data["position"] + right_data["position"]
        ) / 2.0

        centre_velocity = (
            left_data["velocity"] + right_data["velocity"]
        ) / 2.0

        return {
            "left": left_data,
            "right": right_data,
            "centre_position": centre_position,
            "centre_velocity": centre_velocity,
        }

    def reset(self):
        """Reset both encoders."""

        self.left.reset()
        self.right.reset()
