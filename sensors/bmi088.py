import numpy as np
import config


class BMI088Simulator:

    def __init__(self):

        self.dt = config.IMU_DT

        # Approximate RMS noise within configured bandwidth
        self.accel_sigma = (
            config.ACCEL_NOISE_DENSITY
            * np.sqrt(config.IMU_BANDWIDTH)
        )

        self.gyro_sigma = (
            config.GYRO_NOISE_DENSITY
            * np.sqrt(config.IMU_BANDWIDTH)
        )

    def read(self, true_accel_g, true_gyro_dps):

        accel_noise = np.random.normal(
            0, self.accel_sigma
        )

        gyro_noise = np.random.normal(
            0, self.gyro_sigma
        )

        accel = true_accel_g + accel_noise
        gyro = (
            true_gyro_dps
            + config.GYRO_BIAS_DPS
            + gyro_noise
        )

        return accel, gyro