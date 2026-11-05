import numpy as np
from scipy.linalg import solve_discrete_are

import config


def design_lqr(Ad, Bd):

    P = solve_discrete_are(
        Ad,
        Bd,
        config.Q,
        config.R
    )

    K = np.linalg.solve(
        config.R + Bd.T @ P @ Bd,
        Bd.T @ P @ Ad
    )

    return K


def calculate_control(K, state):

    torque = -K @ state

    torque = np.clip(
        torque,
        -config.MAX_TORQUE,
        config.MAX_TORQUE
    )

    return float(torque[0])