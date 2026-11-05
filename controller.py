import numpy as np
from scipy.linalg import solve_discrete_are

import config


def design_lqr(Ad, Bd):

    Q = config.Q
    R = config.R

    P = solve_discrete_are(Ad, Bd, Q, R)

    K = np.linalg.solve(
        R + Bd.T @ P @ Bd,
        Bd.T @ P @ Ad
    )

    return K


def calculate_control(K, state):

    u = -K @ state

    # Motor force saturation
    u = np.clip(
        u,
        -config.MAX_FORCE,
        config.MAX_FORCE
    )

    return float(u[0])