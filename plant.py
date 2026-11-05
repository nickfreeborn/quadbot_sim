import numpy as np
from scipy.signal import cont2discrete

import config


def get_continuous_model():

    M = config.M
    m = config.m
    l = config.l
    I = config.I
    b = config.b
    g = config.g

    delta = (M + m) * (I + m * l**2) - (m * l)**2

    A = np.array([
        [0, 1, 0, 0],
        [0, -b*(I + m*l**2)/delta,
         -(m**2 * l**2 * g)/delta, 0],
        [0, 0, 0, 1],
        [0, b*m*l/delta,
         (M + m)*m*g*l/delta, 0]
    ])

    B = np.array([
        [0],
        [(I + m*l**2)/delta],
        [0],
        [-m*l/delta]
    ])

    C = np.eye(4)
    D = np.zeros((4, 1))

    return A, B, C, D


def get_discrete_model(dt):

    A, B, C, D = get_continuous_model()

    Ad, Bd, Cd, Dd, _ = cont2discrete(
        (A, B, C, D),
        dt,
        method="zoh"
    )

    return Ad, Bd, Cd, Dd