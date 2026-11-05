import numpy as np
from scipy.signal import cont2discrete

import config


def get_continuous_model():

    M = config.BODY_MASS
    mw = config.WHEEL_MASS_TOTAL
    r = config.WHEEL_RADIUS
    l = config.BODY_COM_HEIGHT
    I = config.BODY_INERTIA
    Jw = config.WHEEL_INERTIA_TOTAL
    g = config.GRAVITY

    bs = config.LINEAR_FRICTION
    bt = config.ANGULAR_FRICTION

    # Effective translational mass
    Mt = M + mw + Jw / r**2

    # Coupled mass matrix at upright equilibrium
    mass_matrix = np.array([
        [Mt, M * l],
        [M * l, I + M * l**2]
    ])

    mass_inv = np.linalg.inv(mass_matrix)

    # Linearized acceleration equations
    dynamics_matrix = np.array([
        [0.0, -bs, M * g * l, -0.0],
        [0.0, -0.0, 0.0, -bt]
    ])

    input_matrix = np.array([
        [1.0 / r],
        [-1.0]
    ])

    # Solve for accelerations
    acceleration_A = mass_inv @ dynamics_matrix
    acceleration_B = mass_inv @ input_matrix

    A = np.array([
        [0, 1, 0, 0],
        acceleration_A[0],
        [0, 0, 0, 1],
        acceleration_A[1]
    ], dtype=float)

    B = np.array([
        [0],
        [acceleration_B[0, 0]],
        [0],
        [acceleration_B[1, 0]]
    ])

    # Measurements: wheel position and body angle
    C = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0]
    ])

    D = np.zeros((2, 1))

    return A, B, C, D


def get_discrete_model(dt):

    A, B, C, D = get_continuous_model()

    Ad, Bd, Cd, Dd, _ = cont2discrete(
        (A, B, C, D),
        dt,
        method="zoh"
    )

    return Ad, Bd, Cd, Dd