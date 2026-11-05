import numpy as np
import matplotlib.pyplot as plt

import config
from plant import get_discrete_model
from controller import design_lqr, calculate_control


def simulate():

    dt = config.DT
    time = np.arange(0, config.SIM_TIME, dt)

    # Obtain discrete state-space model
    Ad, Bd, Cd, Dd = get_discrete_model(dt)

    # Calculate LQR gain
    K = design_lqr(Ad, Bd)

    print("Discrete LQR gain:")
    print(K)

    # Initial state:
    # [position, velocity, angle, angular_velocity]
    x = np.array([
        [0.0],
        [0.0],
        [np.deg2rad(8.0)],
        [0.0]
    ])

    states = []
    controls = []

    # Simulation loop
    for t in time:

        u = calculate_control(K, x)

        # Discrete state update
        x = Ad @ x + Bd * u

        states.append(x.flatten())
        controls.append(u)

    states = np.array(states)
    controls = np.array(controls)

    # Convert angle to degrees
    angle_deg = np.rad2deg(states[:, 2])

    # Plot results
    fig, axes = plt.subplots(3, 1, figsize=(10, 9))

    axes[0].plot(time, states[:, 0])
    axes[0].set_ylabel("Position (m)")
    axes[0].grid()

    axes[1].plot(time, angle_deg)
    axes[1].set_ylabel("Pendulum angle (deg)")
    axes[1].grid()

    axes[2].plot(time, controls)
    axes[2].set_ylabel("Control force (N)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    simulate()