import numpy as np

# Physical parameters
M = 0.5       # Cart mass (kg)
m = 0.2       # Pendulum mass (kg)
l = 0.3       # COM distance from pivot (m)
I = 0.006     # Pendulum inertia about COM (kg m^2)
b = 0.1       # Cart friction (N s/m)
g = 9.81      # Gravity (m/s^2)

# Simulation parameters
DT = 0.002          # 500 Hz
SIM_TIME = 5.0

# LQR weighting matrices
Q = np.diag([1.0, 0.1, 100.0, 1.0])
R = np.array([[0.1]])

# Actuator constraints
MAX_FORCE = 10.0