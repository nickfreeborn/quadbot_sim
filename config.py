import numpy as np

# Robot physical parameters
BODY_MASS = 0.50       # kg

# Wheel properties
WHEEL_MASS_TOTAL = 0.08       # kg, combined mass of both wheels
WHEEL_DIAMETER = 0.065        # m
WHEEL_RADIUS = WHEEL_DIAMETER / 2

# Approximate wheel inertia (solid discs)
WHEEL_INERTIA_TOTAL = (
    0.5 * WHEEL_MASS_TOTAL * WHEEL_RADIUS**2
)

# Initial assumptions
BODY_COM_HEIGHT = 0.10  # m, placeholder
BODY_INERTIA = 0.005    # kg m^2, placeholder
GRAVITY = 9.81
LINEAR_FRICTION = 0.1 # N
ANGULAR_FRICTION = 3 # N

# Simulation
CONTROL_FREQ = 500.0
DT = 1.0 / CONTROL_FREQ
SIM_TIME = 10.0

# BMI088
IMU_INTERFACE = "SPI"  
IMU_FREQ = 500.0
ACCEL_RANGE_G = 6
GYRO_RANGE_DPS = 500

# LQR
Q = np.diag([10.0, 1.0, 150.0, 2.0])
R = np.array([[0.1]])