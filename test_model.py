import numpy as np

from plant import get_continuous_model, get_discrete_model
from controller import design_lqr

import config


A, B, C, D = get_continuous_model()

Ad, Bd, Cd, Dd = get_discrete_model(config.DT)

K = design_lqr(Ad, Bd)

print("\nContinuous A:")
print(A)

print("\nContinuous B:")
print(B)

print("\nDiscrete Ad:")
print(Ad)

print("\nDiscrete Bd:")
print(Bd)

print("\nLQR gain K:")
print(K)

print("\nOpen-loop eigenvalues:")
print(np.linalg.eigvals(A))

print("\nClosed-loop discrete eigenvalues:")
print(np.linalg.eigvals(Ad - Bd @ K))