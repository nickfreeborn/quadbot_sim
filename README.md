# quadbot_sim

A Python-based simulation environment for modelling, analysing, and developing the control system of a two-wheeled self-balancing robot.

The simulator is intended to support the development of the robot's embedded control software before deployment to an STM32 microcontroller. It provides a place to experiment with the robot's dynamics, sensor behaviour, state estimation, and feedback control algorithms.

## Project Overview

The simulated robot consists of:

- A rigid body mounted above two independently driven wheels.
- DC geared motors providing balancing and movement torque.
- Wheel encoders for measuring wheel rotation and displacement.
- A BMI088 inertial measurement unit (IMU) for measuring acceleration and angular velocity.
- A Kalman filter for estimating the robot's internal state.
- A Linear Quadratic Regulator (LQR) controller for maintaining balance.

The long-term goal is to develop and validate the control algorithms in Python before transferring them to an STM32-based embedded implementation.

## Features

Planned and developing features include:

- [x] Configurable robot parameters
- [ ] Two-wheeled robot dynamics model
- [ ] Nonlinear plant simulation
- [ ] Linearised state-space model
- [ ] LQR controller design
- [ ] BMI088 accelerometer and gyroscope simulation
- [ ] Wheel encoder simulation
- [ ] Kalman filter state estimation
- [ ] Motor torque and saturation modelling
- [ ] Simulation plotting and analysis
- [ ] Controller performance testing
- [ ] Comparison between simulated and real robot behaviour

## Repository Structure

```text
quadbot_sim/
│
├── config.py
│
├── plant.py
├── controller.py
├── main.py
│
├── sensors/
│   ├── __init__.py
│   ├── imu.py
│   └── encoder.py
│
├── estimation/
│   ├── __init__.py
│   └── kalman.py
│
├── results/
│
├── requirements.txt
├── README.md
└── .gitignore
```

### File Descriptions

| File | Description |
|---|---|
| `config.py` | Robot parameters and simulation settings |
| `plant.py` | Robot physical model and equations of motion |
| `controller.py` | LQR controller implementation |
| `main.py` | Simulation entry point |
| `sensors/imu.py` | Simulated BMI088 measurements |
| `sensors/encoder.py` | Simulated wheel encoder measurements |
| `estimation/kalman.py` | State estimation |
| `results/` | Simulation plots and generated results |

## Robot Parameters

The simulator uses configurable physical parameters. Initial values are estimates and should be updated as the physical robot is assembled and measured.

| Parameter | Initial value | Description |
|---|---:|---|
| Body mass | 0.50 kg | Main robot body |
| Total wheel mass | 0.08 kg | Combined mass of both wheels |
| Wheel diameter | 0.065 m | Wheel diameter |
| Wheel radius | 0.0325 m | Wheel radius |
| Centre of mass height | 0.10 m | Height above wheel axle |
| Body moment of inertia | 0.005 kg·m² | Body pitch inertia |
| Gravity | 9.81 m/s² | Gravitational acceleration |

These values are provisional and are not intended to represent measured hardware specifications.

All parameters should be defined centrally in `config.py`.

## System Model

The robot is modelled as a two-wheeled inverted pendulum.

The system has four primary states:

\[
x =
\begin{bmatrix}
s & \dot{s} & \theta & \dot{\theta}
\end{bmatrix}^{T}
\]

Where:

- \(s\): Linear wheel displacement
- \(\dot{s}\): Linear velocity
- \(\theta\): Body pitch angle
- \(\dot{\theta}\): Body pitch angular velocity

The control input represents the combined drive torque applied by the two motors, subject to the sign convention used in the plant model.

The initial objective is to stabilise the robot around the upright equilibrium:

\[
\theta = 0
\]

A linearised state-space model will be used for initial controller design, with a nonlinear model providing more realistic simulation and testing.

## Control System

### LQR Controller

The primary control strategy is Linear Quadratic Regulator (LQR) feedback.

The controller uses the estimated state vector to calculate the required motor torque:

\[
u = -Kx
\]

The feedback gain matrix \(K\) is calculated by minimising the quadratic cost function:

\[
J = \int_0^\infty
(x^TQx + u^TRu)\,dt
\]

Where:

- \(Q\) penalises deviations in the system states.
- \(R\) penalises control effort.
- \(K\) is the resulting optimal feedback gain matrix.

The weighting matrices will be tuned to balance stability, response speed, and motor effort.

### State Estimation

The robot will use a Kalman filter to estimate its internal state from simulated sensor measurements.

The estimator will combine:

- BMI088 gyroscope measurements.
- BMI088 accelerometer measurements.
- Wheel encoder measurements.

The purpose of state estimation is to obtain a more reliable estimate of body angle, angular velocity, wheel position, and wheel velocity than would be possible using individual sensor measurements alone.

## Sensor Simulation

### BMI088 IMU

The BMI088 provides:

- Three-axis accelerometer measurements.
- Three-axis gyroscope measurements.

The simulated IMU will include configurable measurement noise and gyroscope bias.

Accelerometer measurements must account for both gravity and translational acceleration. Gyroscope measurements will provide angular velocity with configurable noise and bias.

### Wheel Encoders

The encoder model will simulate wheel rotation measurements.

Configurable parameters will include:

- Encoder resolution.
- Gearbox ratio.
- Wheel radius.
- Measurement noise.

Encoder measurements will be used to estimate wheel position and velocity.

## Installation

### Requirements

- Python 3.10 or newer
- NumPy
- SciPy
- Matplotlib

### 1. Clone the Repository

```bash
git clone <repository-url>
cd quadbot_sim
```

### 2. Create a Virtual Environment

```bash
python -m venv .venv
```

### 3. Activate the Environment

**Windows PowerShell:**

```powershell
.\.venv\Scripts\Activate.ps1
```

**Linux/macOS:**

```bash
source .venv/bin/activate
```

### 4. Install Dependencies

```bash
python -m pip install -r requirements.txt
```

## Running the Simulation

Run the main simulation script:

```bash
python main.py
```

Simulation results and plots will be saved or displayed according to the configuration in the project.

## Development Roadmap

### Phase 1 — Physical Model

- [ ] Implement two-wheeled inverted pendulum dynamics.
- [ ] Validate the equations of motion.
- [ ] Implement the linearised state-space model.
- [ ] Test open-loop system behaviour.

### Phase 2 — Controller Design

- [ ] Implement continuous-time LQR.
- [ ] Calculate feedback gain matrix.
- [ ] Test closed-loop stability.
- [ ] Tune \(Q\) and \(R\).
- [ ] Implement motor torque saturation.

### Phase 3 — Sensor Modelling

- [ ] Implement BMI088 accelerometer model.
- [ ] Implement BMI088 gyroscope model.
- [ ] Add configurable sensor noise and bias.
- [ ] Implement wheel encoder model.

### Phase 4 — State Estimation

- [ ] Implement Kalman filter prediction.
- [ ] Implement sensor measurement updates.
- [ ] Tune process and measurement noise covariance.
- [ ] Compare estimated states against actual simulated states.

### Phase 5 — Embedded Implementation

- [ ] Port the controller to C.
- [ ] Match Python and STM32 state definitions.
- [ ] Validate control timing and numerical behaviour.
- [ ] Test with real sensor and motor hardware.

## Design Philosophy

The simulator is being developed alongside the embedded firmware.

Where practical, the Python implementation should use the same:

- State definitions.
- Coordinate conventions.
- Control input conventions.
- Sampling frequency.
- Controller equations.
- Sensor assumptions.

This will make it easier to compare simulated behaviour with the physical robot and transfer validated algorithms to the STM32 firmware.

## Safety

The simulation is intended to support development and testing, but successful simulation does not guarantee stable real-world operation.

Before testing on physical hardware:

- Verify motor direction and polarity.
- Check IMU axis orientation.
- Confirm encoder direction and scaling.
- Implement motor output limits.
- Include a motor-disable mechanism.
- Test with the robot supported before attempting free balancing.

## License

To be determined.