# Quadrotor Precision Landing Control System

This project implements a control system for enabling a quadrotor drone to land precisely on a moving vehicle. The solution combines traditional control methods with Physics-Informed Neural Networks (PINNs) to achieve robust and accurate landing performance.

## Project Overview

The system consists of three main components:
1. MATLAB/Simulink model for quadrotor dynamics and control
2. Python-based PINN controller
3. Interface layer for MATLAB-Python communication

### Key Features

- 12-state quadrotor dynamics model
- Real-time control using PINNs
- Moving platform tracking
- Fallback to traditional control
- Comprehensive simulation and analysis tools

## System Requirements

### MATLAB Requirements
- MATLAB R2020b or later
- Simulink
- Control System Toolbox
- Simscape (optional, for 3D visualization)

### Python Requirements
- Python 3.8 or later
- PyTorch 1.9.0 or later
- Flask 2.0.1 or later
- NumPy 1.19.2 or later
- Requests 2.26.0 or later

## Installation

1. Clone the repository:
```bash
git clone [repository-url]
cd uav-design
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

3. Ensure MATLAB is properly installed and configured in your system PATH.

## Project Structure

```
uav-design/
├── quadrotor_control.slx    # Simulink model for quadrotor control
├── pinn_controller.py       # Python PINN controller implementation
├── pinn_interface.m         # MATLAB-Python interface
├── analyze_results.m        # Simulation analysis script
├── requirements.txt         # Python dependencies
└── README.md               # This file
```

## UAV Analysis Notebook

The `uav_analysis_new.ipynb` notebook provides a comprehensive analysis and visualization of the UAV design parameters and performance metrics. It includes:

- Data loading and simulation results visualization
- Statistical analysis of position, velocity, and acceleration
- Interactive plots for better understanding of the UAV dynamics

You can run the notebook using Jupyter:

```bash
jupyter notebook src/python/uav_analysis_new.ipynb
```

## Usage

### Running the Simulation

1. Start the Python PINN controller server:
```bash
python pinn_controller.py
```

2. Open MATLAB and run the simulation:
```matlab
% Open the Simulink model
open('quadrotor_control.slx')

% Run the simulation
sim('quadrotor_control')

% Analyze results
run('analyze_results.m')
```

### Simulation Parameters

The simulation can be configured by modifying the following parameters in `quadrotor_control.slx`:

#### Quadrotor Parameters
- Mass: 1.0 kg
- Moments of inertia: [0.1, 0.1, 0.2] kg·m²
- Arm length: 0.2 m
- Thrust coefficient: 1.0
- Moment coefficient: 0.1

#### Control Parameters
- Position control gains: [2.0, 2.0, 2.0]
- Attitude control gains: [2.0, 2.0, 2.0]

#### Platform Parameters
- Initial position: [0, 0, 0] m
- Velocity: [0.5, 0.5, 0] m/s

#### Simulation Parameters
- Simulation time: 20 seconds
- Step size: 0.01 seconds

## Results Analysis

The `analyze_results.m` script provides comprehensive analysis of the simulation results, including:

1. Position Control Performance
   - Root Mean Square Error (RMSE)
   - Maximum position error
   - Settling time

2. Attitude Control Performance
   - Attitude error metrics
   - Control effort analysis

3. Visualization
   - Position tracking plots
   - Attitude control plots
   - Control input plots
   - 3D trajectory visualization

## PINN Controller Architecture

The Physics-Informed Neural Network controller consists of:

1. Input Layer (12 neurons)
   - Position (x, y, z)
   - Velocity (vx, vy, vz)
   - Attitude (roll, pitch, yaw)
   - Angular rates (p, q, r)

2. Hidden Layers
   - Two fully connected layers with 64 neurons each
   - ReLU activation functions

3. Output Layer (4 neurons)
   - Thrust command
   - Roll command
   - Pitch command
   - Yaw command

## Safety Features

The system includes several safety mechanisms:

1. Fallback Control
   - Automatic switch to traditional control if PINN controller fails
   - Graceful degradation of performance

2. Control Signal Bounds
   - Thrust limits: [0, 20] N
   - Attitude command limits: [-π/4, π/4] rad

3. Error Handling
   - Robust communication error handling
   - State validation
   - Control signal validation

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

This project is based on research from the following papers:
- "Autonomous Landing of a Quadrotor on a Moving Platform"
- "Vision-based autonomous quadrotor landing on a moving platform"
- "Physics-informed Neural Network for Quadrotor Dynamical Modeling"

## Contact

For questions and support, please open an issue in the repository. 