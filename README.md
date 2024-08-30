# Quadrotor Trajectory Generation and PD Control

This repository contains the implementation of trajectory generation and PD control for a quadrotor, as part of the Aerial Robotics course. The project is designed to control a quadrotor's motion in a 3D environment, allowing it to follow a set of predefined waypoints smoothly and accurately.

## Project Overview

The main objectives of this project are:
1. **Trajectory Generation**: 
   - Generate smooth polynomial trajectories for a quadrotor to follow.
   - Ensure continuous motion through waypoints without stopping at each one.
   - Handle real-world flight dynamics.

2. **PD Control Implementation**:
   - Implement a PD controller to stabilize the quadrotor and ensure it follows the generated trajectories.
   - Tune the control gains for accurate waypoint tracking.

## Files in this Repository

- **traj_generator.m**: MATLAB script that generates polynomial trajectories for the quadrotor.
- **control.m**: MATLAB script that implements the PD controller.
- **simulation_3d.m**: MATLAB script that simulates the quadrotor's flight in a 3D environment.
- **runsim.m**: Main script to run the simulation.

## How to Run

1. Clone this repository to your local machine.
2. Open MATLAB and navigate to the directory containing the scripts.
3. Run the `runsim.m` script to start the simulation.
4. The quadrotor will follow the predefined waypoints using the generated trajectory and PD control.

## Acknowledgements

This project is based on concepts and frameworks learned in the Aerial Robotics course. Special thanks to the course instructors for their guidance and resources.

