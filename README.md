# FinalProject_VSC
This document details the modeling and simulation of my 2004 Envoy XL in various driving modes. 
The models and code enclosed in this repo are based on lecture slides and examples provided by Professor Raul Longoria at the
University of Texas at Austin. The repo contains a MATLAB script for each component of the simulation.

EnvoyXL.m sets up the relevant physical parameters for the vehicle.
car_state.m is used for animation.
The various bicycle_....m files represent the system dynamics for each part of the simulation and are fed into an ODE solver (not provided).

Part 1: Performance and Turning
- Accelerating to Steady State from Rest
- Braking from Steady State
- Turning

Part 2: Cruise Control

Part 3: ABS/Anti-Slip Control

Part 4: Steering Control

Part 5: Hydroplane
