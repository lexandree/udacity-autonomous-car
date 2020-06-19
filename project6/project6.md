# Project 6 - Kidnapped-Vehicle

The goal of this first project was to implements a [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter) applied to a [Kidnapped robot(car) problem](https://en.wikipedia.org/wiki/Kidnapped_robot_problem). A simulator is provided by Udacity ([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)). This simulator will generate noisy landmark observations from a car sensor with cartesian coordinates to feed the Particle Filter using [WebSocket](https://en.wikipedia.org/wiki/WebSocket). The Particle Filter uses the [uWebSockets](https://github.com/uNetworking/uWebSockets) WebSocket implementation to respond to this observation with the estimated car position. Udacity provides a [seed project to start from](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

Prerequisites and instructions how to install project components are on the [project page](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project) and the websocket is [here](https://github.com/uNetworking/uWebSockets)

I have installed the environment under Udacity Ubuntu Virtual Box with default pretty old tools:
- gcc 5.4.0
- make 4.1
- cmake 3.5.1
- Udacity Term 2 Simulator v1.45

# Code description

The Particle Filter is implemented in [src/particle_filter.cpp](./src/particle_filter.cpp) and consists of 4 routines:
- filter initialization, particles generation [ParticleFilter::init](./src/particle_filter.cpp#L25)
- predict step [ParticleFilter::prediction](./src/particle_filter.cpp#L57)
- update step [ParticleFilter::updateWeights](./src/particle_filter.cpp#L104)
- resample step [ParticleFilter::resample](./src/particle_filter.cpp#L138)

One great explanations with code can be found [here](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python).
The big difference is the update step. Without association between observations and landmarks the weights become too small values.

This implementation gives satisfactory result with particle number 50 but is not optimized.
[](./success.PNG)
