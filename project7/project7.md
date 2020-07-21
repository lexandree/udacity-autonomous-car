# Project 7 - Path-Planning 

## Goal
The goal of this first project was to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.  The car should be able to make one complete loop around the 6946m highway. Since the car is traveling at a maximum speed of 50 mph, it will take just over 7 minutes to complete 1 loop, taking into account the slowing traffic. Also the car should not experience total acceleration over 10 m/s² and jerk that is greater than 10 m/s³.

## Environment
Prerequisites and instructions how to install project components are on the [project page](https://github.com/udacity/CarND-Path-Planning-Project) and the websocket is [here](https://github.com/uNetworking/uWebSockets)

I have installed the environment under Udacity Ubuntu Virtual Box with default pretty old tools:
- gcc 5.4.0
- make 4.1
- cmake 3.5.1
- [Udacity Term 3 Simulator v1.2](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

## Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Project Data

The project data consists of 181 waypoints from the map as our reference path. Each waypoint has an (x,y) global map position, and a Frenet s value and Frenet d unit normal vector (split up into the x component, and the y component). The s value is the distance along the direction of the road. The first waypoint has an s value of 0 because it is the starting point. The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. The d vector can be used to calculate lane positions. For example, if you want to be in the left lane at some waypoint just add the waypoint's (x,y) coordinates with the d vector multiplied by 2. Since the lane is 4 m wide, the middle of the left lane (the lane closest to the double-yellow dividing line) is 2 m from the waypoint. If you would like to be in the middle lane, add the waypoint's coordinates to the d vector multiplied by 6 = (2+4), since the center of the middle lane is 4 m from the center of the left lane, which is itself 2 m from the double-yellow dividing line and the waypoints.

Localization of the own car from the simulator, consists of x, y, yaw, speed in cartesian and s, d in Frenet space.

Sensor fusion information about the other cars from the simulator, those include car ID, x, y, vx, vy, s and d.

Previous path x and y coordinates from the simulator. It uses this information to generate a path made up of x and y points, that the car will visit sequentially every 0.02s.

## Planner
The behavior planner is responsible to generate the path and the speed to be followed by the car. The project implements a Finite State Machine (FSM) with 5 states, which the transition is realized through the minimization of a cost function. 
![txt][image1]

FSM is the "Behavior" part on the diagram and is implemented in [BehaviorLayer class](src/behavior_layer.cpp). For each possible transition, a trajectory will be generated and the cost estimated. 
![txt][image2]

The method [generate_trajectory](src/behavior_layer.cpp#L118-L134) calls the corresponding methods of the ego_car object to generate the trajectory for the possible state, which is then evaluated.

Two cost functions are implemented: 
 - [goal_distance_cost](src/cost.cpp#L17-L35) increases with both the distance of intended lane from the goal and the distance of the final lane from the goal. The cost of being out of the goal lane also becomes larger as the vehicle approaches the goal. This is an easy way to determine your preferred lane without traffic.
 - [inefficiency_cost](src/cost.cpp#L37-L53) becomes higher for trajectories with intended lane and final lane that have traffic slower than ego vehicle's target speed.

The "Prediction" part is implemented in [Vehicle class](src/vehicle.cpp). This class [predicts positions at the next time](src/vehicle.cpp#L192-L206), gets [next timestep kinematics](src/vehicle.cpp#L101-L133) (position, velocity, acceleration) for a given lane, tries to choose the maximum velocity, given other vehicle positions.

The "Trajectory" part is implemented in [PathGenerator class](src/path_generator.cpp). The PathGenerator needs a lane and velocity from the BehaviorLayer and Vehicle, uses localization data and outputs the car's trajectory as a series of x,y points. For smooth and comfortable path with small acceleration the [spline library](http://kluge.in-chemnitz.de/opensource/spline/) is used. The acceleration and jerk are defined due this part. Simple trajectory following requires less planning distance than lane change driving even with the cubic spline used.

## Conclusion
The elaborated implementation made it possible to pass several laps without violating the restrictions and avoiding collisions with other cars.
![txt][image3]


[//]: # (Image References)

[image1]: img/automat.png "Finite State Machine"
[image2]: img/behavior_control.png "system architecture"
[image3]: img/car5.PNG "Car in motion"
