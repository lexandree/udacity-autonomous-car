#include "vehicle.h"
//#include "helpers.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <bits/stdc++.h>
//#include "cost.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state, float target_speed, float max_acceleration, int goal_lane) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  this->target_speed = target_speed;
  this->max_acceleration = max_acceleration;
  this->goal_lane = goal_lane;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  float next_pos = position_at(1);
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state), 
                                Vehicle(this->lane, next_pos, this->v, 0, this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, state)};
  vector<float> kinematics = get_kinematics(predictions, this->lane);
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];
  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
  
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(const string state, 
                                                     map<int, vector<Vehicle>> &predictions) {
  // Generate a trajectory preparing for a lane change.
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  int new_lane = this->lane - LANE_DIRECTION.at(state);
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);
  vector<float> best_kinematics;
  vector<float> new_lane_kinematics = get_kinematics(predictions, new_lane);
  // Choose kinematics with lowest velocity.
  if (new_lane_kinematics[1] < curr_lane_new_kinematics[1]) 
	best_kinematics = new_lane_kinematics;
  else
	best_kinematics = curr_lane_new_kinematics;
  new_s = best_kinematics[0];
  new_v = best_kinematics[1];
  new_a = best_kinematics[2];

  trajectory.push_back(Vehicle(new_lane, new_s, new_v, new_a, state));
  
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(const string state, 
                                                map<int, vector<Vehicle>> &predictions) {
  // Generate a lane change trajectory.
  int new_lane = this->lane - LANE_DIRECTION.at(state);
  vector<Vehicle> trajectory;
  // Check if a lane change is possible (check if another vehicle occupies that spot).
  for (auto car : predictions) {
    if (car.second[0].lane == new_lane) 
		if ((int32_t)(( (int)(car.second[0].s - this->s + 17) | (int)(this->s - car.second[0].s + 6) ) >= 0)||
			(int32_t)(( (int)(car.second[1].s - this->s + 17) | (int)(this->s - car.second[1].s + 6) ) >= 0)) {
		  // If lane change is not possible, return empty trajectory.
		  return trajectory;
		}
		// check the car that is overtaking
		if (car.second[0].s < this->s && car.second[1].s > this->s)
		  return trajectory;
  }
  trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
  vector<float> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
  return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions, 
                                      int lane) {
  // Gets next timestep kinematics (position, velocity, acceleration) 
  //   for a given lane. Tries to choose the maximum velocity and acceleration, 
  //   given other vehicle positions and accel/velocity constraints.
  float max_velocity_accel_limit = this->max_acceleration + this->v;
  float new_position;
  float new_velocity;
  float new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      new_velocity = vehicle_ahead.v;
    } else {
      float max_velocity_in_front = (vehicle_ahead.s - this->s 
                                  - this->preferred_buffer) + vehicle_ahead.v 
                                  - 0.5 * (this->a);
      new_velocity = std::min(std::min(max_velocity_in_front, 
                                       max_velocity_accel_limit), 
                                       this->target_speed);
    }
  } else {
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
  }
    
  new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity + new_accel / 2.0;
    
  return{new_position, new_velocity, new_accel};
}

void Vehicle::increment(int dt = 1) {
  this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
  return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane, 
                                 Vehicle &rVehicle) {
  // Returns a true if a vehicle is found behind the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = this->s - this->v*2; // look at 2 sek backwards
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (auto car : predictions) {
    temp_vehicle = car.second[0];
    if (temp_vehicle.lane == lane && temp_vehicle.s < this->s 
        && temp_vehicle.s > max_s) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

float Vehicle::get_lane_speed(map<int, vector<Vehicle>> &predictions, int lane, int dist_range) {
  Vehicle vehicle_ahead;
  if (get_vehicle_ahead(predictions, lane, vehicle_ahead))
	  if (vehicle_ahead.s - this->s < dist_range)
		return vehicle_ahead.get_v();
  return this->get_target_speed();
	
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane, 
                                Vehicle &rVehicle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int min_s = this->s + this->v*5; // look at 5 sek ahead
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (auto car : predictions) {
    temp_vehicle = car.second[0];
    if (temp_vehicle.lane == lane && temp_vehicle.s > this->s 
        && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
  // Generates predictions for non-ego vehicles to be used in trajectory 
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  for(int i = 0; i < horizon; i++) {
    float next_s = position_at(i);
    float next_v = this->v;
    if (i < horizon-1) {
      next_v = position_at(i+1) - s;
    }
    predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  }
  
  return predictions;
}
