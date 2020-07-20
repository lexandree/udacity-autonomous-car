#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::string;
using std::vector;

const float REACH_GOAL = 0.1;
const float EFFICIENCY = 0.9;


float goal_distance_cost(Vehicle &vehicle, 
                         const vector<Vehicle> &trajectory, 
                         map<int, vector<Vehicle>> &predictions, 
                         map<string, float> &data) {
  // Cost increases based on distance of intended lane (for planning a lane 
  //   change) and final lane of trajectory.
  // Cost of being out of goal lane also becomes larger as vehicle approaches 
  //   goal distance.
  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0) {
    cost = 1 - 2*exp(-(abs(2.0*vehicle.get_goal_lane() - data["intended_lane"] 
         - data["final_lane"]) / distance));
  } else {
    cost = 1;
  }

  return cost;
}

float inefficiency_cost(Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.
  float proposed_speed_intended;
  float proposed_speed_final;

  proposed_speed_intended = vehicle.get_lane_speed(predictions, data["intended_lane"]);
  proposed_speed_final = vehicle.get_lane_speed(predictions, data["final_lane"]);
    
  float cost = (2.0*vehicle.get_target_speed() - proposed_speed_intended 
             - proposed_speed_final)/vehicle.get_target_speed();

  return cost;
}

float calculate_cost(Vehicle &vehicle, 
                     map<int, vector<Vehicle>> &predictions, 
                     const vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, 
                                                       predictions);
  float cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<float(Vehicle &, const vector<Vehicle> &, 
                             map<int, vector<Vehicle>> &, 
                             map<string, float> &)
    >> cf_list = {goal_distance_cost, inefficiency_cost};
  vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, 
                                               trajectory_data);
    cost += new_cost;
  }

  return cost;
}

map<string, float> get_helper_data(const Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   const map<int, vector<Vehicle>> &predictions) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or 
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help 
  //   differentiate between planning and executing a lane change in the 
  //   cost functions.
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;

  if (trajectory_last.get_state().compare("PLCL") == 0) {
    intended_lane = trajectory_last.get_lane() + 1;
  } else if (trajectory_last.get_state().compare("PLCR") == 0) {
    intended_lane = trajectory_last.get_lane() - 1;
  } else {
    intended_lane = trajectory_last.get_lane();
  }

  float distance_to_goal = vehicle.get_goal_s() - trajectory_last.get_s();
  float final_lane = trajectory_last.get_lane();
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
    
  return trajectory_data;
}
