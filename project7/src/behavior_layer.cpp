#include "behavior_layer.h"
#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "cost.h"
//#include "helpers.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

BehaviorLayer::BehaviorLayer(){}

BehaviorLayer::BehaviorLayer(Vehicle *car, int lanes_available) {
  this->ego_car = car;
  this->lanes_available = lanes_available; 
}

BehaviorLayer::~BehaviorLayer() {}

std::pair<int, float> BehaviorLayer::get_next_lane(map<int, vector<Vehicle>> &predictions) {
  vector<Vehicle> trajectory = choose_next_state(predictions);
  /**
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output lane to follow and speed on it 
   */
  int next_lane = trajectory[1].get_lane();
  if (	this->ego_car->get_state().compare("LCL") == 0 || 
		this->ego_car->get_state().compare("LCR") == 0) 
	return {next_lane, trajectory[1].get_v()};

  return {this->ego_car->get_lane(), this->keep_lane_trajectory[1].get_v()};
}

vector<Vehicle> BehaviorLayer::choose_next_state(map<int, vector<Vehicle>> &predictions) {
  /**
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   */
  float min_costs = 99;
  vector<Vehicle> opt_trajectory;
  vector<string> s_states = successor_states();
  string new_state = "KL";
  for (auto &curr_state : s_states) {
	if(curr_state.compare("KL0") == 0) {
		curr_state = "KL";
		this->keep_lane_trajectory = generate_trajectory(curr_state, predictions);;
		ego_car->set_state(curr_state);
		return this->keep_lane_trajectory;
	}
    vector<Vehicle> trajectory = generate_trajectory(curr_state, predictions);
	if(curr_state.compare("KL") == 0) {
		this->keep_lane_trajectory = trajectory;
		//cout << "set keep_lane_trajectory size:" << this->keep_lane_trajectory.size() << endl;
	}
    if (trajectory.size()) {
		float current_costs = calculate_cost(*this->ego_car, predictions, trajectory);
		//cout << "check state: " << curr_state << ", cost: " << current_costs << endl;
		if (current_costs < min_costs){
			min_costs = current_costs;
			opt_trajectory = trajectory;
			new_state = curr_state;
		}
	}
  }
  //cout << "current lane, state: " << this->ego_car->get_lane() << this->ego_car->get_state() << ", new state: " <<  new_state << "\n" << endl;
  ego_car->set_state(new_state);
  if (opt_trajectory.size()) 
	  return opt_trajectory;
  return this->keep_lane_trajectory; 
}

vector<string> BehaviorLayer::successor_states() {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  string state = this->ego_car->get_state();
  if(state.compare("KL") == 0) {
	if (this->ego_car->get_lane() != 0)
		states.push_back("PLCL");
	if (this->ego_car->get_lane() != lanes_available - 1)
		states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (this->ego_car->get_lane() != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
	if (this->ego_car->get_lane() != lanes_available - 1)
		states.push_back("PLCR");
  } else if (state.compare("PLCR") == 0) {
    if (this->ego_car->get_lane() != lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
	if (this->ego_car->get_lane() != 0)
		states.push_back("PLCL");
  } else if((state.compare("LCL") == 0)||(state.compare("LCR") == 0)) {
	states.clear();
	states.push_back("KL0"); // additional state to stay on the lane after change
  }
  return states;
}

vector<Vehicle> BehaviorLayer::generate_trajectory(const string state, 
                                             map<int, vector<Vehicle>> &predictions) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = this->ego_car->constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = this->ego_car->keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = this->ego_car->lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = this->ego_car->prep_lane_change_trajectory(state, predictions);
  }

  return trajectory;
}



