#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
  //int L = 1;

  int preferred_buffer = 18; // impacts "keep lane" behavior.

  int lane, s, goal_lane, goal_s; //, lanes_available;

  float v, d, target_speed, a, max_acceleration;

  string state;

  map<string, int> LANE_DIRECTION = {{"PLCL", 1}, {"LCL", 1}, 
                                     {"LCR", -1}, {"PLCR", -1}};

 public:
  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state="CS", float target_speed=21.87, float max_acceleration=10.0, int goal_lane=2);

  virtual ~Vehicle();

  
  float get_target_speed() const {return target_speed;}

  float get_v() const {return v;}

  int get_goal_lane() const {return goal_lane;}

  int get_s() const {return s;}

  int get_goal_s() const {return goal_s;}

  int get_lane() const {return lane;}

  string get_state() const {return state;}

  void set_lane(int lane) {this->lane = lane;}

  void set_state(string state) {this->state = state;}
  
  void set_observation(int s, float v, float d) {
	this->s = s;
	this->v = v;
	this->d = d;
  }

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> lane_change_trajectory(const string state, 
                                         map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> prep_lane_change_trajectory(const string state, 
                                              map<int, vector<Vehicle>> &predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane, 
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane, 
                         Vehicle &rVehicle);

  float get_lane_speed(map<int, vector<Vehicle>> &predictions, int lane, int dist_range=120);

  vector<Vehicle> generate_predictions(int horizon=2);

};

#endif  // VEHICLE_H