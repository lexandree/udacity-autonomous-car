#ifndef BEHAVIOR_LAYER_H
#define BEHAVIOR_LAYER_H

#include <string>
#include <vector>
#include <utility>
#include "vehicle.h"

class BehaviorLayer {

  string state;
  int lanes_available;
  Vehicle *ego_car;
  vector<Vehicle> keep_lane_trajectory;

public:

  BehaviorLayer();
  BehaviorLayer(Vehicle *car, int lanes_available);

  virtual ~BehaviorLayer();

  std::pair<int, float> get_next_lane(map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);
  
  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(const string state, 
                                      map<int, vector<Vehicle>> &predictions);
};

#endif  // BEHAVIOR_LAYER_H