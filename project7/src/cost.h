#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(Vehicle &vehicle, 
                     map<int, vector<Vehicle>> &predictions, 
                     const vector<Vehicle> &trajectory);

float goal_distance_cost(Vehicle &vehicle,  
                         const vector<Vehicle> &trajectory,  
                         map<int, vector<Vehicle>> &predictions, 
                         map<string, float> &data);

float inefficiency_cost(Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        map<int, vector<Vehicle>> &predictions, 
                        map<string, float> &data);

map<string, float> get_helper_data(const Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   const map<int, vector<Vehicle>> &predictions);

#endif  // COST_H