#ifndef PATH_GENERATOR_H_
#define PATH_GENERATOR_H_

#include <string>
#include <vector>
#include <unordered_map>
//#include "helpers.h"
//#include "json.hpp"


class PathGenerator { 

  std::vector< std::vector<double> > next_values;
  int lane;
  double ref_vel;
 
 public:

  PathGenerator() : next_values(2), lane(1), ref_vel(0.0) {}

  ~PathGenerator() {}

  void set_lane(int lane);
  void set_velocity(double ref_vel);
  std::vector< std::vector<double> >& create_path(
										std::unordered_map<std::string, double> message_double_data, 
										std::unordered_map<std::string, std::vector<double> > message_vector_data,
										std::vector<std::vector<double> > sensor_fusion);
};

#endif  // PATH_GENERATOR_H_
