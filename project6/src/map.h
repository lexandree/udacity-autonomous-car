/**
 * map.h
 *
 * Created on: Dec 12, 2016
 * Author: mufferm
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>

/**
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
  
  int id;     // Id of matching landmark in the map.
  double x;   // Local (vehicle coords) x position of landmark observation [m]
  double y;   // Local (vehicle coords) y position of landmark observation [m]
};

class Map {
 public:  
  std::vector<LandmarkObs> landmark_list; // List of landmarks in the map
};

#endif  // MAP_H_
