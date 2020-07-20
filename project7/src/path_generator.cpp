#include "path_generator.h"
#include "helpers.h"

#include <math.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
//#include <random>
#include <string>
#include <vector>
#include <unordered_map>
#include "spline.h"
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"

using namespace std;

void PathGenerator::set_lane(int lane) {
	this->lane = lane;
}

void PathGenerator::set_velocity(double ref_vel) {
	this->ref_vel = ref_vel;
}

 
vector< vector<double> >& PathGenerator::create_path( 
											unordered_map<string, double> message_double_data, 
											unordered_map<string, vector<double> > message_vector_data,
											vector<vector<double> > sensor_fusion) {
	  double car_x = message_double_data["car_x"];
	  double car_y = message_double_data["car_y"];
	  double car_yaw = message_double_data["car_yaw"];
	  double car_s = message_double_data["car_s"];
	  double car_d = message_double_data["car_d"];
	  double end_path_s = message_double_data["end_path_s"];
	  double car_speed = message_double_data["car_speed"];

	  const vector<double> previous_path_x = message_vector_data["previous_path_x"];
	  const vector<double> previous_path_y = message_vector_data["previous_path_y"];
	  const vector<double> map_waypoints_x = {message_vector_data["map_waypoints_x"]};
	  const vector<double> map_waypoints_y = {message_vector_data["map_waypoints_y"]};
	  const vector<double> map_waypoints_s = {message_vector_data["map_waypoints_s"]};
					
	  int prev_size = previous_path_x.size();
	  vector<double>  ptsx;
	  vector<double>  ptsy;
	  double ref_x = car_x;
	  double ref_y = car_y;

	  if (prev_size > 0)
		  car_s = end_path_s;
	  // speed equal measured speed or planned speed on the end of previous_path_x in terms of polling time
	  double prev_speed = 0.02 * car_speed; 
	  if (prev_size < 2) {
		  double prev_car_x = car_x - cos(car_yaw);
		  double prev_car_y = car_y - sin(car_yaw);
		  
		  ptsx.push_back(prev_car_x);
		  ptsx.push_back(car_x);
		  ptsy.push_back(prev_car_y);
		  ptsy.push_back(car_y);
		  std::cout << "prev_size: " << prev_size << std::endl;
	  } else {
		  ref_x = previous_path_x[prev_size - 1];
		  ref_y = previous_path_y[prev_size - 1];
		  double ref_x_prev = previous_path_x[prev_size - 2];
		  double ref_y_prev = previous_path_y[prev_size - 2];
		  car_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
		  prev_speed = distance(ref_x_prev, ref_y_prev, ref_x, ref_y); // speed in terms of polling time
		  
		  ptsx.push_back(ref_x_prev);
		  ptsx.push_back(ref_x);
		  ptsy.push_back(ref_y_prev);
		  ptsy.push_back(ref_y);
	  }
	  double offset_s = 30.0;
	  // if changing lane planned set further points to get more smooth trajectory, avoid large lateral jerk.
	  if(abs(car_d - (2+4*this->lane))>2.0)  
		offset_s = 50.0;
	  for (int i=0; i<3; i++) {
		vector<double>  next_wp = getXY(car_s + offset_s + i*30.0, (2+4*this->lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		ptsx.push_back(next_wp[0]);
		ptsy.push_back(next_wp[1]);
	  }
	  
	  for (int i = 0; i<ptsx.size(); i++) {
		  // do translate (shift) first and then rotate, the function does rotate first and this will be used later to bring trajectory back to cartesian coord.
		  vector<double> shift_xy = {ptsx[i] - ref_x, ptsy[i] - ref_y};
		  vector<double> points = rotate_translate(shift_xy, {0.0, 0.0}, -car_yaw);
		  ptsx[i] = points[0];
		  ptsy[i] = points[1];
	  }
	  
	  tk::spline s;
	  s.set_points(ptsx, ptsy, true); //default cubic_spline = true
	  
	  vector<double> next_x_vals;
	  vector<double> next_y_vals;
	  for (int i = 0; i < prev_size; i++) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	  }
	  double x_add_on = 0.0;
	  for (int i = 0; i < 67 - prev_size; i++) { // 30 m / (0.02 * 22.35 m/s) = 67 points - full path size (for max velocity)
		// 0.002 * i - defines acceleration; 
		// (0.02s * ref_vel m/s) = path for the choosen end velocity per 0.02s
		double x_point = x_add_on + 
			prev_speed +
			std::copysign(	std::min(0.002 * (double)i, abs(0.02 * ref_vel - prev_speed)), 
							(0.02 * ref_vel - prev_speed)); 
		double y_point = s(x_point);
		
		x_add_on = x_point;
		
		vector<double> xy_points = {x_point, y_point};
		vector<double> xy_ref_points = {ref_x, ref_y};
		// rotate and translate trajectory points
		vector<double> points = rotate_translate(xy_points, xy_ref_points, car_yaw);
		next_x_vals.push_back(points[0]);
		next_y_vals.push_back(points[1]);
	  }
	  next_values[0] = next_x_vals;
	  next_values[1] = next_y_vals;
	  return next_values;
}