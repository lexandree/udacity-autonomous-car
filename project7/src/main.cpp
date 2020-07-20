#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "path_generator.h"
#include "vehicle.h"
#include "behavior_layer.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::unordered_map;
using std::round;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  //sart in lane 1
  int lane = 1;
  double ref_vel = 0.0; //m/s
  std::cout.precision(3);
  PathGenerator path_gen;
  Vehicle ego_car( lane, 0, ref_vel, 0, "KL");
  BehaviorLayer car_brain(&ego_car, 3);
  
  h.onMessage([&ego_car, &car_brain, &ref_vel, &path_gen, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = deg2rad(j[1]["yaw"]);
          double car_speed = (double)j[1]["speed"] / 2.24; // MPH to ms, all values are metric now

          // Previous path data given to the Planner
          vector<double>  previous_path_x = j[1]["previous_path_x"];
          vector<double>  previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double> > sensor_fusion = j[1]["sensor_fusion"];
		  //
          json msgJson;

		  unordered_map<string, double> message_double_data;
		  unordered_map<string, vector<double> > message_vector_data;
		  
		  message_double_data["car_x"] = car_x;
		  message_double_data["car_y"] = car_y;
		  message_double_data["car_yaw"] = car_yaw;
		  message_double_data["car_s"] = car_s;
		  message_double_data["car_d"] = car_d;
		  message_double_data["lane"] = (double)lane;
		  message_double_data["ref_vel"] = ref_vel;
		  message_double_data["end_path_s"] = end_path_s;
		  message_double_data["car_speed"] = car_speed;

		  message_vector_data["previous_path_x"] = previous_path_x;
		  message_vector_data["previous_path_y"] = previous_path_y;
		  message_vector_data["map_waypoints_x"] = map_waypoints_x;
		  message_vector_data["map_waypoints_y"] = map_waypoints_y;
		  message_vector_data["map_waypoints_s"] = map_waypoints_s;
		/*id = observation[0];
		  x = observation[1];
		  y = observation[2];
		  vx = observation[3];
		  vy = observation[4];
		  s = observation[5];
		  d = observation[6]; */
		  map<int ,vector<Vehicle> > predictions;
		  // make prediction for every sensor fusion sample and use it instead
		  for (auto &observation : sensor_fusion) {
			  Vehicle car((int)round((observation[6]-2)/4), observation[5], distance(0.0, 0.0, observation[3], observation[4]), 0.0, "KL");
			  predictions[observation[0]] = car.generate_predictions();
		  }
		  ego_car.set_observation((int)car_s, (float)car_speed, (float)car_d);
		  std::pair<int, float> new_state = car_brain.get_next_lane(predictions);
		  lane =  new_state.first;
		  ref_vel = new_state.second;
		  path_gen.set_lane(lane);
		  path_gen.set_velocity(ref_vel);
		  ego_car.set_lane(lane);
		  vector<vector<double> > next_values = path_gen.create_path(message_double_data, message_vector_data, sensor_fusion);
		  
          msgJson["next_x"] = next_values[0];
          msgJson["next_y"] = next_values[1];
		  
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}