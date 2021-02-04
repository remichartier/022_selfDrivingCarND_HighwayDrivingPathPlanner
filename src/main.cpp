
/* main.cpp
 * HISTORY
 * v000 : original/initial version - without any changes
 * v001 : test car go straight (now commented)
 * v002 : test car follow track
 * v003 : test with spline.h to smooth the drive following lane. but not working
 * v004 : spline not working
 * v005 : spline video example working
 * v006 : Tackle avoiding running into cars according to Q&A video
 * v007 : Fix cold start acceleration issue like video example
 * v008 : Implement change lanes when hitting 30 meters before car ahead
 * v009 : Add MAX_ACCEL, MAX_SPEED_MPH, Try to add fsm.cpp and fsm.h  
 *       to introduce fsm_transition_function() usage
 * v010 : Clean code replaced by fsm_transition_function()
 *       Prototype change for fsm_transition_function() to switch state
 *       add `state` to honMessage lambda function prototype
 * v011 : Code before test changing trajectory to start at first 5 points 
 *       instead of adding to the whole previous_path_x/previous_path_y ie rebuilding
 *       next_x_vals/next_y_vals from  from previous_path_x[4]
 *       This would allow to functionalize Trajectory Generation and apply chosen trajectory based on 
 *       Behavior Planning decision (KeepLane, LaneChangeRight, LaneChangeLeft) and to generate trajectories 
 *       accordingly.
 * v012 : Test changing trajectory to start at current previous_path_x[4] + remove car go straight code in comments
 *        and remove example of car follow track without spline trajectory generation - code in comment
 * v013 : try to functionalize TrajectoryGeneration()
 *        add trajectory.cpp file, transform helpers.h into helpers.cpp and helper.h
 * v014 : Working version with TrajectoryGeneration(), removed replaced from TrajectoryGeneration()
 *        Fixed bug on prev_size
 */

#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"  // included already in trajectory.h below
#include "json.hpp"

#include "spline.h" // for polynomials
#include "fsm.h"
#include "trajectory.h"

#define MAX_ACCEL 					0.224
#define MAX_SPEED_MPH 				49.5
#define REUSE_PREVIOUS_PATH_SIZE 	5

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


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
  
  // drive car in middle lane following track
  int lane = 1;
  double ref_vel = 0.0; // 49.5;
  
  // initialise FSM state at the begining
  fsm_state state = KeepLane;
          
  /*
   * Note the change ...
   * From original : 
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) { 
   */
  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&state]
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
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          

          /**
          * Test code for car to follow track like in Q&A
          * with spline to smoothen trajectory
          */

          // For Trajectory Generation, only reuse 5 first previous_path points
          // And then for rest of 50 points, generate using Trajectory Generation function
          // or Trajectory chosen by Behavior Planner and generated via Trajectory Generation function
          
          int prev_size = previous_path_x.size();
          //(b<a)?b:a;
          prev_size = (REUSE_PREVIOUS_PATH_SIZE<prev_size) ? REUSE_PREVIOUS_PATH_SIZE : prev_size;
          // std::cout << "prev_size = " << prev_size << std::endl; 
          
          // drive car in middle lane following track
          
          bool too_close; // to trigger increase/decrease acceleration
          
          // Using Sensor Fusion data to avoid hitting cars
          // Call to FSM TRANSITION FUNCTION to decide KeepLane or ChangeLane Left or Right
          fsm_transition_function(prev_size, car_s, car_d, end_path_s, too_close, 
                            sensor_fusion, lane, state);
          
          // code to adjust incrementally speed changes to avoid crashing into car ahead
          // in the lane
          if(too_close)
          {
            ref_vel -= MAX_ACCEL;
          }
          else if(ref_vel < MAX_SPEED_MPH)
          {
            ref_vel += MAX_ACCEL;
          }
          
          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          // call to TRAJECTORY GENERATION, right now to generate trajectory to follow highway waypoints
          trajectory_generation(car_x, car_y, car_yaw, car_s, prev_size,
                               previous_path_x, previous_path_y,
                               map_waypoints_s, map_waypoints_x, map_waypoints_y,
                               lane, ref_vel, next_x_vals, next_y_vals);

          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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