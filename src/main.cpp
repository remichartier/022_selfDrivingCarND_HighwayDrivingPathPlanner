#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "spline.h" // for polynomials

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

/*
 * HISTORY
 * v00 : original/initial version - without any changes
 * v01 : test car go straight (now commented)
 * v02 : test car follow track
 * v03 : test with spline.h to smooth the drive following lane.
 */

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          /**
          * Test code to go straight for first time testing simulator
          
          double dist_inc = 0.5;
          for (int i = 0; i < 50; ++i) {
            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          }
          */
          
          /**
          * Test code for car to follow track like in Q&A
          double dist_inc = 0.5;
          for (int i = 0; i < 50; ++i) {
            double next_s = car_s + (i+1)*dist_inc;
            double next_d = 6 ; // 4 left lane + 2 half of middle lane
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
		  */
		  /**
          * Test code for car to follow track like in Q&A
          * with spline to smoothen trajectory
          */
          // start n lane 1
          int lane = 1;
          // have a reference velocity to target
          double ref_vel = 49.5; // mph
          
          int prev_size = previous_path_x.size();
          
          // Create a list of widely spaced x,y waypoints, evenly spaced at 30 m
          // Later, we will interpolate those waypoints with a spline and fill it
          // with more points that control ...
          vector<double> ptsx;
          vector<double> ptsy;
          // reference x,y yaw states
          // Either we will reference the starting point as where the car is or as 
          // the previous path and points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          // if previous state is almost empty, use the car as starting reference
          if(prev_size <2){
            // use 2 points that makes the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
                                            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
		    ptsy.push_back(car_y);
          }
          // use the previous path's end point as starting reference
          else{
            // redefine reference state as previous path and points
            ref_x = previous_path_x[prev_size -1];
            ref_y = previous_path_y[prev_size -1];
            double ref_x_prev = previous_path_x[prev_size -2];
            double ref_y_prev = previous_path_y[prev_size -2];
            ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);
            // use 2 points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
		    ptsy.push_back(ref_y);
          }
          // in Frenet add evenly 30 m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 70,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp0[1]);
          
          // move to local coordinates to the car, position 0,0, angle 0
          for(int i=0;i<ptsx.size();++i){
          	double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }
          // create a spline
          tk::spline s;
          // set x,y points to the spline
          s.set_points(ptsx,ptsy);
          // define the actual points (x,y) we will use for the planner
          // use next_x_vals, next_y_vals
          
          // start with all the previous path points from the last time
          for(int i=0;i<previous_path_x.size();++i){
          	next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // calculate how to break up spline points so that we travel out our desired velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0;
          
          // fill up the rest of our path planner after filling it with previous points, here we will always 
          // output 50 points
          for(int i=0; i <= 50 - previous_path_x.size(); ++i){
          	double N = target_dist / (0.02*ref_vel/2.24); // because ref_vel in miles/h, needs to be in m/h
            double x_point = x_add_on + (target_x/N);
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            // inverse back from local to global coordinates
            double x_ref = x_point;
            double y_ref = y_point;
            // rotate back to normal after rotating it earlier
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += x_ref;
            y_point += y_ref; 
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          
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