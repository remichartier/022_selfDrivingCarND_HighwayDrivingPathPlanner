/* trajectory.cpp
 * HISTORY
 * v001 : try to functionalize TrajectoryGeneration(), add trajectory.cpp file
 *       
 */
#include "trajectory.h"
#include "spline.h" // for polynomials
#include "helpers.h" // for deg2rad()


void trajectoryGeneration(double car_x, double car_y, double car_yaw, double car_s, int prev_size,
                          vector<double> previous_path_x, vector<double> previous_path_y,
                          vector<double> map_waypoints_s, vector<double> map_waypoints_x,
                          vector<double> map_waypoints_y, int lane, double ref_vel,
                          vector<double> &next_x_vals, vector<double> &next_y_vals)
{
  // Code to follow lane and full track.

  // Create a list of widely spaced x,y waypoints, evenly spaced at 30 m
  // Later, we will interpolate those waypoints with a spline and fill it
  // with more points that control ...
  vector<double> ptsx;
  vector<double> ptsy;

  // Reference x, y, yaw states
  // Reference the starting point as where the car is

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // if path size is almost empty, use the car as starting reference
  if(prev_size < 2)
  {
    // Use two points that make the path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else
  {
    // Redefine reference state as previous path's end point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // Use the points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  } 

  // In Frenet add evenly 30m spaced points ahead of the starting reference.
  // NOTE : GOOD TRAJECTORY FOR FOLLOWING TRACK, BUT NOT FOR TURNING LEFT OR RIGHT ...
  vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // transform into car coordinates before building the spline
  for(int i=0; i < ptsx.size(); i++)
  {
    // Shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }
  /*
          std::cout << "car_yaw = " << car_yaw << ", ";
          std::cout << "ptsx[] = " ;
          for(int i=0; i < ptsx.size(); i++)
          {
            std::cout << ptsx[i] << ", ";
          }
          std::cout << std::endl;
          */

  // Create a spline
  tk::spline s;

  // Set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  


  // Start with all of the previous path points from last time
  // Modification to only use the x first previous_path points instead of
  // all the one available
  //for(int i=0; i<previous_path_x.size(); i++)
  for(int i=0; i<prev_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up the spline points so that we travel at our desired reference velocity
  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
  double x_add_on = 0;

  // Fill up the rest of the path planner after filling it with previous points, we aim to output 50 pts
  // Modify previous_path_x.size() --> prev_size to only reuse 5 previous_path points at max.
  // for(int i=1; i <= 50 - previous_path_x.size(); i++)
  for(int i=1; i <= 50 - prev_size; i++)
  {
    double N = target_dist/(0.02*ref_vel/2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate back to normal after rotating it earlier
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

} // end trajectoryGeneration()