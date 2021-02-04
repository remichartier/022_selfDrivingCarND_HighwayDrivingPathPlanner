/* trajectory.h to manage Trajectory Generation
 * History :
 * v000 : first version with trajectoryGeneration() function
 * v02 : add fsm_state input/output to fsm_transition_function()
 *       
 */
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h> // for cos/sin/atan2 functions
#include <vector>
using std::vector;


void trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, int prev_size,
                          vector<double> previous_path_x, vector<double> previous_path_y,
                          vector<double> map_waypoints_s, vector<double> map_waypoints_x,
                          vector<double> map_waypoints_y, int lane, double ref_vel,
                          vector<double> &next_x_vals, vector<double> &next_y_vals);

#endif  // TRAJECTORY_H