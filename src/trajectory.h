/* trajectory.h to manage Trajectory Generation
 * History :
 * v000 : first version with trajectoryGeneration() function
 * v001 : add fsm_state input/output to fsm_transition_function()
 * TAGGED v1.0 on Github : Working version, Change Lane if too close to car ahead
 *        Test : Enable Lane change anytime Cost computation judge necessary
 * TAGGED v1.1 on Github (Working version, changing lanes depending of costs
 *        Cleanup of code       
 */
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
using std::vector;


void trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, int prev_size,
                          vector<double> previous_path_x, vector<double> previous_path_y,
                          vector<double> map_waypoints_s, vector<double> map_waypoints_x,
                          vector<double> map_waypoints_y, int lane, double ref_vel,
                          vector<double> &next_x_vals, vector<double> &next_y_vals);

#endif  // TRAJECTORY_H