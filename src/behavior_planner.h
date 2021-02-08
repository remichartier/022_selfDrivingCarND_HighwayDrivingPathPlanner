/* fsm.h to manage finate state machine
 * History :
 * v01 : first version with fsm_transition_function()
 * v02 : add fsm_state input/output to fsm_transition_function()
 *       Add function isLaneChangeDone()
 * v03 : move isLaneChangeDone() --> fsm_isLaneChangeDone()
 *       Create fsm_isCarInLaneTooClose(), move increase/decrease speed
 *       from main to fsm with constants as well, MAX_ACCEL, MAX_SPEED_MPH 
 * v04 : introduce possible immediate move analyis via function fsm_next_lanes_possible()
 *       with LANE_MIN and LANE_MAX
 * 
 * Now called behavior_planner.h
 * v001 : Add bp_indexClosestCarAhead(), bp_lane_decider(), change bp_adjustAcceleration()
 * v002 : add SAFE_DISTANCE_BEHIND_M, to avoid hitting lateral cars. Increase SAFE_DISTANCE_M
 *        add changeLaneCounter
 * v003 : Generate trajectories for each prediction (KeepLane, LaneChangeLeft, LaneChangeRight
 *        Add int bp_next_lane()
 *        bp_compute_cost_states()
 */

#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include <math.h>

using std::vector;


enum fsm_state { KeepLane, LaneChangeLeft, LaneChangeRight };
enum direction { AHEAD, BEHIND};

void bp_transition_function(int prev_size, double car_s, double car_d, double end_path_s,
                            double &ref_vel, vector<vector<double>> sensor_fusion, 
                            int &lane, fsm_state &state, int &changeLaneCounter,
                            double car_x, double car_y, double car_yaw,
                            vector<double> previous_path_x, vector<double> previous_path_y,
                            vector<double> map_waypoints_s, vector<double> map_waypoints_x,
                            vector<double> map_waypoints_y, 
                            vector<double> &next_x_vals, vector<double> &next_y_vals);

void bp_compute_cost_states(double car_s, vector<vector<double>> sensor_fusion, 
                            vector<fsm_state> possible_steer, int lane,
                            int index_car_ahead_currentLane,
                            int index_car_behind_currentLane,
                            double ref_vel, vector<double> &cost_steer);

void bp_adjustAcceleration(double car_s, vector<vector<double>> sensor_fusion,
                           int index_car_ahead, int dist_min, double &ref_vel,
                           bool &too_close);

bool bp_isLaneChangeDone(int lane, double car_d);

int bp_indexClosestCars(double car_s, vector<vector<double>> sensor_fusion, 
                        int lane, int &index_closest_behind,
                        int &index_closest_ahead);

void bp_lane_decider(vector<fsm_state> possible_steer, vector<double> cost_steer, 
                     int &lane, fsm_state &state, int &changeLaneCounter);

int bp_next_lane(fsm_state state, int lane);

void bp_generate_trajectories(vector<vector<double>> &trajectories_x,
                              vector<vector<double>> &trajectories_y,
                              vector<fsm_state> possible_steer, int lane,
                              double car_x, double car_y, double car_yaw, double car_s, int prev_size,
                              vector<double> previous_path_x, vector<double> previous_path_y,
                              vector<double> map_waypoints_s, vector<double> map_waypoints_x,
                              vector<double> map_waypoints_y, double ref_vel);

// functions not used anymore I think : 

int bp_indexClosestCarAhead(double car_s, vector<vector<double>> sensor_fusion, 
                               int lane);
  
bool bp_isCarInLaneTooClose(int prev_size, double car_s, double end_path_s, 
                            vector<vector<double>> sensor_fusion, int lane, int dist_min);

void bp_possible_steer(vector<fsm_state> &possible_steer,int lane);

void bp_next_lanes_possible(vector<fsm_state> &possible_next_move, bool &too_close,
                            int prev_size, double car_s, double end_path_s, 
                            vector<vector<double>> sensor_fusion, int lane);

#endif  // BEHAVIOR_PLANNER_H