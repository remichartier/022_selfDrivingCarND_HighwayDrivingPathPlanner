/* fsm.h to manage finate state machine
 * History :
 * v01 : first version with fsm_transition_function()
 * v02 : add fsm_state input/output to fsm_transition_function()
 *       Add function isLaneChangeDone()
 * v03 : move isLaneChangeDone() --> fsm_isLaneChangeDone()
 *       Create fsm_isCarInLaneTooClose(), move increase/decrease speed
 *       from main to fsm with constants as well, MAX_ACCEL, MAX_SPEED_MPH 
 */

#ifndef FSM_H
#define FSM_H

#include <vector>
#include <math.h>
#include "fsm.h"

using std::vector;

#define MAX_ACCEL 					0.224
#define MAX_SPEED_MPH 				49.5

enum fsm_state { KeepLane, LaneChangeLeft, LaneChangeRight };

void fsm_transition_function(int prev_size, double car_s, double car_d, double end_path_s,
                             double &ref_vel, vector<vector<double>> sensor_fusion, 
                             int &lane, fsm_state &state);

bool fsm_isLaneChangeDone(int lane, double car_d);

bool fsm_isCarInLaneTooClose(int prev_size, double car_s, double car_d, double end_path_s, 
                            vector<vector<double>> sensor_fusion, int lane);

#endif  // FSM_H