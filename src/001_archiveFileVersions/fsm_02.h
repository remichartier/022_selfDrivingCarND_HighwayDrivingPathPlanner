/* fsm.h to manage finate state machine
 * History :
 * v01 : first version with fsm_transition_function()
 * v02 : add fsm_state input/output to fsm_transition_function()
 *       Add function isLaneChangeOver()
 */

#ifndef FSM_H
#define FSM_H

#include <vector>
#include <math.h>
#include "fsm.h"

using std::vector;

enum fsm_state { KeepLane, LaneChangeLeft, LaneChangeRight };


void fsm_transition_function(int prev_size, double car_s, double car_d, double end_path_s,
                             bool &too_close, vector<vector<double>> sensor_fusion, 
                             int &lane, fsm_state &state);
bool isLaneChangeOver(int lane, double car_d);

#endif  // FSM_H