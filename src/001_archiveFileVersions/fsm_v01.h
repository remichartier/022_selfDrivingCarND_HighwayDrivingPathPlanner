/* fsm.h to manage finate state machine
 * History :
 * v01 : first version with fsm_transition_function()
 */

#ifndef FSM_H
#define FSM_H

#include <vector>
#include <math.h>
#include "fsm.h"

using std::vector;

enum fsm_state { KeepLane, LaneChangeLeft, LaneChangeRight };


void fsm_transition_function(int prev_size, double car_s, double end_path_s, bool &too_close, 
                            vector<vector<double>> sensor_fusion, int &lane);

#endif  // FSM_H