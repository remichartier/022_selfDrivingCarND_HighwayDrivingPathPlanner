/* fsm.cpp to manage finate state machine
 * History :
 * v01 : first version with fsm_transition_function()
 */

#include "fsm.h"

// using std::vector;


// usually : transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights)
void fsm_transition_function(int prev_size, double car_s, double end_path_s, bool &too_close, 
                            vector<vector<double>> sensor_fusion, int &lane){
/* 
 * Inputs : 
 *			- int prev_size
 *			- double car_s
 *			- double end_path_s
 *			- bool too_close, by reference
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane by reference
 * Outputs :
 *			- bool too_close
 *			- int lane
 *
 */
 
  if(prev_size > 0){
    car_s = end_path_s;
  }
          
  // bool too_close = false;
  too_close = false;

  // find ref_v to use
  for(int i=0; i < sensor_fusion.size(); i++)
  {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if( d < (2+4*lane+2) && d > (2+4*lane-2) )
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s+=((double)prev_size*0.02*check_speed);//if using previous points can project s value outwards in time
      // check s values greater than mine and s gap
      if((check_car_s > car_s) && ((check_car_s - car_s) < 30))
      {
        // do some logic here, lower reference velocity, so we don't crash into the car in front of us, could
        // also flag to try to change lanes
        // ref_vel = 29.5; //mph
        too_close = true;
        if(lane > 0)
        {
          lane = 0;
        }

      }
    }
  }
}