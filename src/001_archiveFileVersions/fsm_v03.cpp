/* fsm.cpp to manage finate state machine
 * History :
 * v01 : first version with fsm_transition_function()
 * v02 : Separate into different states in fsm_transition_function()
 *       Modify fsm_transition_function() + change state
 *       Add switch/case to fsm_transition_function()
 *       Add function isLaneChangeDone()
 * v03 : improve isLaneChangeDone() car_d threshold/range to switch back
 *       to KeepLane state
 */

#include <iostream> // for cout, endl
#include "fsm.h"



// using std::vector;


// usually : transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights)
void fsm_transition_function(int prev_size, double car_s, double car_d, double end_path_s, bool &too_close, 
                            vector<vector<double>> sensor_fusion, int &lane, fsm_state &state){
/* 
 * Inputs : 
 *			- int prev_size
 *			- double car_s
 *			- double car_d
 *			- double end_path_s
 *			- bool too_close, by reference
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane by reference
 *			- fsm state : current state, by reference
 * Outputs :
 *			- bool too_close
 *			- int lane
 *			- fsm state : next state
 */
  
  switch(state){
      
    case KeepLane :
      
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
              state = LaneChangeLeft;
              std::cout << "state --> LaneChangeLeft" << std::endl;
            }

          } // end if car ahead and less than 30m ahead.
        } // end if car ahead in same lane
      } // end for each car id in sensor_fusion data
      break;
      
    case LaneChangeLeft:
      // wait for car position to be at position corresponding to 'lane'
      // This would indicate LaneChange procedure is over.
      // If over, then next state should be KeepLane
      if(isLaneChangeDone(lane, car_d))
      {
        state = KeepLane;
        std::cout << "state --> KeepLane" << std::endl;
      }
      break;

    case LaneChangeRight:
      // wait for car position to be at position corresponding to 'lane'
      // This would indicate LaneChange procedure is over.
      // If over, then next state should be KeepLane
      if(isLaneChangeDone(lane, car_d))
      {
        state = KeepLane;
        std::cout << "state --> KeepLane" << std::endl;
      }
      break;
      
    default:
      break;
  } // end switch(state)

} // end fsm_transition_function()


bool isLaneChangeDone(int lane, double car_d)
/* 
 * Inputs : 
 *			- int lane
 *			- double car_d
 * Return :
 *			- true if car_d correspond to middle of 'lane´
 *			- false if car_d does not correspond yet to middle of 'lane'
 */
{
  bool result = false;
  double diff = fabs(car_d - (2+4*lane));
  if (diff < 0.1) result = true;
  // std::cout << "car_d : " << car_d << ", (2+4*lane) = " << (2+4*lane) << ", diff = " << diff << std::endl;
  return result;
}  // end isLaneChangeOver()
