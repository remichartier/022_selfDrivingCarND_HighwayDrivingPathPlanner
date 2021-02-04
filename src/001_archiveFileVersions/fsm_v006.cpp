/* fsm.cpp to manage finate state machine
 * History :
 * v01 : first version with fsm_transition_function()
 * v02 : Separate into different states in fsm_transition_function()
 *       Modify fsm_transition_function() + change state
 *       Add switch/case to fsm_transition_function()
 *       Add function isLaneChangeDone()
 * v03 : improve isLaneChangeDone() car_d threshold/range to switch back
 *       to KeepLane state
 * v04 : Need to implement prediction, then based on prediction, need to decide to change
 *       lane to Left or Right while in KeepLane State.
 * v05 : check car in lane too close --> move to function fsm_isCarInLaneTooClose()
 * v06 : introduce possible immediate move analyis via function fsm_next_lanes_possible()
 */

#include <iostream> // for cout, endl
#include "fsm.h"

// using std::vector;


// usually : transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights)
void fsm_transition_function(int prev_size, double car_s, double car_d, double end_path_s,double &ref_vel,
                             vector<vector<double>> sensor_fusion, int &lane, fsm_state &state)
{
/* 
 * Inputs : 
 *			- int prev_size
 *			- double car_s
 *			- double car_d
 *			- double end_path_s
 *			- double &ref_vel, by reference
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane by reference
 *			- fsm state : current state, by reference
 * Outputs :
 *			- int lane
 *			- fsm state : next state
 *			- double &ref_vel
 */
 vector<fsm_state> possible_next_move;

  switch(state){
      
    case KeepLane :

      bool too_close;
      // too_close = fsm_isCarInLaneTooClose(prev_size, car_s, car_d, end_path_s,
      //                                         sensor_fusion, lane);
      
      fsm_next_lanes_possible(possible_next_move, too_close, prev_size, car_s,
                              end_path_s, sensor_fusion, lane);
      // return too_close + possible_next_move
      

      
      if(too_close)
      {
        // Need to analyse possible_next_move, if empty or not.
        if(possible_next_move.size()!=0)
        {
          std::cout << "possible_next_move (0 Keep, 1 Left, 2 right) = ";
          for(int i=0; i < possible_next_move.size(); i++)
          {
            std::cout << possible_next_move[i] << ", " ;
          }
          std::cout << std::endl;
        }
        
        if(lane > LANE_MIN)
        {
          lane = LANE_MIN;
          state = LaneChangeLeft;
          std::cout << "state --> LaneChangeLeft" << std::endl;
        }
        // code to adjust incrementally speed changes to avoid crashing into car ahead
        // in the lane
        ref_vel -= MAX_ACCEL;
      } else if(ref_vel < MAX_SPEED_MPH)
      {
        ref_vel += MAX_ACCEL;
      }

      break;
      
    case LaneChangeLeft:
      // wait for car position to be at position corresponding to 'lane'
      // This would indicate LaneChange procedure is over.
      // If over, then next state should be KeepLane
      if(fsm_isLaneChangeDone(lane, car_d))
      {
        state = KeepLane;
        std::cout << "state --> KeepLane" << std::endl;
      }
      break;

    case LaneChangeRight:
      // wait for car position to be at position corresponding to 'lane'
      // This would indicate LaneChange procedure is over.
      // If over, then next state should be KeepLane
      if(fsm_isLaneChangeDone(lane, car_d))
      {
        state = KeepLane;
        std::cout << "state --> KeepLane" << std::endl;
      }
      break;
      
    default:
      break;
  } // end switch(state)

} // end fsm_transition_function()


bool fsm_isLaneChangeDone(int lane, double car_d)
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


bool fsm_isCarInLaneTooClose(int prev_size, double car_s, double end_path_s, 
                            vector<vector<double>> sensor_fusion, int lane, int dist_min)
/* 
 * Inputs : 
 *			- int prev_size
 *			- double car_s
 *			- double end_path_s
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane by reference
 *			- int dist_min (distance to check), if>0 : ahead, if<0,behind 
 * Return : 
 *			- bool True or False
 */
{
  if(prev_size > 0){
    car_s = end_path_s;
  }

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
      if(dist_min >= 0)
      {
        if((check_car_s > car_s) && ((check_car_s - car_s) < dist_min))
        {
          // do some logic here, lower reference velocity, so we don't crash into the car in front of us, could
          // also flag to try to change lanes
          // ref_vel = 29.5; //mph
          // too_close = true;
          return true;

        } // end if car ahead and less than 30m ahead.
      }
      else
      {
		// case(dist_min < 0):
        if((check_car_s < car_s) && ((check_car_s - car_s) < dist_min))
        {
          // do some logic here, lower reference velocity, so we don't crash into the car in front of us, could
          // also flag to try to change lanes
          // ref_vel = 29.5; //mph
          // too_close = true;
          return true;
        } // end if car ahead and less than 30m ahead.
      } // end if dist_min...
    } // end if car ahead in same lane
  } // end for each car id in sensor_fusion data
  // if reached that line without returning true before --> return false
  return false;
}

void fsm_next_lanes_possible(vector<fsm_state> &possible_next_move, bool &too_close,
                            int prev_size, double car_s, double end_path_s, 
                            vector<vector<double>> sensor_fusion, int lane)
/* 
 * Inputs : 
 *			- vector<fsm_state> &possible_next_move, by reference
 *			- bool &too_close, by reference
 *			- int prev_size
 *			- double car_s
 *			- double end_path_s
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane by reference
 * Return : 
 *			- vector<fsm_state> possible_next_move
 *			- bool too_close
 */
{
  too_close = fsm_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane, SAFE_DISTANCE_M);
  
  // if no car too close ahead, can keep on same lane
  if(!too_close) possible_next_move.push_back(KeepLane);
  
  // consider if lane -1 possible
  if(lane > LANE_MIN)
  {
    // Left could be possible but need to check if no cars on the Left (lane -1)
    // both at + 30m and - 30m.... SAFE_DISTANCE_M
    bool too_close_left;
    too_close_left = fsm_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane-1, SAFE_DISTANCE_M);
    // need to check as well if no car on lane-1 behind at SAFE_DISTANCE_M
    too_close_left |= fsm_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane-1, -SAFE_DISTANCE_M);
    
    if(!too_close_left) possible_next_move.push_back(LaneChangeLeft);  
  }
  
  // consider if lane +1 possible
  if(lane < LANE_MAX)
  {
    // Right could be possible but need to check if no cars on the Right (lane +1)
    bool too_close_right;
    too_close_right = fsm_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane+1, SAFE_DISTANCE_M);
    // need to check as well if no car on lane-1 behind at SAFE_DISTANCE_M
    too_close_right |= fsm_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane+1, -SAFE_DISTANCE_M);
    
    if(!too_close_right) possible_next_move.push_back(LaneChangeRight); 
  }
} // end fsm_next_lanes_possible()
