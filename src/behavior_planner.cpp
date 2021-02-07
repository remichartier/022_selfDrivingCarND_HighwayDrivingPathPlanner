/* fsm.cpp to manage finate state machine
 * History :
 * v001 : first version with fsm_transition_function()
 * v002 : Separate into different states in fsm_transition_function()
 *       Modify fsm_transition_function() + change state
 *       Add switch/case to fsm_transition_function()
 *       Add function isLaneChangeDone()
 * v003 : improve isLaneChangeDone() car_d threshold/range to switch back
 *       to KeepLane state
 * v004 : Need to implement prediction, then based on prediction, need to decide to change
 *       lane to Left or Right while in KeepLane State.
 * v005 : check car in lane too close --> move to function fsm_isCarInLaneTooClose()
 * v006 : introduce possible immediate move analyis via function fsm_next_lanes_possible()
 * v007 : 1st algo to decide KeepLane, Left or Right without studing speed, only distances.
 *
 * Now call behavior_planner.cpp
 * v001 : re-design to use Cost Functions for Lane Change decisions / FSM state changes
 *        Add bp_indexClosestCarAhead(), bp_lane_decider()
 * v002 : Only allow lane change possibilities when forced to speed down
 *        Change bp_adjustAcceleration()
 *        
 */

// IF KEEP LANE, ON S'EN FOUT DE LA VOITURE DE DERRIERE !!!! --> DO NOT COUNT COST FOR CAR BEHIND
//  WHEN CHOICE FOR KEEP LANE .... REPLACE THIS COST BY -1.
  
// PROBLEM AFTER CHANGING LANE TO FIND OUT ... MAY BE USED ANOTHER VARIABLE THAN STATE ????
  
  
#include <iostream> // for cout, endl
#include "behavior_planner.h"
#include "cost.h"

// using std::vector;


// usually : transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights)
void bp_transition_function(int prev_size, double car_s, double car_d, double end_path_s,double &ref_vel,
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

  // variables initialization needed
  vector<fsm_state> possible_steer;
  vector<double> cost_steer;			// cost vector for each steer possibilities
  int index_car_ahead_currentLane;
  int index_car_behind_currentLane;
  bool need_change_lane;

  std::cout << "Enter bp_transition_function()" << std::endl;
  // Do this right away so can decide if needs a change or not
  // in case car ahead it too close ...
  // Note : do this on the current lane
  bp_indexClosestCars(car_s, sensor_fusion, lane, index_car_behind_currentLane,
                        index_car_ahead_currentLane);
  
  // if car ahead < 30m
  // Then need to speed down ... Otherwise, continue to speed up  
  // Speed Up or Down depending of too_close value
  bp_adjustAcceleration(car_s, sensor_fusion, index_car_ahead_currentLane,
                        SAFE_DISTANCE_M, ref_vel, need_change_lane);
  // at this point, need_change_lane gives an indication it is better to change lanes
  // but only do so if current state = KeepLane and not LanechangeLeft or Right
  
  
  // FSM to decide next steps and actions
  switch(state){
      
    case KeepLane :
      std::cout << "FSM KeepLane, need_change_lane = " << need_change_lane << std::endl;
      if(need_change_lane)
      {
        // check what is possible ? Straight, Left, Right ?
        bp_possible_steer(possible_steer,lane); 

        // Now, according to steer possible, evaluate current cost/risk
        // like if Straight, colliding car head ? Speed car ahead, Distance car ahead, 
        // Speed car ahead ? Acceleration car ahead ?
        // Also do same for if Left or Right possible.

        // Note : possible_steer will never be empty, will at least have KeepLane
        // and one of ChangeLaneLeft or ChangeLaneRight, or both of them as well
        // So will always have at least 2, at most 3.

        std::cout << "POSSIBLE CHANGES COSTS #############################" << std::endl ;

        for (int i=0; i < possible_steer.size(); i++)
        {
          double cost(0.0),cost1,cost2,cost3;
          int index_car_ahead;
          int index_car_behind;

          // Start to search index of closest car ahead and behind in this lane
          //index_car_ahead = bp_indexClosestCarAhead(car_s, sensor_fusion,lane);

          switch(possible_steer[i])
          {
            case LaneChangeLeft :
              bp_indexClosestCars(car_s, sensor_fusion, lane-1, index_car_behind,
                                  index_car_ahead);
              break;
            case LaneChangeRight :
              bp_indexClosestCars(car_s, sensor_fusion, lane+1, index_car_behind,
                                  index_car_ahead);
              break;
            default: // if KeepLane, nothing to change
              // bp_indexClosestCars(car_s, sensor_fusion, lane, index_car_behind,
              //            index_car_ahead);

              // Done just before, info in index_car_behind_currentLane and 
              // index_car_ahead_currentLane
              index_car_ahead = index_car_ahead_currentLane;
              index_car_behind = index_car_behind_currentLane;
              break;
          } // end switch()

          // colliding car head ? : similar to distance car ahead ---> skip this one

          // Distance car ahead,
          // want cost function return : 1 if dist < dist_min,--> dist_min/dist
          // ie with be 1 if < dist_min, and decreast propertionnaly if > dist_min ...
          cost1 = cost_car_distance(car_s, sensor_fusion, SAFE_DISTANCE_M,
                                    index_car_ahead);

          // Speed car ahead, 
          // Compare our car ref_vel with next car ahead speed,
          // speed_car_ahead - ref_vel / MAX_SPEED_MPH
          cost2 = cost_car_speed_ahead(ref_vel, sensor_fusion,index_car_ahead);        

          // Acceleration car ahead ?
          // no straight forward info from sensor_fusion on acceleration
          // would need to develop prediction module
          // --> skip for time being

          // Distance car behind ?
          // similar function as for 'Distance car ahead'
          cost3 = cost_car_distance(car_s, sensor_fusion, SAFE_DISTANCE_M,
                                    index_car_behind);

          cost = cost1 + cost2 + cost3;

          std::cout << "Steering = " << possible_steer[i] << ", " ;
          std::cout << "costs = " << cost1 << ", " ;
          std::cout <<  cost2 << ", " ;
          std::cout <<  cost3 << ", " ;
          std::cout << "total = " << cost << ", " ;
          std::cout <<  std::endl ;

          // Speed car behind, 
          // Compare our car ref_vel with closest car behind's speed,
          // speed_car_ahead - ref_vel / MAX_SPEED_MPH
          //cost += cost_car_speed_ahead(ref_vel, sensor_fusion,index_car_behind);  

          // store cost of this steer possibility in cost_steer vector
          cost_steer.push_back(cost);

        } // end for()

        bp_lane_decider(possible_steer, cost_steer, lane, state);

        // Output is state + lane.

      } // end if need_change_lane
      break;

    case LaneChangeLeft:
    case LaneChangeRight:
      std::cout << "FSM LaneChangeLeft or Right, need_change_lane = " << need_change_lane;
      std::cout << ", lane = " << lane << std::endl;

      // wait for car position to be at position corresponding to 'lane'
      // This would indicate LaneChange procedure is over.
      // If over, then next state should be KeepLane
      std::cout << "call bp_isLaneChangeDone() " << std::endl;
      if(bp_isLaneChangeDone(lane, car_d))
      {
        state = KeepLane;
        std::cout << "state --> KeepLane" << std::endl;
      }
      break;
      
    default:
      break;
      
  } // end switch()
  
  std::cout << "Exit bp_transition_function(), state = "<< state << ", lane = " << lane << std::endl;
} // end function

#if 0
void bp_adjustAcceleration(int prev_size, double car_s, double end_path_s, 
                           vector<vector<double>> sensor_fusion, int lane, 
                           int dist_min, double &ref_vel)
/* 
 * Inputs : 
 *			- int prev_size
 *			- double car_s
 *			- double end_path_s
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane
 *			- int dist_min
 *			- double &ref_vel, by reference
 * Outputs :
 *			- double ref_vel
 */
{
  bool too_close;
  too_close = bp_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                     sensor_fusion, lane, SAFE_DISTANCE_M);
  
  // Adjust acceleration depending of cars ahead
  if(too_close)
  {
    // code to adjust incrementally speed changes to avoid crashing into car ahead
    // in the lane
    ref_vel -= MAX_ACCEL;
  } else if(ref_vel < MAX_SPEED_MPH)
  {
    ref_vel += MAX_ACCEL;
  }  
} // end function
#endif //0


void bp_adjustAcceleration(double car_s, vector<vector<double>> sensor_fusion,
                           int index_car_ahead, int dist_min, double &ref_vel,
                           bool &too_close)
/* 
 * Inputs : 
 *			- double car_s
 *			- <vector<vector<double>> sensor_fusion
 *			- int index_car_ahead
 *			- int dist_min
 *			- double ref_vel, by reference
 *			- bool too_close by reference;
 * Outputs :
 *			- double ref_vel
 *			- bool too_close
 */
{  
  double car_ahead_s = sensor_fusion[index_car_ahead][5];
  too_close = false;
  
  std::cout << "bp_adjustAcceleration() : car_ahead_s = " << car_ahead_s ;
  std::cout << ", car_s = " << car_s ;
  
  if((car_ahead_s - car_s) <= dist_min)
  {
    too_close = true;
  
    // Adjust acceleration depending of cars ahead
    // code to adjust incrementally speed changes to avoid crashing into car ahead
    // in the lane
    if(ref_vel - MAX_ACCEL >0)
    {
      ref_vel -= MAX_ACCEL;
    } else
    {
      ref_vel = 0;
    }
    std::cout << ", speed DOWN, ref_vel = " << ref_vel ;
  
  } else if(ref_vel < MAX_SPEED_MPH)
  {
   
    std::cout << ", speed UP, ref_vel = " << ref_vel ;
    ref_vel += MAX_ACCEL;
  }  
  std::cout << ", exit too_close = " << too_close;
  std::cout << std::endl;
} // end function



bool bp_isLaneChangeDone(int lane, double car_d)
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


int bp_indexClosestCarAhead(double car_s, vector<vector<double>> sensor_fusion, 
                               int lane)
  /* 
 * Inputs : 
 *			- double car_s
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane
 * Return : 
 *			- int next_car_index, reused for future cost functions to avoid doing 
 *				the same search again (if no car ahead --> NONE(-1))
 */
{
  double sdiff = COST_DIST_MAX;
  int next_car_index = NONE ;
  // need to find closest car ahead in the same lane !

  for(int i=0; i < sensor_fusion.size(); i++)
  {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if( d < (2+4*lane+2) && d > (2+4*lane-2) )
    {
      double check_car_s = sensor_fusion[i][5];      
      
        if(check_car_s >= car_s)		// if car ahead in the same lane
        {
          // if this car closer than previous sdiff --> sdiff          
          if ((check_car_s - car_s) < sdiff)
          {
            sdiff = (check_car_s - car_s);
            next_car_index = i;
          } // end if < sdiff
        } // end if car ahead 
    } // end if car ahead in same lane
  } // end for each car  
  return next_car_index;
} // end function

int bp_indexClosestCars(double car_s, vector<vector<double>> sensor_fusion, 
                        int lane, int &index_closest_behind,
                        int &index_closest_ahead)
 /* 
 * Inputs : 
 *			- double car_s
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane
 *			- int &index_closest_behind, by reference
 *			- int &index_closest_ahead, by reference
 * Return : 
 *			- int index_closest_ahead, reused for future cost functions to avoid doing 
 *				the same search again (if no car ahead --> NONE(-1))
 *			- int index_closest_behind, reused for future cost functions to avoid doing 
 */
{
  double sdiff_behind = COST_DIST_MAX;
  double sdiff_ahead = COST_DIST_MAX;

  index_closest_behind = NONE;
  index_closest_ahead = NONE;
  // need to find closest car AHEAD/BEHIND in the same lane !

  for(int i=0; i < sensor_fusion.size(); i++)
  {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if( d < (2+4*lane+2) && d > (2+4*lane-2) )
    {
      double check_car_s = sensor_fusion[i][5];      
      
      // 1. Test if closest ahead
      if(check_car_s >= car_s)		// if car BEHIND or AHEAD in the same lane
      {
        // if this car closer than previous sdiff --> sdiff          
        if((check_car_s - car_s) < sdiff_ahead)
        {
          sdiff_ahead = check_car_s - car_s;
          index_closest_ahead = i;
        } // end if < sdiff
      } // end if car ahead 
      
      // 2. Test if closest behind
      if(check_car_s <= car_s)		// if car BEHIND or AHEAD in the same lane
      {
        // if this car closer than previous sdiff --> sdiff          
        if((car_s - check_car_s) < sdiff_behind)
        {
          sdiff_ahead = car_s - check_car_s;
          index_closest_behind = i;
        } // end if < sdiff
      } // end if car ahead 

    } // end if car ahead in same lane
  } // end for each car  
} // end function


void bp_lane_decider(vector<fsm_state> possible_steer, vector<double> cost_steer, 
                     int &lane, fsm_state &state)
 /* 
 * Inputs : 
 *			- vector<fsm_state> possible_steer
 *			- vector<double> cost_steer
 *			- int &lane, by reference
 *			- fsm_state &state, by reference
 * Return : 
 *			- int lane 
 *			- fsm_state state
 */  
{
  // Note : possible_steer will never be empty, will at least have KeepLane
  // and one of ChangeLaneLeft or ChangeLaneRight, or both of them as well
  // So will always have at least 2, at most 3.  

  // Need to choose the steering for which we get the minimum cost
  int index=0;
  double cost = cost_steer[0];
  
  for(int i=1; i < possible_steer.size(); i++)
  {
    if(cost_steer[i] < cost)
    {
      cost = cost_steer[i];
      index = i;
    } // end if
  } // end for
  
  // Now have the index for the steer with the least cost
  // set next lane / state
  switch(possible_steer[index])
  {
    case LaneChangeLeft :
      state = LaneChangeLeft;
      lane -= 1;
      std::cout << "state --> LaneChangeLeft" << std::endl;
      break;
    case LaneChangeRight :
      state = LaneChangeRight;
      lane += 1;
      std::cout << "state --> LaneChangeRight" << std::endl;
      break;
    default: // if KeepLane, nothing to change
      break;
  } // end switch()
  
} // end function


bool bp_isCarInLaneTooClose(int prev_size, double car_s, double end_path_s, 
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

void bp_possible_steer(vector<fsm_state> &possible_steer,int lane)
{
/* 
 * Inputs : 
 *			- vector<fsm_state> &possible_steer, by reference
 *			- int lane
 * Return : 
 *			- vector<fsm_state> possible_steer
 */
  
  // First possibility is always to KeepLane 
  possible_steer.push_back(KeepLane);
  
  // Next possibility Left ? consider if lane -1 possible
  if(lane > LANE_MIN) possible_steer.push_back(LaneChangeLeft);  
  
  // Next possibility Right ? consider if lane +1 possible
  if(lane < LANE_MAX) possible_steer.push_back(LaneChangeRight); 
} // End function



void bp_next_lanes_possible(vector<fsm_state> &possible_next_move, bool &too_close,
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
 *			- int lane 
 * Return : 
 *			- vector<fsm_state> possible_next_move
 *			- bool too_close
 */
{
  too_close = bp_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane, SAFE_DISTANCE_M);
  
  // if no car too close ahead, can keep on same lane
  if(!too_close) possible_next_move.push_back(KeepLane);
  
  // consider if lane -1 possible
  if(lane > LANE_MIN)
  {
    // Left could be possible but need to check if no cars on the Left (lane -1)
    // both at + 30m and - 30m.... SAFE_DISTANCE_M
    bool too_close_left;
    too_close_left = bp_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane-1, SAFE_DISTANCE_M);
    // need to check as well if no car on lane-1 behind at SAFE_DISTANCE_M
    too_close_left |= bp_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane-1, -SAFE_DISTANCE_M);
    
    if(!too_close_left) possible_next_move.push_back(LaneChangeLeft);  
  }
  
  // consider if lane +1 possible
  if(lane < LANE_MAX)
  {
    // Right could be possible but need to check if no cars on the Right (lane +1)
    bool too_close_right;
    too_close_right = bp_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane+1, SAFE_DISTANCE_M);
    // need to check as well if no car on lane-1 behind at SAFE_DISTANCE_M
    too_close_right |= bp_isCarInLaneTooClose(prev_size, car_s, end_path_s,
                                      sensor_fusion, lane+1, -SAFE_DISTANCE_M);
    
    if(!too_close_right) possible_next_move.push_back(LaneChangeRight); 
  }
} // end bp_next_lanes_possible()
