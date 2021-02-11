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
 *        Add debug prints, fix segmentation fault / Core Dumped
 * v003 : comment some debug prints, raise Distance for car behind, and 
 *        if KeepLane, cost for car distance behind --> 0, because should not count.
 *        add changeLaneCounter
 * v004 : add cost_car_cutting_lane_ahead()
 *        Generate trajectories for each prediction (KeepLane, LaneChangeLeft, LaneChangeRight)
 *        Add int bp_next_lane()
 *        Add call to trajectory generation bp_generate_trajectories()
 *        Call prediction_get()
 *        Add bp_compute_cost_states()
 *        Add predictions + car_s_predict as input parameter to bp_compute_cost_states()
 *        Add calls to cost_collided_rear_car() + cost_collision_car_head(), cost_car_buffer()
 *        Adjust costs to avoid collisions
 *        For front / rear car searches, consider also cars starting to change lanes ie 1 meter
 *        away from lane borders --> ( d < (2+(4*lane)+2 +1) && d > (2+(4*lane)-2 -1) )
 * TAGGED v1.0 on Github
 *        Test : Enable Lane change anytime Cost computation judge necessary
 * TAGGED v1.1 on Github (Working version, changing lanes depending of costs
 *        Cleanup of code
 */
  
  
#include <iostream> // for cout, endl
#include <math.h> // for fabs()
#include "behavior_planner.h"
#include "cost.h"
#include "trajectory.h"
#include "predictions.h"
#include "constants.h"

// using std::vector;
using std::cout;
using std::endl;


// usually : transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights)
void bp_transition_function(int prev_size, double car_s, double car_d, double end_path_s,double &ref_vel,
                            vector<vector<double>> sensor_fusion, int &lane, fsm_state &state, 
                            int &changeLaneCounter,
                            double car_x, double car_y, double car_yaw,
                            vector<double> previous_path_x, vector<double> previous_path_y,
                            vector<double> map_waypoints_s, vector<double> map_waypoints_x,
                            vector<double> map_waypoints_y, 
                            vector<double> &next_x_vals, vector<double> &next_y_vals)
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
 *			FOR GENERATING TRAJECTORIES : 
 *			- double car_x, car_y, car_yaw : car position
 *			- vector<double> previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x,
 *                         vector<double> map_waypoints_y
 *			- vector<double> next_x_vals, next_y_vals : array to define new Trajectory (passed by reference) 
 * Outputs :
 *			- int lane
 *			- fsm state : next state
 *			- double &ref_vel
 *			- vector<double> next_x_vals, next_y_vals : array to define new Trajectory (passed by reference) 
 */

  // variables initialization needed
  vector<fsm_state> possible_steer;		// could be KeepLane, LaneChangeLeft, LaneChangeRight
  vector<double> cost_steer;			// cost vector for each steer possibilities
  int index_car_ahead_currentLane;
  int index_car_behind_currentLane;
  bool need_change_lane;				// used if triggering Lane Change only due to car ahead too close

  //std::cout << "Enter bp_transition_function()" << std::endl;
  // Do this bp_indexClosestCars() right away so could decide if needs a lane change or not
  // in case car ahead is too close ...
  // Note : do this on the current lane
  //std::cout << "call bp_indexClosestCars()" << std::endl;
  bp_indexClosestCars(car_s, sensor_fusion, lane, index_car_behind_currentLane,
                        index_car_ahead_currentLane);  
  // Note : Outputs are index_car_behind_currentLane and index_car_behind_currentLane
  
  // if car ahead < 30m or 50m,
  // Then need to speed down ... Otherwise, continue to speed up  
  // Speed Up or Down depending of too_close value
  //std::cout << "call bp_adjustAcceleration()" << std::endl;
  bp_adjustAcceleration(car_s, sensor_fusion, index_car_ahead_currentLane,
                        SAFE_DISTANCE_M, ref_vel, need_change_lane);
  // Note : Output = ref_vel, need_change_lane
  // at this point, need_change_lane gives an indication it is better to change lanes
  // but only do so if current state = KeepLane and not LanechangeLeft or Right
  
  //std::cout << "Enter switch(state)" << std::endl;
  // FSM to decide next steps and actions
  switch(state){
      
    case KeepLane :
      //std::cout << "FSM KeepLane, need_change_lane = " << need_change_lane << std::endl;
      //if(need_change_lane) --> Only allows Lane Change when too close to car ahead
      //if(true) --> Could enable change anytime Cost computation judge necessary
      if(true)
      {
        // check what is possible ? Straight(=KeepLane), Left, Right ?
        //std::cout << "Call bp_possible_steer()" << need_change_lane << std::endl;
        bp_possible_steer(possible_steer,lane); 

#if 0 // CF LATER COMMENTS FOR PREDICTIONS --> NO NEED TO GENERATE ALL THOSE TRAJECTORIES
        // The 'lane' directive and modifications serves to indicate a specific trajectory 
        // which will be generated in main.cpp via trajectory_generation()
        // Now need to generate trajectories for each possible_steer
        vector<vector<double>> trajectories_x;
        vector<vector<double>> trajectories_y;
        
        bp_generate_trajectories(trajectories_x, trajectories_y,possible_steer,lane,
                                 car_x, car_y, car_yaw, car_s, prev_size,
                                 previous_path_x, previous_path_y,
                                 map_waypoints_s, map_waypoints_x,
                                 map_waypoints_y, ref_vel);
#endif // 0
        
        // Now, need to generate predictions (positions of car_s + 30m or 50m)
        // car will be at which t in 30 meters ?
        // ref_vel (mph)= m/h = 30 meters in miles / t
        // t = 30 meters in miles / ref_vel
        // t in hour = 30 * METER2MILE / ref_vel
        // So will have to predict all cars given t and given their own velocity.
        // Then, knowing the indexes of closest next and behind cars, will know their predictions
        // after t duration, and can then calculate the costs.
        double car_s_predict;
        vector<double> predictions;
        
        // NOTE: PREDICTIONS WOULD NEED THE TRAJECTORY TO CALCULATE CAR_S_PREDICT ...
        // IN FACT NOT NECESSARILY, BECAUSE SPECIFICALLY IN THIS PROJECT, WAY TRAJECTORY
        // IS GENERATED GIVES US CERTAINTY AND IF CHANGE LANE, CAR WILL BE IN NEW LANE
        // AFTER 30METERS AS DESIGNED BY THE TRAJECTORY FUNCTION ....
        // IE NO NEED OF PREVIOUS FUNCTION bp_generate_trajectories() ...
        predictions_get(car_s, ref_vel, sensor_fusion, car_s_predict, predictions);
        // Note : Outputs are 'car_s_predict´ and 'predictions'
        
        
        // Now, according to steer possible, evaluate current cost/risk
        // like if Straight, colliding car head ? Speed car ahead, Distance car ahead, 
        // Also do same for if Left or Right possible.

        // Note : possible_steer[] will never be empty, due to way bp_possible_steer()
        // is written. It will at least have KeepLane
        // and one of ChangeLaneLeft or ChangeLaneRight, or both of them
        // So will always have at least 2 elements, and at most 3 elements.

        std::cout << "POSSIBLE CHANGES COSTS #############################" << std::endl ;

        bp_compute_cost_states(car_s, sensor_fusion,possible_steer,lane,
                            index_car_ahead_currentLane, index_car_behind_currentLane,
                            ref_vel, cost_steer, predictions, car_s_predict); 
        // Note : output is 'cost_steer' vector
        
        
        //cout << "call bp_lane_decider()" << endl;
        // This function will compute the minimum cost from cost_steer[],
        // and therefore pick the next state (KeepLane, Right or Left) and 
        // corresponding lane number to apply. This updated lane number value
        // will be used in main() to generate the new trajectory via trajectory_generation()
        // using this new 'lane' updated value
        bp_lane_decider(possible_steer, cost_steer, lane, state, changeLaneCounter);
        // Output is state + lane.

      } // end if need_change_lane
      break;

    case LaneChangeLeft:
    case LaneChangeRight:
      // std::cout << "FSM LaneChangeLeft or Right, need_change_lane = " << need_change_lane;
      // std::cout << ", lane = " << lane << std::endl;

      // wait for car position to be at position (d) corresponding to 'lane'
      // This would indicate LaneChange procedure is over.
      // If over, then next state should be KeepLane
      // std::cout << "call bp_isLaneChangeDone() " << std::endl;
      if(bp_isLaneChangeDone(lane, car_d))
      {
        state = KeepLane;
        std::cout << "state --> KeepLane" << std::endl;
      }
      break;
      
    default:
      break;
      
  } // end switch()
  //std::cout << "Exit switch(state)" << std::endl;

  
  //std::cout << "Exit bp_transition_function(), state = "<< state << ", lane = " << lane << std::endl;
} // end function



void bp_compute_cost_states(double car_s, vector<vector<double>> sensor_fusion, 
                            vector<fsm_state> possible_steer, int lane,
                            int index_car_ahead_currentLane,
                            int index_car_behind_currentLane,
                            double ref_vel, vector<double> &cost_steer,
                            vector<double> predictions, double car_s_predict)
/* 
 * Inputs : 
 *			- double car_s, 
 *			- vector<vector<double>> sensor_fusion
 *			- vector<fsm_state> possible_steer
 *			- int lane
 *			- int index_car_ahead_currentLane,
 *			- int index_car_behind_currentLane,
 *			- double ref_vel
 *			- vector<double> cost_steer, by reference
 * Outputs :
 *			- vector<double> cost_steer
*/
{
  // for each possible steer/trajectory (KeepLane, Left, Right)
  for (int i=0; i < possible_steer.size(); i++)
  {
    double cost(0.0),cost1,cost2(0.0),cost3,cost4(0.0),cost5,cost6,cost7(0);
    int index_car_ahead;
    int index_car_behind;

    // Start to search index of closest car ahead and behind in the lane
    // corresponding to the possible steer (lane if KeepLane, lane+1 if Right,
    // lane-1 if Left. This next_lane value is computed via bp_next_lane()
    int next_lane = bp_next_lane(possible_steer[i], lane);
    // If keepLane, we already computed the closest car index behind and ahead
    // If Right or Left, go get those indexes via bp_indexClosestCars()
    if(possible_steer[i] != KeepLane)
    {
      bp_indexClosestCars(car_s, sensor_fusion, next_lane, index_car_behind,
                          index_car_ahead);
      // Note : Outputs :  index_car_behind, index_car_ahead
    } else
    {
      // Done just before, info in index_car_behind_currentLane and 
      // index_car_ahead_currentLane
      index_car_ahead = index_car_ahead_currentLane;
      index_car_behind = index_car_behind_currentLane;
    }

    // Colliding car ahead ?
    // Compare car_s_predict with index_car_ahead s prediction, if <= car_s_predict
    // return 1, otherwise 0
    cost1 = cost_colliding_car_ahead(index_car_ahead, predictions, car_s_predict);
    
    // Collided by rear car ?
    // Compare car_s_predict with index_car_behind s prediction, if >= car_s_predict
    // return 1, otherwise 0
    // Note : only for Left or Right. If KeepLane, generally car_behind is not a risk/cost
    // to decide switching lanes
    if(possible_steer[i] != KeepLane)
    {
      cost2 = cost_collided_rear_car(index_car_behind, predictions, car_s_predict);
    }

    // Cost buffer car ahead of SAFE_DISTANCE_M
    // want cost function return : 1 if dist < dist_min,--> dist_min/dist
    // ie with be 1 if < dist_min, and decrease propertionnaly if > dist_min ...
    cost3 = cost_car_buffer(car_s_predict, predictions, SAFE_DISTANCE_M,
                              index_car_ahead);
    
    // Cost buffer car behind of SAFE_DISTANCE_M
    // want cost function return : 1 if dist < dist_min,--> dist_min/dist
    // ie with be 1 if < dist_min, and decrease proportionnaly if > dist_min ...
    // Note : Only for Left or Right. If KeepLane, generally car_behind is not a risk/cost
    // to decide switching lanes
    if(possible_steer[i] != KeepLane)
    {
      cost4 = cost_car_buffer(car_s_predict, predictions, SAFE_DISTANCE_M,
                              index_car_behind);
    }
    
    // Speed car ahead, 
    // Compare our car ref_vel with next car ahead speed, prefering to switch to
    // higher speed lanes compare to lower speed lanes
    // ref_vel - speed_car_ahead / MAX_SPEED_MPH
    cost5 = cost_car_speed_ahead(ref_vel, sensor_fusion,index_car_ahead);   
    
    
    //cost = cost1 + cost2 + cost3 + cost4 + cost5 + cost6 + cost7;
    cost = cost1 + cost2 + cost3 + cost4 + cost5;
    
    std::cout << "Steering = " << possible_steer[i] << ", " ;
    std::cout << "costs = " << cost1 << ", " ;
    std::cout <<  cost2 << ", " ;
    std::cout <<  cost3 << ", " ;
    std::cout <<  cost4 << ", " ;
    std::cout <<  cost5 << ", " ;
    //std::cout <<  cost6 << ", " ;
    //std::cout <<  cost7 << ", " ;
    std::cout << "total = " << cost << ", " ;
    std::cout <<  std::endl ;

    // Speed car behind, 
    // Compare our car ref_vel with closest car behind's speed,
    // speed_car_ahead - ref_vel / MAX_SPEED_MPH
    //cost += cost_car_speed_ahead(ref_vel, sensor_fusion,index_car_behind);  

    // store cost of this steer possibility in cost_steer vector
    cost_steer.push_back(cost);

  } // end for()
  
}

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
  too_close = false;
  //cout << "too_close = false;" << endl;
  double car_ahead_s;

  if(index_car_ahead != NONE)
  {
    car_ahead_s = sensor_fusion[index_car_ahead][5];
    //cout << "double car_ahead_s = sensor_fusion[index_car_ahead][5]; =" << index_car_ahead <<","<<sensor_fusion[index_car_ahead][5]<<  endl;

    //std::cout << "bp_adjustAcceleration() : car_ahead_s = " << car_ahead_s ;
    //std::cout << ", car_s = " << car_s ;
  
    if((car_ahead_s - car_s) <= dist_min)
    {
      too_close = true;
    }
  }
  if(too_close == true){
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
    std::cout << ", speed DOWN (" << (car_ahead_s - car_s) << " meters, ref_vel = " << ref_vel << std::endl ;
  
  } else if(ref_vel < MAX_SPEED_MPH)
  {
    std::cout << ", speed UP, ref_vel = " << ref_vel << std::endl;
    ref_vel += MAX_ACCEL;
  }  
  //std::cout << ", exit too_close = " << too_close;
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


int bp_next_lane(fsm_state state, int lane)
/* 
 * Inputs : 
 *			- fsm_state state
 *			- int lane
 * Return :
 *			- int next lane
 */
{
  if (state == KeepLane) return lane;
  if (state == LaneChangeLeft) return(lane -1);
  if (state == LaneChangeRight) return(lane +1);
} // end function


void bp_generate_trajectories(vector<vector<double>> &trajectories_x,
                              vector<vector<double>> &trajectories_y,
                              vector<fsm_state> possible_steer, int lane,
                              double car_x, double car_y, double car_yaw, double car_s, int prev_size,
                              vector<double> previous_path_x, vector<double> previous_path_y,
                              vector<double> map_waypoints_s, vector<double> map_waypoints_x,
                              vector<double> map_waypoints_y, double ref_vel)
/* INPUTS :
 *			- vector<double> trajectories_x, trajectories_y, by reference : array to define new Trajectories
 *			- vector<fsm_state> possible_steer
 *			- int lane
 *			- double car_x, car_y, car_yaw, car_s : car position
 *			- int prev_size : prev_size of points to reuse to build coming Trajectory
 *			- vector<double> previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x,
 *                         vector<double> map_waypoints_y
 *			- double ref_vel : vehicle velocity
 *			- vector<double> next_x_vals, next_y_vals : array to define new Trajectory (passed by reference)
 * OUTPUTS : 
 *			- - vector<double> trajectories_x, trajectories_y : array to define new Trajectories
 *			- 
 */
{
  for (int i=0; i < possible_steer.size(); i++)
  {
    vector<double> path_x;
    vector<double> path_y;
    int next_lane = bp_next_lane(possible_steer[i],lane);

    trajectory_generation(car_x, car_y, car_yaw, car_s, prev_size,
                          previous_path_x, previous_path_y,
                          map_waypoints_s, map_waypoints_x, map_waypoints_y,
                          next_lane, ref_vel, path_x, path_y);

    trajectories_x.push_back(path_x);
    trajectories_y.push_back(path_y);
  } // end for
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
    // if( d < (2+4*lane+2) && d > (2+4*lane-2) )
    // to also consider cars starting to change line and cutting 
    // our cars path, consider if next car / rear car 1meter away from 
    // lane borders
    if( d < (2+(4*lane)+2 +1) && d > (2+(4*lane)-2 -1) )
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
                     int &lane, fsm_state &state, int &changeLaneCounter)
 /* 
 * Inputs : 
 *			- vector<fsm_state> possible_steer
 *			- vector<double> cost_steer
 *			- int &lane, by reference
 *			- fsm_state &state, by reference
 *			- int changeLaneCounter, by reference
 * Return : 
 *			- int lane 
 *			- fsm_state state
 *			- int changeLaneCounter
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
      changeLaneCounter ++;
      std::cout << "state --> LaneChangeLeft(#" << changeLaneCounter << ")" <<std::endl;
      break;
    case LaneChangeRight :
      state = LaneChangeRight;
      lane += 1;
      changeLaneCounter ++;
      std::cout << "state --> LaneChangeRight(#" << changeLaneCounter << ")" <<std::endl;
      break;
    default: // if KeepLane, nothing to change
      break;
  } // end switch()
  
} // end function


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

