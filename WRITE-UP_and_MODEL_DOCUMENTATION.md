This Write-up includes : 
- The rubric points.
  - Brief description of how I addressed each point and references to the relevant code.
  - ie including detailed description of the code used in each step (mainly via code snippets where appropriate) 
  - Links to other supporting documents or external references.
- Model Description : explaining the code model for generating paths, how the code works and why I wrote it that way.

# Project Rubric points

| Criteria Compilation| Meets Specifications |
|-----------|----------------|
|Code compiles correctly.|Code must compile without errors with cmake and make.|

```
root@18af971d902a:/home/workspace/CarND-Path-Planning-Project/build# ./buildremi.sh 
-- Configuring done
-- Generating done
-- Build files have been written to: /home/workspace/CarND-Path-Planning-Project/build
Scanning dependencies of target path_planning
[ 12%] Building CXX object CMakeFiles/path_planning.dir/src/behavior_planner.cpp.o
[ 25%] Linking CXX executable path_planning
[100%] Built target path_planning
root@18af971d902a:/home/workspace/CarND-Path-Planning-Project/build# 
```

| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|Car able to drive at least 4.32 miles without incident| This implementation allows the car to drive more than one lap without incidents as defined in below criterias, cf screenshot below : |
(Simulator Screenshot after more than one highway lap driven)[20210213_154733_ScreenshotSimulatorPastOneLap.png]

| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|car drives according to the speed limit| The implementation sets the maximum speed to 49.5 mph for the car, ensuring the car does not exceed the speed limit of 50 mph. cf following pieces of code spread in different files/functions with futher explanations as shown below : |
```
// constants.h
// ===========
#define MAX_SPEED_MPH 				49.5
#define MAX_ACCEL 					0.224
```
It also ensures to not drive slower than speed limit by always keeping accelerating until 49.5 mph unless car becomes too close to any cars ahead in the same lane. cf the code below : 
```
// behavior_planner.cpp, function bp_adjustAcceleration()
// ======================================================
// Ensures to not drive slower than speed limit
// And ensuring the car does not exceed the speed limit of 50 mph
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
```
And this is also done by having a Cost Function which is encouraging to switch Lanes towards faster lane speeds rather than any other slower lane speeds. This is done thanks to the following pieces of code below :
```
// cost.cpp / cost_car_speed_ahead() function used to lean towards lane with faster speed cars ahead
// =================================================================================================
// behavior_planner.cpp will screen any lane candidate for the next car ahead and 
// use a cost function to quantify difference between current car speed (ref_vel)
// and speed of car ahead in the candidate lane

// Cost Speed car ahead, 
// Compare our car ref_vel with next car ahead speed,
// ref_vel - speed_car_ahead  / MAX_SPEED_MPH
double cost_car_speed_ahead(double ref_vel,  vector<vector<double>> sensor_fusion, 
                            int next_car_index)
/* 
 * Inputs : 
 *			- double ref_vel : vehicule speed
 *			- <vector<vector<double>> sensor_fusion
 *			- int next_car_index, reused to avoid search in sensor_fusion 
 *				(if no car ahead --> NONE(-1))
 * Return : 
 *			- double cost
 */
{
  if(next_car_index == NONE) 
  {
    // cout << "speed_ahead = " << "no cars; " ;
    return 0;
  }
  // retrieve next car speed
  double speed_ahead = sf_get_speed_milesph(sensor_fusion, next_car_index);
  return ((ref_vel - speed_ahead) / MAX_SPEED_MPH);
  
} // end function
```

| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|Max Acceleration and Jerk are not Exceeded| To keep the car not exceeding a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3, I followed the project course video recommendations by only applying positive or negative acceleration or speed increases by MAX_ACCEL (0.224 mph), cf bp_adjustAcceleration() shown above, and by keeping the implementation suggestion for the Trajectory Generation of waypoints spaced by 30 meters, cf code below : |
```
// trajectory.cpp / trajectory_generation()
// ========================================
// In Frenet add evenly 30m spaced points ahead of the starting reference.
  // NOTE : GOOD TRAJECTORY FOR FOLLOWING TRACK, BUT NOT FOR TURNING LEFT OR RIGHT ...
  vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);
```
And also by using smooth transitions between current trajectory driving by car, and new trajectory generated, by keeping 5 waypoints from current trajectory before generating a brand new trajectory via function trajectory_generation(), shown below :
```
// main.cpp
// ========
#define REUSE_PREVIOUS_PATH_SIZE 	5
...
         // For Trajectory Generation, only reuse 5 first previous_path points
          // And then for rest of 50 points, generate using Trajectory Generation function
          // or Trajectory chosen by Behavior Planner and generated via Trajectory Generation function
          
          int prev_size = previous_path_x.size();
          prev_size = (REUSE_PREVIOUS_PATH_SIZE<prev_size) ? REUSE_PREVIOUS_PATH_SIZE : prev_size;
          
          trajectory_generation(car_x, car_y, car_yaw, car_s, prev_size,
                               previous_path_x, previous_path_y,
                               map_waypoints_s, map_waypoints_x, map_waypoints_y,
                               lane, ref_vel, next_x_vals, next_y_vals);
```

| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|Car does not have collisions| Compliance with this requirement was done in several parts of the code. One piece of implementation is by analyzing for each execution of h.onMessage(), the code analyses from sensor_fusion data (done via function bp_indexClosestCars()) if any vehicle ahead of our own car is at a distance less than 50m. It if is the case, it will decrease incrementally the speed (code of function bp_adjustAcceleration() already showed in above criteria. cf code below for bp_indexClosestCars() : |
```
// constants.h
// ===========
#define SAFE_DISTANCE_M				50
#define SAFE_DISTANCE_BEHIND_M		50

// behavior_planner.cpp, function bp_indexClosestCars()
// ====================================================
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
```
Another part of implementation is done via several cost functions, used when analyzing candidate trajectories, to reward trajectories with buffer distances higher than 50 meters from the next car of the lane considered, and penalize the trajectories with lower buffer distances below 50 m with the next car ahead in the same lane. cf code below : 
```
// cost.cpp / and cost_car_buffer() :
=====================================
// Cost buffer car ahead of SAFE_DISTANCE_M
// want cost function return : 1 if dist < dist_min,--> dist_min/dist
// ie with be 1 if < dist_min, and decrease propertionnaly if > dist_min ...
double cost_car_buffer(double car_s_predict, vector<double> predictions, double dist_min,
                              int index_car)
/* 
 * Inputs : 
 *			- double car_s_predict
 *			- vector<double> predictions
 *			- double dist_min (buffer distance)
 *			- int index_car_ahead, reused to avoid doing the same search again 
 *				(if no car ahead --> NONE(-1)) across several cost functions
 * Return : 
 *			- double cost
 */
{
  if(index_car == NONE)
  {
    // cout << "buffer = " << " no cars; ";
    return 0;
  }
  // if car ahead exists on same lane :
  
  double s = predictions[index_car];
  // here s contains the coordinate of the next car ahead of car_s in the same lane
  double dist = fabs(s - car_s_predict);
  // cout << "buffer = " << dist << " meters; ";
  if(dist == 0)
  {
    return 1;
  }
  return(dist_min / dist);  
} // end function
```
And there is another cost function rewarding or penalizing trajectories resulting in collisions as well, both front and rear collisions, cf cost functions penalizing front collision below, but of course I have a similar cost function dedicated to penalize rear car collision as well working with the same principles.
```
// cost.cpp / cost_colliding_car_ahead()
========================================
// Cost Colliding car ahead
// if prediction collide --> 1, otherwise --> 0
double cost_colliding_car_ahead(int next_car_index, vector<double> predictions,
                               double car_s_predict)
/* 
 * Inputs : 
 *			- int next_car_index, reused to avoid doing the same search again 
 *				(if no car ahead --> NONE(-1)) across several cost functions
 *			- vector<double> predictions
 *			- double car_s_predict
 * Return : 
 *			- double cost
 */
{
  if(next_car_index == NONE)
  {
    cout << "collision ahead = " << " no cars; ";
    return 0;
  }
  // if car ahead exists on same lane :
  
  double s = predictions[next_car_index];
  // here s contains the coordinate of the next car ahead of car_s in the same lane
  double dist = s - car_s_predict;
  // cout << "collision ahead = nextCarPredict - car_s_predict = " << s << " - " << car_s_predict << " = " << dist << " meters; ";
  cout << "collision ahead = " << dist << " meters; ";
if(dist <= 0)
  {
    return 1;  
  }
} // end function
```
Another feature I had to implement in order to prevent collisions was also to consider any cars predicted in different lanes compared to our own car, which would be in the process of deviating from the center of their lanes, and deviating towards the candidate lane of the the considered trajectory. And therefore I selected car predictions from sensor_fusion data, which where approaching too much the lane borders from the candidate lane, within 1 meter of the border lane. This allowed to penalize trajectories for which the risk of an adjacent lane car would "cut the road" and cross our car trajectory at the very last second. cf implementation below : 

```
// in behavior_planner.cpp, function bp_indexClosestCars() 
// =======================================================
// Consider cars ahead starting to deviate from the center of their lanes, and deviating towards the candidate lane of the the considered trajectory
// if( d < (2+4*lane+2) && d > (2+4*lane-2) )
    // to also consider cars starting to change line and cutting 
    // our cars path, consider if next car / rear car 1meter away from 
    // lane borders
    if( d < (2+(4*lane)+2 +1) && d > (2+(4*lane)-2 -1) )
```
Another part of implementation is re-using all the solutions to avoid bumping in other cars ahead, for the closest cars behind our car as well, as previously mentioned above, by re-using function cost_car_buffer() for the car positioned behind our car. 

And lastly, an important implementation against collisions is to use prediction of Sensor Fusion data, and predict the positions of other cars after my own car would travel 30 meters into a candidate trajectory, and via cost functions, penalize trajectories violating collisions and buffer minimal distances between cars ahead and behind. Predictions calculations are done via the code below and provided to the cost functions to calculate safety buffer costs or collision costs :
```
 // in predictions.cpp, function : predictions_get
 // ==============================================
 
 void predictions_get(double car_s, double ref_vel,
                    vector<vector<double>> sensor_fusion,
                   double &car_s_predict,
                   vector<double> &predictions)
/* 
 * Inputs : 
 *			- double car_s
 *			- double ref_vel
 *			- <vector<vector<double>> sensor_fusion
 *			- double car_s_predict by reference,
 *			- vector<double> predictions, by reference (s predictions for sensor_fusion data) 
 * Outputs :
 *			- double car_s_predict by reference,
 *			- vector<double> predictions, by reference (s predictions for sensor_fusion data) 
 */
{
  // car will be at which t for 30 meters ?
  // ref_vel (mph)= m/h = 30 meters in miles / t
  // t = 30 meters in miles / ref_vel
  // t in hour = 30 * METER2MILE / ref_vel
  // So will have to predict all cars given t and given their own velocity.
  // Then, knowing the indexes of closest next and behind cars, will know their predictions
  // after t duration, and can then calculate the costs.
  
  //cout << "Enter predictions_get()" << endl;
  
  if(ref_vel == 0)
  {
    std::cout << "prediction_get() ERROR : ref_vel = 0 can not divide by 0" << std::endl;
    std::cout << "This causes program TERMINATION - exit(EXIT_FAILURE)" << std::endl;
    exit(EXIT_FAILURE);
  }
  /* want to predict car after 30 meters
   Car travels 30 meters in how much time ?
   30m in miles is 30m*METER2MILE.
   speed (m/h) = 30m*METER2MILE / thour
   thour = 30m*METER2MILE/speed
  */
  double t_hour = SAFE_DISTANCE_M * METER2MILE / ref_vel;
  double t_seconds = t_hour * 60 * 60;
  
  //cout << "t_seconds =" << t_seconds << " for 30 m; "; 
  
  // Now take car about our car prediction
  car_s_predict = car_s + (ref_vel * t_hour / METERPERSECOND2MPH); // Note : car_s is in meters, ref_vel is in miles per hour !!!
  // Now need to convert from miles to meters
  
  // Now take care about our sensor_fusion predictions
  // sensor_fusion data is in meters and seconds !!!
  for(int i=0; i<sensor_fusion.size();i++)
  {
    double speed_mps = sf_get_speed_meterps(sensor_fusion, i); // meter per seconds
    double s_pos = sensor_fusion[i][5];
    double s_next = s_pos + (speed_mps * t_seconds);
    predictions.push_back(s_next);
  } // end for
  
  // cout << "Exit predictions_get()" << endl;
  
  // And so output will be car_s_predict and predictions[]
} // end function

```


| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|car stays in its lane, except for the time between changing lanes| I complied to this requirement by using the trajectory generation powered with spline.h and suggested by the project course video, only generating splines for which lanes are switched within a 30 meters distance, and for which the splines align with the middle of the target lane at the end of the first 30 meters of the spline trajectory generation and stays in this middle lane for the next 60 meters after: |
```
// trajectory.cpp / trajectory_generation()
// ========================================
// In Frenet add evenly 30m spaced points ahead of the starting reference.
  // NOTE : GOOD TRAJECTORY FOR FOLLOWING TRACK, BUT NOT FOR TURNING LEFT OR RIGHT ...
  vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
```
Also, to keep the car within the 3 lanes on the right hand side of the road, I restrict the lane numbers to be only between 0 to 2, via LANE_MIN and LANE_MAX constants ie the 3 right hand side lanes of the road, and I apply this restriction whenever considering other lane trajectories. All those compliances are implemented via the following pieces of code, some of them already partly mentioned in above criteria :
```

// constants.h
// ===========
#define LANE_MAX					2
#define LANE_MIN					0

// behavior_planner.cpp, function bp_possible_steer()
// ==================================================
void bp_possible_steer(vector<fsm_state> &possible_steer,int lane)
{
/* 
 * Inputs : 
 *			- vector<fsm_state> &possible_steer, by reference
 *			- int lane
 * Output : 
 *			- vector<fsm_state> possible_steer
 */
  
  // First possibility is always to KeepLane 
  possible_steer.push_back(KeepLane);
  
  // Next possibility Left ? consider if lane -1 possible
  if(lane > LANE_MIN) possible_steer.push_back(LaneChangeLeft);  
  
  // Next possibility Right ? consider if lane +1 possible
  if(lane < LANE_MAX) possible_steer.push_back(LaneChangeRight); 
} // End function
```

| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|car is able to change lanes| I implemented several strategies to reach that goal. First one was to implement an entire Behavior Planner to decide when to change lanes. Behavior Planner includes an FSM (Finite State Machine) with states KeepLane, LaneChangeLeft, LaneChangeRight. When the car is in a specific lane, it retrieves from Sensor Fusion the index of the car ahead and the car behind. This allows to know the distance to those cars, as well as their speed. cf below implementation : |
```
// behavior_planner.cpp / functions bp_transition_function(), bp_compute_cost_states() and bp_lane_decider()
// ===============================================================================================================
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

  bp_indexClosestCars(car_s, sensor_fusion, lane, index_car_behind_currentLane,
                        index_car_ahead_currentLane);  
  // Note : Outputs are index_car_behind_currentLane and index_car_behind_currentLane
  
  bp_adjustAcceleration(car_s, sensor_fusion, index_car_ahead_currentLane,
                        SAFE_DISTANCE_M, ref_vel, need_change_lane);
  // Note : Output = ref_vel, need_change_lane

  // FSM to decide next steps and actions
  switch(state){
      
    case KeepLane :
      //if(need_change_lane) --> Only allows Lane Change when too close to car ahead
      //if(true) --> Could enable change anytime Cost computation judge necessary
      if(true)
      {
        // check what is possible ? Straight(=KeepLane), Left, Right ?
        bp_possible_steer(possible_steer,lane); 
        
        // Now, need to generate predictions (positions of car_s + 30m or 50m)
        double car_s_predict;
        vector<double> predictions;
        
        predictions_get(car_s, ref_vel, sensor_fusion, car_s_predict, predictions);
        // Note : Outputs are 'car_s_predict´ and 'predictions'
                
        // Now, according to steer possible, evaluate current cost/risk
        // like if Straight, colliding car head ? Speed car ahead, Distance car ahead, 
        // Also do same for if Left or Right possible.

        bp_compute_cost_states(car_s, sensor_fusion,possible_steer,lane,
                            index_car_ahead_currentLane, index_car_behind_currentLane,
                            ref_vel, cost_steer, predictions, car_s_predict); 
        // Note : output is 'cost_steer' vector
        
        
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
      // wait for car position to be at position (d) corresponding to 'lane'
      // This would indicate LaneChange procedure is over.
      // If over, then next state should be KeepLane
      // std::cout << "call bp_isLaneChangeDone() " << std::endl;
      if(bp_isLaneChangeDone(lane, car_d))
      {
        state = KeepLane;
      }
      break;
  } // end switch()
} // end function
```
If the current state is KeepLane, the behavior planner analyses the possible next trajectories (ie what would be the next lanes possible), what would be the car ahead and behind after such lane changes (using prediction calculation of other cars based and Sensor Fusion data), and then the cost functions (Collision ahead or from behind, buffer distance limits, speed of cars in different lanes) allow to decide, if the state is KeepLane, if we should trigger a lane change and to which lane it should be triggered (it would pick the lane for which the cost function is the least  -cf bp_lane_decider()- compared to the other lanes associated costs). Cf the behavior planer cost functions calculation below :
``` 
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

    int next_lane = bp_next_lane(possible_steer[i], lane);
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

    cost1 = cost_colliding_car_ahead(index_car_ahead, predictions, car_s_predict);
    
    if(possible_steer[i] != KeepLane)
    {
      cost2 = cost_collided_rear_car(index_car_behind, predictions, car_s_predict);
    }

    cost3 = cost_car_buffer(car_s_predict, predictions, SAFE_DISTANCE_M,
                              index_car_ahead);
    
    if(possible_steer[i] != KeepLane)
    {
      cost4 = cost_car_buffer(car_s_predict, predictions, SAFE_DISTANCE_M,
                              index_car_behind);
    }
    
    cost5 = cost_car_speed_ahead(ref_vel, sensor_fusion,index_car_ahead);   
    
    cost = cost1 + cost2 + cost3 + cost4 + cost5;
   
    // store cost of this steer possibility in cost_steer vector
    cost_steer.push_back(cost);
  } // end for()
} // end function
```
cf the behavior planner lane decider depending of the output of the cost functions calculation below : 
```
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
```
Once the Behavior Planner decides to switch lane, it switches to state LaneChangeRight or LaneChangeLeft and updates the lane variable, and it keeps on that ChangeLane state until the position of the car matches with the target Lane middle position, in which case it switches the state back to KeepLane state to become ready for another lane change whenever the behavior planner algorithm decides to do so again via the minimization for the cost functions associated to candidate trajectories/lanes. 

When the behavior planner updates the lane variable, it allows the trajectory generation function to generate the next trajectory from the current lane position to the next lane position, and that's how the lane changes happen between lane 0, 1 or 2.

And the cost function to reward or penalize lane with slower moving car ahead (cost_car_speed_ahead() code is already showed above).

The ability to change lane while car is behind a slower moving car is implemented via this cost function (cost_car_speed_ahead()) and also by a cost function penalizing the current lane if the buffer distance with the car ahead goes below threshold SAFE_DISTANCE_M (50 meters). And therefore those 2 cost functions would penalize the candidate trajectory to keep in the current lane by penalizing if the car ahead moves slower than the cars in adjacent lanes and if the buffer distance becomes lower than SAFE_DISTANCE_M, triggering a lane change if adjacent lanes are clear of other traffic. All those functions have been shown earlier above.

And the smooth change is managed in the main() function via the trajectory_generation() function, and the smooth feature is already explained above in the criteria "Max Acceleration and Jerk are not Exceeded". 

| Criteria Reflection| Meets Specifications |
|-----------|----------------|
|There is a reflection on how to generate paths| cf Model Description chapter below describing the path generation in detail|

# Model Description

Goal : 
- Explaining the code model for generating paths in detail, how the code works and why I wrote it that way.

## High Level Description
In order to generate trajectory paths, I started from the example given in the project video course, both to follow the highway waypoints, and also to operate lane change. Every time the simulator would give back the current trajectory left to be run by the car, to our main.cpp to generate the next trajectory, it basically took the current trajectory not yet driven/consumed by the car, and it generated a trajectory of 50 points following the highway waypoints, centered in the middle of a specific lane, and it filled the remaining points of the current trajectory with the next generated trajectory to fill a 50 points trajectory array, to be fed it back to the simulator.

This implementation of path trajectory uses the lane variable. So just by changing the lane variable, this implementation would generate a trajectory between one lane to another, or just following the same lane, using the spline function and library.

I optimized this first implementation for this project purpose by only reusing 5 points from the previous trajectory, provided from the simulator, and filled the next 45 points needed in the trajectory path array to feed the simulator, with any new trajectory that the behavior planner (`behavior_planner.cpp`) would decide to follow, via the variable lane. This would allow smoot transition between previous trajectory planned, and new trajectory generated by the code.

So the simulator, via the `h.onMessage()` function, would call `bp_transition_function()`. This function would decide which lane to select next. it would return the lane variable, which would be injected into the `trajectory_generation()` function to generate the next trajectory path, and send it back to the simulator to move the car ahead on that trajectory. This is done in the `h.onMessage()` function via the following piece of code :  
```
          // Call to FSM TRANSITION FUNCTION to decide KeepLane or ChangeLane Left or Right
          bp_transition_function(prev_size, car_s, car_d, end_path_s, ref_vel,
                                 sensor_fusion, lane, state, changeLaneCounter,
                                 car_x, car_y, car_yaw,
                                 previous_path_x, previous_path_y,
                                 map_waypoints_s, map_waypoints_x, map_waypoints_y,
                                 next_x_vals, next_y_vals);

          
          // call to TRAJECTORY GENERATION, right now to generate trajectory to follow highway waypoints
          // or to change trajectory to change lane if lane variable is changed via earlier function
          // bp_transition_function()
          trajectory_generation(car_x, car_y, car_yaw, car_s, prev_size,
                               previous_path_x, previous_path_y,
                               map_waypoints_s, map_waypoints_x, map_waypoints_y,
                               lane, ref_vel, next_x_vals, next_y_vals);
```
## Zooming-in to the details : `trajectory_generation()` in `trajectory.cpp`

This function gathers the entire trajectory/path generation implementation.

- Based on the current car attributes provided by the simulator (`car_x, car_y, car_yaw, car_s`), 
- Based on the number of points from the previous trajectory (`previous_path_x, previous_path_y`) remaining to be driven and provided by the simulator (`prev_size` which I set to `REUSE_PREVIOUS_PATH_SIZE`(=5))
- Based on the highway map waypoints (`map_waypoints_s, map_waypoints_x, map_waypoints_x`)
- Base on the target `lane` variable, the current velocity of the car `ref_vel`

It will generate a new trajectory/path in cartesian coordinates, composed of 50 points with the coordinates stored in vectors `next_x_vals, next_y_vals`.

```
void trajectory_generation(double car_x, double car_y, double car_yaw, double car_s, int prev_size,
                          vector<double> previous_path_x, vector<double> previous_path_y,
                          vector<double> map_waypoints_s, vector<double> map_waypoints_x,
                          vector<double> map_waypoints_y, int lane, double ref_vel,
                          vector<double> &next_x_vals, vector<double> &next_y_vals)
/* INPUTS :
 *			- double car_x, car_y, car_yaw, car_s : car position
 *			- int prev_size : prev_size of points to reuse to build coming Trajectory
 *			- vector<double> previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x,
 *                         vector<double> map_waypoints_y
 *			- int lane : lane # to follow
 *			- double ref_vel : vehicle velocity
 *			- vector<double> next_x_vals, next_y_vals : array to define new Trajectory (passed by reference)
 * OUTPUTS : 
 *			- - vector<double> next_x_vals, next_y_vals : array to define new Trajectory
 *			- 
 */
```
 
It will do so by first changing the reference and set the new origin (`ref_x, ref_y`) to the current position of the car after the 5 first points we reuse from the previous trajectory, or if there is not previous trajectory (for instance at the startup) at the starting position of the car (`car_x, car_y`) and use the last but one point as well, in order to reuse the yaw orientation from previous trajectory (use current yaw angle if no previous trajectory).
 
And it will store those first 2 points in 2 vectors for x and y (`ptsx,ptsy`). This is the intent of the code below : 
 
```
{
  // Code to follow lane and full track.

  // Create a list of widely spaced x,y waypoints, evenly spaced at 30 m
  // Later, we will interpolate those waypoints with a spline and fill it
  // with more points that control ...
  vector<double> ptsx;
  vector<double> ptsy;

  // Reference x, y, yaw states
  // Reference the starting point as where the car is

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // if path size is almost empty, use the car as starting reference
  if(prev_size < 2)
  {
    // Use two points that make the path tangent to the car
    double prev_car_x = car_x - cos(deg2rad(car_yaw));
    double prev_car_y = car_y - sin(deg2rad(car_yaw));

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else
  {
    // Redefine reference state as previous path's end point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // Use the points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  } 
```
  Using the current car coordinates in Frenet space (`car_s`) and the intended position of the car inside the selected next lane `(2+4*lane)`, plus the highway waypoints in both cartesian and Frenet coordinates (`map_waypoints_s, map_waypoints_x,map_waypoints_y`), the code generates 3 additional cartesian point coordinates (`next_wp0, next_wp1, next_wp2`), following the highway path, thanks to `map_waypoints_s, map_waypoints_x,map_waypoints_y`, and spaced of 30 meters in s coordinate. 
  
  And it will add those 3 new points `next_wp0, next_wp1, next_wp2` to the previous 2 points already stored in `ptsx,ptsy` (So that's where if the current car position is not yet at the middle of the next selected lane, there would be a lane transition between the 2nd point in stored in `ptsx,ptsy`, and the 3rd point 30 meters away but in the right intended targeted lane. And this is done via the code below : 
  
```

  // In Frenet add evenly 30m spaced points ahead of the starting reference.
  // NOTE : GOOD TRAJECTORY FOR FOLLOWING TRACK, BUT NOT FOR TURNING LEFT OR RIGHT ...
  vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
```

Then the code would transform those 5 points stored in `ptsx,ptsy` into car coordinates (origin is at the car position, yaw rate of 0) so at the end of this code block, `ptsx,ptsy` are containing the coordinates of those 5 points in the car coordinates referential/space, this transformation being useful to simplify the implementation following after : 

```
  // transform into car coordinates before building the spline
  for(int i=0; i < ptsx.size(); i++)
  {
    // Shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }
```
From there, the code creates a Spline function `s` and creates a spline curve/trajectory relaying those 5 points together in the car coordinate referential/space :

```
  // Create a spline
  tk::spline s;

  // Set (x,y) points to the spline
  s.set_points(ptsx, ptsy);
```
Then, we start to fill the output trajectory points, by first storing in output coordinate vectors `next_x_vals, next_y_vals` the first 5 points from current trajectory provided by the simulator (`previous_path_x, previous_path_y`) : 
```
  // Start with all of the previous path points from last time
  // Modification to only use the x first previous_path points instead of
  // all the one available
  //for(int i=0; i<previous_path_x.size(); i++)
  for(int i=0; i<prev_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
```
The code follows computing the distance driven by the spline for x = 30 meters in car coordinates 
```
  // Calculate how to break up the spline points so that we travel at our desired reference velocity
  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
  double x_add_on = 0;
```

And this distance is used to spread the waypoints on the spline according to the current velocity of the car, and retrieving the x and y coordinates of the points on the spline, then converting them back to the cartesian space coordinates, and storing them in the output vectors `next_x_vals, next_y_vals` :  
```
  // Fill up the rest of the path planner after filling it with previous points, we aim to output 50 pts
  // Modify previous_path_x.size() --> prev_size to only reuse 5 previous_path points at max.
  // for(int i=1; i <= 50 - previous_path_x.size(); i++)
  for(int i=1; i <= 50 - prev_size; i++)
  {
    double N = target_dist/(0.02*ref_vel/2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate back to normal after rotating it earlier
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

} // end trajectoryGeneration()
```
So that at the end, the output coordinate vectors `next_x_vals, next_y_vals` contain the 50 points representing the trajectory to follow by the simulator, and which will be fed back to the simulator to get the car moving on that trajectory / path and transitioning to the lane chosen and returned by bp_transition_function().

# Room for improvements
## Functions Input parameters
Functions input parameters are very lengthy, ideally if I had to continue working on this project, I would gather some car attributes into one single object, and it would simplify the list of input and output parameters for the functions of this project. For instance : 
- car_s, car_d, ref_vel, car_x, car_y, car_yaw

Same thing could be applied for the waymap coordinates, gather them in a single class/object : 
- map_waypoints_s, map_waypoints_x, map_waypoints_y

And eventually as well every couple of vectors split into one x and y coordinates but representing the same points, could be gathered into classes/objects to simplify the use of those entities especially as input/output parameters of functions, or into other classes/objects :
- previous_path_x, previous_path_y
- next_x_vals, next_y_vals
- car_x, car_y

## Possible trajectory analysis in Behavior Planner code
The Behavior Planner module is not really generating different trajectory paths, it relies on the fact that for each candidate trajectory, if this leads to a lane change, the lane change will be over after 30 meters ahead, and therefore the code takes this prediction point to calculate cost of safety buffer and collision risk to programmatically pick the trajectory/lane choice with the least total cost value.

Ideally, Possible trajectories should be provided to Behavior Planner and used to calculate Costs. For the purpose of this project and for the sake of simplification, I took the shortcut of only considering the lane numbers as and indication of candidate trajectories, rather than the trajectory paths themselves, taking into account that the way the trajectories are generated means that no matter what, a change of lane would be done after 30 meters driven through those trajectories. 
