This Write-up includes : 
- The rubric points.
  - Brief description of how I addressed each point and references to the relevant code.
  - ie including detailed description of the code used in each step (mainly via code snippets where appropriate) 
  - Links to other supporting documents or external references.
- Model Description : explaining the code model for generating paths, how the code works and why I wrote it that way.

# Project Rubric Criteria

| Criteria Compilation| Meets Specifications |
|-----------|----------------|
|Code compiles correctly.|Code must compile without errors with cmake and make.|

```
./buildremi.sh 
-- Configuring done
-- Generating done
-- Build files have been written to: /home/workspace/CarND-Path-Planning-Project/build
[100%] Built target path_planning
```

| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|Car able to drive at least 4.32 miles without incident| This implementation allows the car to drive more than one lap without incidents as defined in below criterias|
|car drives according to the speed limit| The implementation sets the maximum speed to 49.5 mph for the car, ensuring the car does not exceed the speed limit of 50 mph. It also ensures to not drive slower than speed limit by always keep accelerating until 49.5 mph unless car becomes too close to any cars ahead in the same lane. And this is also done by having a Cost Function which is encouraging to switch Lanes towards faster lane speeds rather than any other lane speeds. This is done thanks to the following pieces of code spread in different files/functions :|
```
// constants.h
// ===========
#define MAX_SPEED_MPH 				49.5
#define MAX_ACCEL 					0.224

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
|Max Acceleration and Jerk are not Exceeded| To keep the car not exceeding a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3, I followed the project course video recommendations by only applying positive or negative acceleration or speed increases by MAX_ACCEL (0.224 mph), and by keeping the implementation suggestion for the Trajectory Generation of waypoints spaced by 30 meters, and also by using smooth transitions between current trajectory driving by car, and new trajectory generated, by keeping 5 waypoints from current trajectory before generating a brand new trajectory via function trajectory_generation(), and all this is done via the code mentioned above to adjust speed steps using MAX_ACCEL (0.224 mph) as well as the Trajectory Generation pieces of code mentioned below :|
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
...
          trajectory_generation(car_x, car_y, car_yaw, car_s, prev_size,
                               previous_path_x, previous_path_y,
                               map_waypoints_s, map_waypoints_x, map_waypoints_y,
                               lane, ref_vel, next_x_vals, next_y_vals);

// trajectory.cpp / trajectory_generation()
// ========================================
// In Frenet add evenly 30m spaced points ahead of the starting reference.
  // NOTE : GOOD TRAJECTORY FOR FOLLOWING TRACK, BUT NOT FOR TURNING LEFT OR RIGHT ...
  vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x,map_waypoints_y);
```

| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|Car does not have collisions| Compliance with this requirement was done in several parts of the code. One piece of implementation is by analysing for each execution of h.onMessage(), the code analyses from sensor_fusion data (done via function bp_indexClosestCars()) if any vehicle ahead of our own car is at a distance less than 50m. It if is the case, it will decrease incrementally the speed (code of function bp_adjustAcceleration() already showed in above criteria. Another part of implementation is done via several cost functions, used when analysing candidate trajectories, to reward trajectories whith buffer distances higher than 50 meters from the next car of the lane considered, and penalize the trajectories with lower buffer distances below 50 m with the next car ahead in the same lane. And there is another cost function rewarding or penalizing tranjectories resulting in collisions as well. Another feature I had to implement in order to prevent collisions was also to consider any cars predicted in different lanes compared to our own car, but starting to deviate from the center of their lanes, and deviating towards the candidate lane of the the considered trajectory. And therefore I selected car predictions from sensor_fusion data, which where approaching too much the lane borders from the candidate lane, withing 1 meter of the border lane. This allowed to penalize trajectories for which the risk of an adjacent lane car would "cut the road" and cross our car trajector at the very last second. Another part of implementation is re-using all the solutions to avoid bumping in other cars ahead, for the closest cars behind our car as well. Below pieces of code are giving examples of those implementations to avoid collisions :|
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

// cost.cpp functions cost_colliding_car_ahead(), cost_collided_rear_car(), and cost_car_buffer()
=================================================================================================
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


// Collided by rear car ?
// Compare car_s_predict with index_car_behind s prediction, if >= car_s_predict
// return 1, otherwise 0
double cost_collided_rear_car(int index_car_behind, vector<double> predictions, 
                              double car_s_predict)
/* 
 * Inputs : 
 *			- int index_car_behind, reused to avoid doing the same search again 
 *				(if no car ahead --> NONE(-1)) across several cost functions
 *			- vector<double> predictions
 *			- double car_s_predict
 * Return : 
 *			- double cost
 */
{
  if(index_car_behind == NONE)
  {
    // cout << "collision rear = " << " no cars; ";
    return 0;
  }
  // if car ahead exists on same lane :
  
  double s = predictions[index_car_behind];
  // here s contains the s prediction coordinate of the rear car behind car_s in the same lane
  double dist = car_s_predict - s;
  // cout << "collision rear = " << dist << " meters; ";
  if(dist <= 0)
  {
    return 1;  
  }
} // end function


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

// 

// Consider cars ahead starting to deviate from the center of their lanes, and deviating towards the candidate lane of the the considered trajectory
// in behavior_planner.cpp, function bp_indexClosestCars() 
// =================================================================================================================================================
// if( d < (2+4*lane+2) && d > (2+4*lane-2) )
    // to also consider cars starting to change line and cutting 
    // our cars path, consider if next car / rear car 1meter away from 
    // lane borders
    if( d < (2+(4*lane)+2 +1) && d > (2+(4*lane)-2 -1) )

```


| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|car stays in its lane, except for the time between changing lanes| I complied to this requirement by using the trajectory generation powered with spline.h and suggested by the project course video, only generating splines for which lanes are switched within a 30 meters distance, and for which the splines align with the middle of the target lane at the end of the first 30 meters of the spline trajectory generation and stays in this middle lane for the next 60 meters after. Also, to keep the car withing the 3 lanes on the right hand side of the road, I restrict the lane numbers to be only between 0 to 2, via LANE_MAX and LANE_MIN constants ie the 3 right hand side lanes of the road, and I apply this restriction when considering other lane trajectories. All those compliances are implemented via the following pieces of code, some of them already partly mentioned in above criterias : |
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
```

The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road


| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|car is able to change lanes|The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic|

| Criteria Reflection| Meets Specifications |
|-----------|----------------|
|There is a reflection on how to generate paths|The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation|
|||
|||
|||
