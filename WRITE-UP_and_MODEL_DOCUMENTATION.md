Write-up including : 
- The rubric points.
  - Brief description of how I addressed each point and references to the relevant code.
- Detailed description of the code used in each step (with line-number references and code snippets where appropriate) 
- Links to other supporting documents or external references.
- Explain how the code works and why I wrote it that way.

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
|Car does not have collisions| Compliance with this requirement was done in several parts of the code. One piece of implementation is by analysing for each execution of h.onMessage(), the code analyses from sensor_fusion data (done via function bp_indexClosestCars()) if any vehicle ahead of our own car is at a distance less than 50m. It if is the case, it will decrease incrementally the speed (code of function bp_adjustAcceleration() already showed in above criteria. Another part of implementation is done via a cost function, used when analysing candidate trajectories, to reward trajectories which buffer distances higher than 50 meters with the next car of the lane considered, and penalize the trajectories with lower buffer distances below 50 m with the next car ahead in the same lane.
Another features I had to implement in order to prevent collisions was also to consider any cars predicted in different lanes compared to our own car, but starting to deviate from the center of their lanes, and deviating towards the candidate lane of the the considered trajectory. And therefore I selected car predictions from sensor_fusion data, which where approaching too much the lane borders form the candidate lane, withing 1 meter of the border lane. Below pieces of code are giving examples of those implementations to avoid collisions :|
```
```


| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|car stays in its lane, except for the time between changing lanes|The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road|
|car is able to change lanes|The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic|

| Criteria Reflection| Meets Specifications |
|-----------|----------------|
|There is a reflection on how to generate paths|The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation|
|||
|||
|||
