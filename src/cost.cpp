/* cost.cpp to gather cost functions
 * History :
 * v01 : first version, cost_car_distance_ahead(),
 *       cost_car_speed_ahead()
 *       debug prints for distance and speed, correction speed
 *       comparison, speed in m/s, need to convert into mph
 * v02 : add cost_car_cutting_lane_ahead()
 */

#include <iostream> // for cout, endl
#include <stdlib.h> // for EXIT_FAILURE
#include <math.h> // for sqrt()
#include "cost.h"
#include "constants.h"
#include "sensor_fusion.h"

using std::cout;
using std::endl;

// Distance car ahead,
// want cost function return : 1 if dist < dist_min,--> dist_min/dist
// ie with be 1 if dist_min, and decreast propertionnaly if > dist_min ...
double cost_car_distance(double car_s, vector<vector<double>> sensor_fusion, 
                               int dist_min, int next_car_index)
/* 
 * Inputs : 
 *			- double car_s
 *			- <vector<vector<double>> sensor_fusion
 *			- int dist_min (distance to check), if>0 : ahead, if<0,behind 
 *			- int next_car_index, reused to avoid doing the same search again 
 *				(if no car ahead --> NONE(-1)) across several cost functions
 * Return : 
 *			- double cost
 */
{
  if(next_car_index == NONE)
  {
    cout << "distance = " << " no cars; ";
    return -1;
  }
  // if car ahead exists on same lane :
  
  double s = sensor_fusion[next_car_index][5];;
  // here s contains the coordinate of the next car ahead of car_s in the same lane
  double dist = s - car_s;
  cout << "distance = " << dist << " meters; ";
  if(dist == 0)
  {
    std::cout << "cost_car_distance_ahead() ERROR : dist = 0 can not divide by 0" << std::endl;
    std::cout << "This causes program TERMINATION - exit(EXIT_FAILURE)" << std::endl;
    exit(EXIT_FAILURE);
  }

  return(dist_min / dist);
} // end function


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
    cout << "speed_ahead = " << "no cars; " ;
    return -1;
  }
  // retrieve next car speed
  double speed_ahead = get_index_speed_milesph(sensor_fusion, int next_car_index);
#if 0  
  double vx = sensor_fusion[next_car_index][3];
  double vy = sensor_fusion[next_car_index][4];
  double speed_ahead = sqrt(vx*vx + vy*vy); // Note : this is in m/s
  speed_ahead *= METERPERSECOND2MPH; // Note : converting to MPH
  cout << "speed_ahead = " << speed_ahead << " mph; " ;
#endif // 0
  return ((ref_vel - speed_ahead) / MAX_SPEED_MPH);
  
} // end function

// Cost car ahead in current lane switching to same lane 
double cost_car_cutting_lane_ahead(vector<vector<double>> sensor_fusion, int lane, fsm_state action, int index_car_ahead)
/* 
 * Inputs : 
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane (current lane)
 *			- fsm_state action (action considered)
 *			- int next_car_index, reused to avoid search in sensor_fusion 
 *				(if no car ahead --> NONE(-1))
 * Return : 
 *			- double cost
 */
{
  int lane_candidate;
  double diff_ratio;
  if(index_car_ahead == NONE) return 0.0;
  if(action == KeepLane) return 0.0;
  //if(action == ChangeLaneRight) lane_candidate = lane + 1;
  //if(action == ChangeLaneLeft) lane_candidate = lane -1;
  double d_car_ahead = sensor_fusion[index_car_ahead][6];
  
  if(action == LaneChangeRight)
  // need to measure deviation from (2 + 4*lane) to (4 + 4*lane) and divide by 2
  {
    diff_ratio = ((4 + 4*lane) - d_car_ahead)/ 2;
  }
  else
  {
    // need to measure deviation from (4*lane) and (2 + 4*lane) and divide by 2
	diff_ratio = ((2 + 4*lane) - d_car_ahead)/ 2;
  }
  
  return diff_ratio;
  
}