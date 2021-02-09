/* cost.cpp to gather cost functions
 * History :
 * v01 : first version, cost_car_distance_ahead(),
 *       cost_car_speed_ahead()
 *       debug prints for distance and speed, correction speed
 *       comparison, speed in m/s, need to convert into mph
 * v02 : add cost_car_cutting_lane_ahead()
 *       add cost_colliding_car_ahead()
 *       add cost_collided_rear_car()
 *       add cost_car_buffer()
 */

#include <iostream> // for cout, endl
#include <stdlib.h> // for EXIT_FAILURE
#include <math.h> // for sqrt() + fabs()?
#include "cost.h"
#include "constants.h"
#include "sensor_fusion.h"

using std::cout;
using std::endl;


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
    cout << "collision rear = " << " no cars; ";
    return 0;
  }
  // if car ahead exists on same lane :
  
  double s = predictions[index_car_behind];
  // here s contains the s prediction coordinate of the rear car behind car_s in the same lane
  double dist = car_s_predict - s;
  cout << "collision rear = " << dist << " meters; ";
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
    cout << "buffer = " << " no cars; ";
    return 0;
  }
  // if car ahead exists on same lane :
  
  double s = predictions[index_car];
  // here s contains the coordinate of the next car ahead of car_s in the same lane
  double dist = fabs(s - car_s_predict);
  cout << "buffer = " << dist << " meters; ";
  if(dist == 0)
  {
    return 1;
  }
  return(dist_min / dist);  
} // end function




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
  double dist = fabs(s - car_s);
  cout << "distance = " << dist << " meters; ";
  if(dist == 0)
  {
    return 1;
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
    return 0;
  }
  // retrieve next car speed
  double speed_ahead = sf_get_speed_milesph(sensor_fusion, next_car_index);
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