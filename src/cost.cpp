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
 * TAGGED v1.0 on Github : Working version, Change Lane if too close to car ahead
 *        Test : Enable Lane change anytime Cost computation judge necessary
 * TAGGED v1.1 on Github (Working version, changing lanes depending of costs
 *        Cleanup of code 
 *        Clean comments
 */

#include <iostream> // for cout, endl
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

