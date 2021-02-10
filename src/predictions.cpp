/* prediction.cpp to manage prediction of sensor_fusion data
 * History :
 * v001 : first version with fsm_transition_function()
 *        Change prediction from 30.0 meters to SAFE_DISTANCE_M
 *        Correct car_s_predict
 */

#include <iostream> // for cout, endl
#include <stdlib.h> // for EXIT_FAILURE
#include "constants.h"
#include "sensor_fusion.h"
#include "predictions.h"

using std::cout;
using std::endl;

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
  
  // cout << "Enter predictions_get()" << endl;
  
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
  car_s_predict = car_s + (ref_vel * t_hour); // Note : ref_vel is in miles per hour !!!
  // Now need to convert from miles to meters
  car_s_predict = car_s_predict / METER2MILE ;
  
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
