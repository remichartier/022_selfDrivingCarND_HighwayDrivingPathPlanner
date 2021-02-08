/* prediction.cpp to manage prediction of sensor_fusion data
 * History :
 * v001 : first version with fsm_transition_function()
 */

#include "predictions.h"

void prediction_get(double car_s, double ref_vel,
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

  if(ref_vel == 0)
  {
    std::cout << "prediction_get() ERROR : ref_vel = 0 can not divide by 0" << std::endl;
    std::cout << "This causes program TERMINATION - exit(EXIT_FAILURE)" << std::endl;
    exit(EXIT_FAILURE);
  }
  double t_hour = 30.0 * METER2MILE / ref_vel;
  for(int i=0; i<sensor_fusion.size();i++)
  {
    double s = sensor_fusion[i][5];
    double speed = sensor_fusion[i][];
    
  } // end for
  
  
} // end function

// constructor
Prediction::prediction(int id, double s_value)
{
  
}