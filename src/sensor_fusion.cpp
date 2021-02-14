/* sensor_fusion.cpp to gather functions related to sensor_fusion
 * History :
 * v01 : first version, get_index_speed_mph()
 * TAGGED v1.0 on Github : Working version, Change Lane if too close to car ahead
 *        Test : Enable Lane change anytime Cost computation judge necessary
 * TAGGED v1.1 on Github (Working version, changing lanes depending of costs
 *        Cleanup of code 
 */
#include <math.h> // for sqrt()
#include "sensor_fusion.h"
#include "constants.h"

double sf_get_speed_meterps(vector<vector<double>> sensor_fusion, int index)
/* 
 * Inputs : 
 *			- <vector<vector<double>> sensor_fusion
 *			- int index
 * Return : 
 *			- double speed, in mph
 */
{
  double vx = sensor_fusion[index][3];
  double vy = sensor_fusion[index][4];
  double speed = sqrt(vx*vx + vy*vy); // Note : this is in m/s
  return speed;
} // end function

double sf_get_speed_milesph(vector<vector<double>> sensor_fusion, int index)
/* 
 * Inputs : 
 *			- <vector<vector<double>> sensor_fusion
 *			- int index
 * Return : 
 *			- double speed, in mph
 */
{
  double speed = sf_get_speed_meterps(sensor_fusion, index); // Note : this is in m/s
  speed *= METERPERSECOND2MPH; // Note : converting to MPH
  return speed;
} // end function
