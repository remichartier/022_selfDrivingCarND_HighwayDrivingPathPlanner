/* cost.h to gather cost functions
 * History :
 * v01 : first version, cost_car_distance_ahead(),
 *       cost_car_speed_ahead()
 */

#ifndef COST_H
#define COST_H

#include <vector>
//#include <math.h>
//#include "fsm.h"

using std::vector;

#define COST_DIST_MAX	6945.554
#define NONE			-1
#define MAX_SPEED_MPH 		49.5


double cost_car_distance(double car_s, vector<vector<double>> sensor_fusion, 
                               int dist_min, int next_car_index);

double cost_car_speed_ahead(double ref_vel,  vector<vector<double>> sensor_fusion, 
                            int next_car_index);

#endif  // COST_H