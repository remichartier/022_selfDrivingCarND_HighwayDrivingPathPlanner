/* cost.h to gather cost functions
 * History :
 * v01 : first version
 */

#ifndef COST_H
#define COST_H

#include <vector>
//#include <math.h>
//#include "fsm.h"

using std::vector;

#define COST_DIST_MAX	6945.554

double cost_car_distance_ahead(double car_s, vector<vector<double>> sensor_fusion, 
                               int lane, int dist_min);

#endif  // COST_H