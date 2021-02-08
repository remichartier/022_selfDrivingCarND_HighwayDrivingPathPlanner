/* sensor_fusion.h to gather sensor_fusion related functions
 * History :
 * v01 : first version, 
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>
//#include <math.h>
//#include "fsm.h"

using std::vector;

double get_index_speed_meterps(vector<vector<double>> sensor_fusion, int index);

double get_index_speed_milesph(vector<vector<double>> sensor_fusion, int index);

#endif  // SENSOR_FUSION_H