/* sensor_fusion.h to gather sensor_fusion related functions
 * History :
 * v01 : first version,
 * TAGGED v1.0 on Github : Working version, Change Lane if too close to car ahead
 *        Test : Enable Lane change anytime Cost computation judge necessary
 * TAGGED v1.1 on Github (Working version, changing lanes depending of costs
 *        Cleanup of code 
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>

using std::vector;

double sf_get_speed_meterps(vector<vector<double>> sensor_fusion, int index);

double sf_get_speed_milesph(vector<vector<double>> sensor_fusion, int index);

#endif  // SENSOR_FUSION_H