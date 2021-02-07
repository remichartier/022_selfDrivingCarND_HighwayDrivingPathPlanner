/* cost.h to gather cost functions
 * History :
 * v01 : first version, cost_car_distance_ahead(),
 *       cost_car_speed_ahead()
 *       add conversion meter per second to mph
 * v02 : add cost_car_cutting_lane_ahead()
 */

#ifndef COST_H
#define COST_H

#include <vector>
//#include <math.h>
//#include "fsm.h"
#include "behavior_planner.h"

using std::vector;

#define COST_DIST_MAX	6945.554
#define NONE			-1
#define MAX_SPEED_MPH 		49.5
/*
1 meter = 0.000621371 miles.
Each second :  0.000621371 miles
Each hour = 1s 60 * 60 → 60 *60 *  0.000621371 miles = 2.2369356 miles
→ 1m/s = 2.2369356 mph
*/
#define METERPERSECOND2MPH	2.2369356


double cost_car_distance(double car_s, vector<vector<double>> sensor_fusion, 
                               int dist_min, int next_car_index);

double cost_car_speed_ahead(double ref_vel,  vector<vector<double>> sensor_fusion, 
                            int next_car_index);

double cost_car_cutting_lane_ahead(vector<vector<double>> sensor_fusion, int lane, 
                                   fsm_state action, int index_car_ahead);


#endif  // COST_H