/* cost.h to gather cost functions
 * History :
 * v01 : first version, cost_car_distance_ahead(),
 *       cost_car_speed_ahead()
 *       add conversion meter per second to mph
 * v02 : add cost_car_cutting_lane_ahead(), 
 *       add cost_colliding_car_ahead(), cost_collided_rear_car(),
 *       cost_car_buffer()
 */

#ifndef COST_H
#define COST_H

#include <vector>
//#include <math.h>
//#include "fsm.h"
#include "behavior_planner.h"

using std::vector;

#define COST_DIST_MAX	6945.554

double cost_colliding_car_ahead(int next_car_index, vector<double> predictions,
                               double car_s_predict);

double cost_collided_rear_car(int index_car_behind, vector<double> predictions, 
                              double car_s_predict);

double cost_car_buffer(double car_s_predict, vector<double> predictions, double dist_min,
                              int index_car);
  
// Not used anymore I think

double cost_car_distance(double car_s, vector<vector<double>> sensor_fusion, 
                               int dist_min, int next_car_index);

double cost_car_speed_ahead(double ref_vel,  vector<vector<double>> sensor_fusion, 
                            int next_car_index);

double cost_car_cutting_lane_ahead(vector<vector<double>> sensor_fusion, int lane, 
                                   fsm_state action, int index_car_ahead);


#endif  // COST_H