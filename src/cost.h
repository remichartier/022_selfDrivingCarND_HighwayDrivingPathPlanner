/* cost.h to gather cost functions
 * History :
 * v01 : first version, cost_car_distance_ahead(),
 *       cost_car_speed_ahead()
 *       add conversion meter per second to mph
 * v02 : add cost_car_cutting_lane_ahead(), 
 *       add cost_colliding_car_ahead(), cost_collided_rear_car(),
 *       cost_car_buffer()
 * TAGGED v1.0 on Github : Working version, Change Lane if too close to car ahead
 *        Test : Enable Lane change anytime Cost computation judge necessary
 * TAGGED v1.1 on Github (Working version, changing lanes depending of costs
 *        Cleanup of code 
 */

#ifndef COST_H
#define COST_H

#include <vector>

using std::vector;

#define COST_DIST_MAX	6945.554

double cost_colliding_car_ahead(int next_car_index, vector<double> predictions,
                               double car_s_predict);

double cost_collided_rear_car(int index_car_behind, vector<double> predictions, 
                              double car_s_predict);

double cost_car_buffer(double car_s_predict, vector<double> predictions, double dist_min,
                              int index_car);
  
double cost_car_speed_ahead(double ref_vel,  vector<vector<double>> sensor_fusion, 
                            int next_car_index);

#endif  // COST_H