/* predictions.h interface to predictions.cpp
 * History :
 * v001 : first version
 * TAGGED v1.0 on Github : Working version, Change Lane if too close to car ahead
 *        Test : Enable Lane change anytime Cost computation judge necessary
 * TAGGED v1.1 on Github (Working version, changing lanes depending of costs
 *        Cleanup of code 
 */

#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <vector>

using std::vector;

void predictions_get(double car_s, double ref_vel,
                    vector<vector<double>> sensor_fusion,
                   double &car_s_predict,
                   vector<double> &predictions);

#endif  // PREDICTIONS_H