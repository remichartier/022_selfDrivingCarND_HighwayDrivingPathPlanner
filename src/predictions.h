/* predictions.h interface to predictions.cpp
 * History :
 * v001 : first version
 */

#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <vector>
//#include <math.h>
//#include "fsm.h"

using std::vector;

void predictions_get(double car_s, double ref_vel,
                    vector<vector<double>> sensor_fusion,
                   double &car_s_predict,
                   vector<double> &predictions);

#endif  // PREDICTIONS_H