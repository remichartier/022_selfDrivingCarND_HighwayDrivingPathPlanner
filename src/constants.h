/* constants.h to gather all constants common to all .h/.cpp files
 * History :
 * v01 : first version, 
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

//#include <vector>
//#include <math.h>
//#include "fsm.h"

//using std::vector;

#define MAX_SPEED_MPH 		49.5
#define NONE				-1

/*
1 meter = 0.000621371 miles.
Each second :  0.000621371 miles
Each hour = 1s 60 * 60 → 60 *60 *  0.000621371 miles = 2.2369356 miles
→ 1m/s = 2.2369356 mph
*/
#define METERPERSECOND2MPH	2.2369356

// 1 meter = 0.000621371 miles.
#define METER2MILE			0.000621371

#define MAX_ACCEL 				0.224
#define LANE_MAX				2
#define LANE_MIN				0
#define SAFE_DISTANCE_M			50
#define SAFE_DISTANCE_BEHIND_M		50


#endif  // CONSTANTS