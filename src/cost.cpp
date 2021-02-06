/* cost.cpp to gather cost functions
 * History :
 * v01 : first version, cost_car_distance_ahead(),
 *       cost_car_speed_ahead()
 */

#include <iostream> // for cout, endl
#include <stdlib.h> // for EXIT_FAILURE
#include <math.h> // for sqrt()
#include "cost.h"

// Distance car ahead,
// want cost function return : 1 if dist < dist_min,--> dist_min/dist
// ie with be 1 if dist_min, and decreast propertionnaly if > dist_min ...
double cost_car_distance_ahead(double car_s, vector<vector<double>> sensor_fusion, 
                               int lane, int dist_min, int &next_car_index)
/* 
 * Inputs : 
 *			- double car_s
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane
 *			- int dist_min (distance to check), if>0 : ahead, if<0,behind 
 *			- int next_car_index, by reference, reused for future cost functions to avoid doing 
 *				the same search again (if no car ahead --> NONE(-1))
 * Outputs : 
 *			- int next_car_index
 * Return : 
 *			- double cost
 */
{
  double s = COST_DIST_MAX;
  next_car_index = NONE ;
  // need to find closest car ahead in the same lane !

  // find ref_v to use
  for(int i=0; i < sensor_fusion.size(); i++)
  {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if( d < (2+4*lane+2) && d > (2+4*lane-2) )
    {
      double check_car_s = sensor_fusion[i][5];      
      
        if(check_car_s > car_s)		// if car ahead in the same lane
        {
          // if this car closer than previous s --> s
          s = (check_car_s<s)? check_car_s : s;
          next_car_index = (check_car_s<s)? i : next_car_index;
        } // end if car ahead 
    } // end if car ahead in same lane
  } // end for each car
  
  // if car ahead exists on same lane :
  if(next_car_index != NONE)
  {
    // here s contains the coordinate of the next car ahead of car_s in the same lane
    double dist = s - car_s;
    if(dist == 0)
    {
      std::cout << "cost_car_distance_ahead() ERROR : dist = 0 can not divide by 0" << std::endl;
      std::cout << "This causes program TERMINATION - exit(EXIT_FAILURE)" << std::endl;
      exit(EXIT_FAILURE);
    }

    return(dist_min / dist);
  } 
  else // case no car ahead on the same lane 
  {
    return -1;
  } // end if car ahead on the same lane
} // end function


// Cost Speed car ahead, 
// Compare our car ref_vel with next car ahead speed,
// speed_car_ahead - ref_vel / MAX_SPEED_MPH
double cost_car_speed_ahead(double ref_vel,  vector<vector<double>> sensor_fusion, 
                            int next_car_index)
/* 
 * Inputs : 
 *			- double ref_vel : vehicule speed
 *			- <vector<vector<double>> sensor_fusion
 *			- int next_car_index, reused to avoid search in sensor_fusion 
 *				(if no car ahead --> NONE(-1))
 * Return : 
 *			- double cost
 */
{
  if(next_car_index == NONE) return -1;
  
  // retrieve next car speed
  double vx = sensor_fusion[next_car_index][3];
  double vy = sensor_fusion[next_car_index][4];
  double speed_ahead = sqrt(vx*vx + vy*vy);
  
  return ((ref_vel - speed_ahead) / MAX_SPEED_MPH);
  
} // end function