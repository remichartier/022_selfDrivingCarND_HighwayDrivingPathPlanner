/* cost.cpp to gather cost functions
 * History :
 * v01 : first version
 */

#include <iostream> // for cout, endl
#include <stdlib.h> // for EXIT_FAILURE
#include "cost.h"

// Distance car ahead,
// want cost function return : 1 if dist < dist_min,--> dist_min/dist
// ie with be 1 if dist_min, and decreast propertionnaly if > dist_min ...
double cost_car_distance_ahead(double car_s, vector<vector<double>> sensor_fusion, 
                               int lane, int dist_min)
/* 
 * Inputs : 
 *			- double car_s
 *			- <vector<vector<double>> sensor_fusion
 *			- int lane by reference
 *			- int dist_min (distance to check), if>0 : ahead, if<0,behind 
 * Return : 
 *			- double cost
 */
{
  double s = COST_DIST_MAX;
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
        } // end if car ahead 
    } // end if car ahead in same lane
  } // end for each car
  
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