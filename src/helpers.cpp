/*
 * helpers.cpp
 * This file defines all the utility functions needed to be used across all files
 *  Created on: Feb 7, 2020
 *      Author: sourav
 */
#include "helpers.h"
#include <cstdio>
#include <cmath>
#include <iostream>
#include<fstream>

using namespace std;

void VehData::SetData(double d_coord,double s_coord,double vel, double accl )
{
  dist_d = d_coord;
  dist_s = s_coord;
  vel_s = vel;
  accl_s = accl;
}

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//Convert miles per hour to metres per second
double mphtoms(double x) { return x/2.237 ; }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}



/* Convert Car's d frenet coordinate to lane numbers such that:
 * 0 to 4 is lane 0
 * 4 to 8 is lane 1
 * 8 to 12 is lane 2
 */
int getLane(double d)
{
  if( ( d >= 0.0 ) && ( d <= 4.0 ) )
    return 0;
  else if( ( d > 4.0 ) && ( d <= 8.0 ) )
    return 1;
  else if( ( d > 8.0 ) && ( d <= 12.0 ) )
    return 2;

  return -1;
}

double gecenterD(int lane)
{
  if(lane == 0)
    return 2.0;
  else if(lane == 1)
    return 6.0;
  else
    return 10.0;
}

/* Returns the absolute distance between two points
 * whose S-Coordinates could have potentially wrapped around
 */
double getSdistance(double ego_s, double car_s, bool is_car_behind)
{
  double dist = 0.0;
  if(is_car_behind == true)
  {
    /* Ego's S is greater than 0 due to wrap and vehicle behind it is having
     * S lesser then S_MAX.
     * TODO::: Possible corner case is that in some prediction data, while interpolating
     * forward 50 points for a vehicle, any vehicle that was originally behind Ego shows up as A VehData
     * object in the list of predictions with S greater than EGo's S. Filtering them out is needed. Also valid for vehicle
     * in front.
     */
    if(car_s > ego_s )
    {
      /* Wrap around has occurred */
      dist = fabs(car_s - MAX_S) + fabs(MAX_S - ego_s);
    }
    else
    {
      dist = fabs( ego_s - car_s );
    }
  }
  else
  {
    if( car_s < ego_s )
    {
      dist = fabs(ego_s - MAX_S) + car_s;
    }
    else
    {
      dist = fabs( ego_s - car_s);
    }
  }
  return dist;
}

/* Reference: geeksforgeeks*/
double findMod(double a, double b)
{
    double mod;
    // Handling negative values
    if (a < 0)
        mod = -a;
    else
        mod =  a;
    if (b < 0)
        b = -b;

    // Finding mod by repeated subtraction

    while (mod >= b)
        mod = mod - b;

    // Sign of result typically depends
    // on sign of a.
    if (a < 0)
        return -mod;

    return mod;
}

/*def logistic(x):
    """
    A function that returns a value between 0 and 1 for x in the
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
    """
    return 2.0 / (1 + exp(-x)) - 1.0
*/
double logistic(double x)
{
  return 2.0/(1.0 + exp(-x)) - 1.0;
}

void print_to_console(string s)
{
  if(LOGGING_ENABLED)
  {
    cout<<s<<endl;
  }

  if(LOGGING_ENABLED)
  {
    //Print to a local file
    ofstream outfile;
    outfile.open("logs.txt", ios::out| ios::app);
    outfile<<s<<endl;
    outfile.close();
  }

}


