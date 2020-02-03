/*
 * map.h
 *
 *  Created on: Feb 2, 2020
 *      Author: sourav
 */

#ifndef SRC_MAP_H_
#define SRC_MAP_H_

#include <vector>
#include <iostream>
#include "spline.h"
using namespace std;

class Map_data
{
 private:
  tk::spline spline_x_s;
  tk::spline spline_y_s;
  tk::spline spline_dx_s;
  tk::spline spline_dy_s;
 public:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  void buildAccurateMap(vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_dx, vector<double> &maps_dy, double max_s);
  vector<vector<double>> getXYfromSD_withSpline(vector<vector<double>> sd_traj);
};

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y);


#endif /* SRC_MAP_H_ */
