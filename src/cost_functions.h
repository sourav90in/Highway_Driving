/*
 * cost_functions.h
 *
 *  Created on: Feb 9, 2020
 *      Author: sourav
 */

#ifndef HIGHWAY_DRIVING_SRC_COST_FUNCTIONS_H_
#define HIGHWAY_DRIVING_SRC_COST_FUNCTIONS_H_

#include <vector>
#include "helpers.h"
#include "trajectory.h"
#include <cmath>

using namespace std;

//#define COLLISION_WEIGHT 50000.0
#define COLLISION_WEIGHT pow(10.0,6)/2.0
#define TOTAL_ACCL_WEIGHT 5000.0
#define MAX_ACCL_WEIGHT 10000.0
#define TOTAL_JERK_WEIGHT 15000.0
#define MAX_JERK_WEIGHT 20000.0
#define TOTAL_VEL_WEIGHT pow(10.0,6)


double collision_cost(vector<vector<double>> traj, vector<VehData> predictions);

double total_acceleration_cost(double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list);

double max_accleration_cost(double T,vector<double> s_coeffs_list, vector<double> d_coeffs_list);

double total_jerk_cost(double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list);

double max_jerk_cost(double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list);

/*     return logistic(2*float(targ_v - avg_v) / avg_v) */
double total_velocity_cost(double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list,vector<vector<double>> traj);

double free_lane_cost(double T,vector<vector<double>> traj, vector<VehData> predictions);

double total_trajectory_cost(double T,vector<vector<double>> traj, vector<VehData> predictions, vector<double> s_coeffs_list, vector<double> d_coeffs_list );


#endif /* HIGHWAY_DRIVING_SRC_COST_FUNCTIONS_H_ */
