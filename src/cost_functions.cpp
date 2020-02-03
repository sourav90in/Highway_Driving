/*
 * cost_functions.cpp
 * This file defines all the required cost-functions needed to evaluate each candidate Trajectory
 * amongst the list of potential trajectories.
 *
 *  Created on: Feb 8, 2020
 *      Author: sourav
 */

#include "helpers.h"
#include <vector>
#include <cmath>
#include "cost_functions.h"
#include <fstream>

using namespace std;

double collision_cost(vector<vector<double>> traj, vector<VehData> predictions)
{

  /* The Predictions array has 75 points for each of the vehicles within field of view
   *
   */
  int num_vehs_in_preds = predictions.size();
  int veh_ctr = 0;
  double closest_dist = 999999.0;

  while( veh_ctr < num_vehs_in_preds )
  {
    for(int i=0; i < NUM_POINTS_FOR_TRAJ; i++)
    {
      /* For each time-point of Ego's trajectory, compare it to the trajectory of the
       * predicted position of the non-Ego vehicle and calculate the closest distances
       */
      double ego_d = traj[0][i];
      double ego_s = traj[1][i];

      double car_d = predictions[veh_ctr].dist_d;
      double car_s = fmod(predictions[veh_ctr].dist_s + predictions[veh_ctr].vel_s*i*0.02, MAX_S);

      double dist = sqrt(pow(getSdistance(ego_s,car_s,!predictions[veh_ctr].is_veh_ahead),2) + pow((ego_d-car_d),2));
      if(dist < closest_dist)
        closest_dist = dist;
    }
    veh_ctr++;
  }

  if(closest_dist < 2*VEHICLE_RADIUS)
  {
    return 1.0;
  }

  return 0.0;

}

double total_acceleration_cost(double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list)
{
  double total_acc= 0.0;
  double avg_accl = 0.0;
  double time_deltas  = T/NUM_POINTS_FOR_TRAJ;
  double time_inc = 0.0;
  for(int i=0; i < NUM_POINTS_FOR_TRAJ; i++)
  {
    /* a = 2a2 + 6a3t + 12a4t2 + 20a5t3 */
    double inst_accl = 2.0*s_coeffs_list[2] + 6.0*s_coeffs_list[3]*time_inc + 12.0*s_coeffs_list[4]*time_inc*time_inc +
                       20.0*s_coeffs_list[5]*time_inc*time_inc*time_inc;
    total_acc += inst_accl;
    time_inc += time_deltas;
  }
  avg_accl = total_acc/T;
  return logistic(avg_accl/EXPECTED_ACC_IN_ONE_SEC);

}

double max_accleration_cost(double T,vector<double> s_coeffs_list, vector<double> d_coeffs_list)
{
  double time_deltas  = T/NUM_POINTS_FOR_TRAJ;
  double time_inc = 0.0;
  for(int i=0; i < NUM_POINTS_FOR_TRAJ; i++)
  {
    double inst_accl = 2.0*s_coeffs_list[2] + 6.0*s_coeffs_list[3]*time_inc + 12.0*s_coeffs_list[4]*time_inc*time_inc +
                           20.0*s_coeffs_list[5]*time_inc*time_inc*time_inc;
    if( fabs(inst_accl) > MAX_ACCELERATION)
      return 1.0;
    time_inc += time_deltas;
  }

  return 0.0;
}

double total_jerk_cost(double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list)
{
  double time_deltas  = T/NUM_POINTS_FOR_TRAJ;
  double time_inc = 0.0;
  double total_jerk = 0.0;
  for(int i=0; i < NUM_POINTS_FOR_TRAJ; i++)
  {
    total_jerk += 6.0*s_coeffs_list[3] + 24.0*s_coeffs_list[4]*time_inc + 60.0*s_coeffs_list[5]*time_inc*time_inc;
    time_inc += time_deltas;
  }
  return logistic(total_jerk/(T*EXPECTED_JERK_IN_ONE_SEC));
}

double max_jerk_cost(double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list)
{
  double time_deltas  = T/NUM_POINTS_FOR_TRAJ;
  double time_inc = 0.0;
  for(int i=0; i < NUM_POINTS_FOR_TRAJ; i++)
  {

    double inst_jerk = 6.0*s_coeffs_list[3] + 24.0*s_coeffs_list[4]*time_inc + 60.0*s_coeffs_list[5]*time_inc*time_inc;
    if(fabs(inst_jerk) > MAX_JERK)
      return 1.0;
    time_inc += time_deltas;
  }
  return 0.0;
}

/*     return logistic(2*float(targ_v - avg_v) / avg_v) */
double total_velocity_cost(double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list,vector<vector<double>> traj)
{
 /* Returns a higher cost for a trajectory whose average velocity for time T
  * is much lesser than the MAx Velocity allowed
  */
  double avg_vel = 0.0;
  avg_vel = (traj[1][NUM_POINTS_FOR_TRAJ-1] - traj[1][0])/T;
  return logistic((2.0*(TARGET_SPEED-avg_vel)/avg_vel));
}

//Ensures preference of LCL over PLCL in PLCL state as well as LCR over PLCR in PLCR state
double free_lane_cost(double T,vector<vector<double>> traj, vector<VehData> predictions)
{
  double d_final_traj = traj[0][NUM_POINTS_FOR_TRAJ-1];
  double d_initial_traj = traj[0][0];
  int final_lane = getLane(d_final_traj);
  int initial_lane = getLane(d_initial_traj);
  bool is_lane_change_intended = false;

  //Try to boost this trajectory(reduce cost) if the final-lane has free space
  if(initial_lane != final_lane)
  {
    //If no vehicles within Field of View in the final-lane, boost it
    for(int i=0; i < predictions.size(); i++)
    {
      if( (getLane(predictions[i].dist_d) == final_lane))
      {
        double s_veh_future = fmod(predictions[i].dist_s + predictions[i].vel_s, MAX_S);
        double s_dist_veh = getSdistance(traj[1][NUM_POINTS_FOR_TRAJ-1],s_veh_future,!predictions[i].is_veh_ahead);
        if( fabs(s_dist_veh) < LANE_FIELD_OF_VIEW_BOOST )
        {
          return 0.0;
        }
      }
    }
    //Favour it in this case since there is no vehicle in FOV of this traj in final-lane
    return -40000.0;
  }

  //No favour in case initial and final lanes are the same.
  return 0.0;


}

double total_trajectory_cost(double T,vector<vector<double>> traj, vector<VehData> predictions, vector<double> s_coeffs_list, vector<double> d_coeffs_list )
{
  double total_cost = 0.0;
  double coll_cost = 0.0;
  double total_accl_cost = 0.0;
  double max_accl_cost = 0.0;
  double total_jk_cost = 0.0;
  double max_jk_cost = 0.0;
  double total_vel_cost = 0.0;
  double free_l_cost = 0.0;

  ofstream outfile;
  if(LOGGING_ENABLED)
  {
      outfile.open("logs.txt", ios::out| ios::app);
  }
  coll_cost = COLLISION_WEIGHT*collision_cost(traj,predictions);
  total_cost += coll_cost;
  if(LOGGING_ENABLED)
  {
     outfile<<"Collision cost was"<<coll_cost<<endl;
  }

  total_accl_cost = TOTAL_ACCL_WEIGHT*total_acceleration_cost(T,s_coeffs_list,d_coeffs_list );
  total_cost += total_accl_cost;
  if(LOGGING_ENABLED)
  {
     outfile<<"Total Accl cost was"<<total_accl_cost<<endl;
  }

  max_accl_cost = MAX_ACCL_WEIGHT*max_accleration_cost(T,s_coeffs_list, d_coeffs_list);
  total_cost += max_accl_cost;
  if(LOGGING_ENABLED)
  {
     outfile<<"Max Accl cost was"<<max_accl_cost<<endl;
  }

  total_jk_cost = TOTAL_JERK_WEIGHT*total_jerk_cost(T,s_coeffs_list, d_coeffs_list);
  total_cost += total_jk_cost;
  if(LOGGING_ENABLED)
  {
     outfile<<"Total Jerk cost was"<<total_jk_cost<<endl;
  }


  max_jk_cost = MAX_JERK_WEIGHT*max_jerk_cost(T,s_coeffs_list, d_coeffs_list);
  total_cost += max_jk_cost;
  if(LOGGING_ENABLED)
  {
     outfile<<"Max Jerk cost was"<<max_jk_cost<<endl;
  }

  free_l_cost = free_lane_cost(T,traj, predictions);
  total_cost += free_l_cost;
  if(LOGGING_ENABLED)
  {
     outfile<<"Free lane boosting was"<<free_l_cost<<endl;
  }

  total_vel_cost = TOTAL_VEL_WEIGHT*total_velocity_cost(T,s_coeffs_list,d_coeffs_list,traj);
  total_cost += total_vel_cost;
  if(LOGGING_ENABLED)
  {
     outfile<<"Total Vel cost was"<<total_vel_cost<<endl;
     outfile.close();
  }


  return total_cost;

}



