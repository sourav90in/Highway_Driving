/*
 * trajectory.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: sourav
 */

#include "helpers.h"
#include "trajectory.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


void Trajectory::InitTrajectory(double car_d, double car_s,double s_vel, double s_accl)
{

  /*Initialize the Trajectory points(50) of previous path of S/D coordinates
   * with car_s and car_d
   */
  prev_path_s.clear();
  prev_path_d.clear();
  for(int i=0; i < NUM_POINTS_FOR_TRAJ; i++)
  {
    TrajPoint s;
    TrajPoint d;
    s.InitTrajPoint(car_s,s_vel,0.0);
    d.InitTrajPoint(car_d,0.0,0.0);
    prev_path_s.push_back(s);
    prev_path_d.push_back(d);
  }
}

void Trajectory::SetPrevTrajectory(vector<vector<double>> gen_traj, double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list)
{
  //Clear the previous values of Prev Path D and S
  prev_path_d.clear();
  prev_path_s.clear();
  double time_deltas  = T/NUM_POINTS_FOR_TRAJ;
  double time_inc = 0.0;
  ofstream outfile;
  if(LOGGING_ENABLED)
  {
    outfile.open("logs.txt", ios::out| ios::app);
    outfile<<">>------Logging all points of chosen trajectory <<----"<<endl;
    for(int i=0; i < 6; i++)
    {
      outfile<<i<<"th "<<"S-Coeff is: "<<s_coeffs_list[i]<<" and D-Coeff is: "<<d_coeffs_list[i]<<endl;
    }
    outfile.close();
  }

  for(int i=0; i < NUM_POINTS_FOR_TRAJ; i++)
  {
    TrajPoint s;
    TrajPoint d;
    double car_s = gen_traj[1][i];
    double car_s_dot = s_coeffs_list[1] + 2.0*s_coeffs_list[2]*time_inc + 3.0*s_coeffs_list[3]*time_inc*time_inc +
                       4.0*s_coeffs_list[4]*pow(time_inc,3)  + 5.0*s_coeffs_list[5]*pow(time_inc,4);
    double car_s_dot_dot = 0.0;
    s.InitTrajPoint(car_s,car_s_dot,car_s_dot_dot);
    prev_path_s.push_back(s);

    double car_d = gen_traj[0][i];
    double car_d_dot = d_coeffs_list[1] + 2.0*d_coeffs_list[2]*time_inc + 3.0*d_coeffs_list[3]*time_inc*time_inc +
                       4.0*d_coeffs_list[4]*pow(time_inc,3)  + 5.0*d_coeffs_list[5]*pow(time_inc,4);
    double car_d_dot_dot = 2.0*d_coeffs_list[2] + 6.0*d_coeffs_list[3]*time_inc + 12.0*d_coeffs_list[4]*time_inc*time_inc +
                          20.0*d_coeffs_list[5]*time_inc*time_inc*time_inc;

    d.InitTrajPoint(car_d,car_d_dot, car_d_dot_dot );
    prev_path_d.push_back(d);

    time_inc += time_deltas;

  }

  if(LOGGING_ENABLED)
  {
    outfile.open("logs.txt", ios::out| ios::app);
    outfile<<">>------Logging all points of chosen trajectory <<----"<<endl;
    for(int i=0; i < NUM_POINTS_FOR_TRAJ; i++)
    {
      outfile<<"S-Distance chosen"<<prev_path_s[i].p<<endl;
      outfile<<"S-Vel chosen"<<prev_path_s[i].p_dot<<endl;
      outfile<<"S-Accl chosen"<<prev_path_s[i].p_dot_dot<<endl;
      outfile<<"D-Distance chosen"<<prev_path_d[i].p<<endl;
      outfile<<"D-Vel chosen"<<prev_path_d[i].p_dot<<endl;
      outfile<<"D-Accl chosen"<<prev_path_d[i].p_dot_dot<<endl;

    }
    outfile<<">>>------End of chosen points<<-------"<<endl;
    outfile.close();
  }
}

double Trajectory::getPrevTrajS_dotdot( int i)
{
  return prev_path_s[i].p_dot_dot;
}

double Trajectory::getPrevTrajS( int i)
{
  return prev_path_s[i].p;
}

double Trajectory::getPrevTrajS_dot( int i)
{
  return prev_path_s[i].p_dot;
}



vector<double> Trajectory::getPrevTraj_params( int i )
{
  vector<double> params;
  params.push_back(prev_path_d[i].p);
  params.push_back(prev_path_d[i].p_dot);
  params.push_back(prev_path_d[i].p_dot_dot);
  params.push_back(prev_path_s[i].p);
  params.push_back(prev_path_s[i].p_dot);
  params.push_back(0.0);
  return params;
}

/* Returns the 50 points saved for S and D trajectories in SetPrevTrajectory */
vector<vector<double>> Trajectory::GetCurrentSDTrajectory()
{

  vector<vector<double>> sd_traj;
  vector<double> s_traj;
  vector<double> d_traj;
  for(int i=0; i < NUM_POINTS_FOR_TRAJ;i++)
  {
    s_traj.push_back(prev_path_s[i].p);
    d_traj.push_back(prev_path_d[i].p);
  }
  sd_traj.push_back(s_traj);
  sd_traj.push_back(d_traj);
  return sd_traj;
}

vector<vector<double>> Trajectory::generate_Horizon_Trajectory(vector<double> traj_init_state, vector<double> traj_final_state, double T, vector<double> &s_coeffs_jmt,
                                                               vector<double> &d_coeffs_jmt)
{
  vector<vector<double>> res_traj;
  vector<double> s_traj;
  vector<double> d_traj;
  ofstream outfile;
  vector<double> d_coeffs = this->generate_JMT_coeffs(traj_init_state, traj_final_state, T, 0);
  vector<double> s_coeffs = this->generate_JMT_coeffs(traj_init_state,traj_final_state, T, 3);
  double time_ctr = 0.0;
  double time_inc = T/NUM_POINTS_FOR_TRAJ;

  for(int i=0; i < NUM_POINTS_FOR_TRAJ; i++)
  {
    double s_val = s_coeffs[0] + s_coeffs[1]*time_ctr + s_coeffs[2]*pow(time_ctr,2) + s_coeffs[3]*pow(time_ctr,3) + s_coeffs[4]*pow(time_ctr,4) + s_coeffs[5]*pow(time_ctr,5);
    s_val = fmod(s_val,MAX_S);
    double d_val = d_coeffs[0] + d_coeffs[1]*time_ctr + d_coeffs[2]*pow(time_ctr,2) + d_coeffs[3]*pow(time_ctr,3) + d_coeffs[4]*pow(time_ctr,4) + d_coeffs[5]*pow(time_ctr,5);

    s_traj.push_back(s_val);
    d_traj.push_back(d_val);
    time_ctr += time_inc;
  }
  res_traj.push_back(d_traj);
  res_traj.push_back(s_traj);

  //Save the generated S and D coeffs
  s_coeffs_jmt =  s_coeffs;
  d_coeffs_jmt = d_coeffs;
  return res_traj;

}

vector<double> Trajectory::generate_JMT_coeffs(vector<double> &start, vector<double> &end, double T, int start_idx)
{
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;

  MatrixXd B = MatrixXd(3,1);
  B << end[start_idx+0]-(start[start_idx+0]+start[start_idx+1]*T+.5*start[start_idx+2]*T*T),
       end[start_idx+1]-(start[start_idx+1]+start[start_idx+2]*T),
       end[start_idx+2]-start[start_idx+2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai*B;

  vector <double> result = {start[start_idx+0], start[start_idx+1], .5*start[start_idx+2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
}
