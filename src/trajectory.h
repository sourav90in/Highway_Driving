/*
 * trajectory.h
 *
 *  Created on: Feb 4, 2020
 *      Author: sourav
 */

#ifndef HIGHWAY_DRIVING_SRC_TRAJECTORY_H_
#define HIGHWAY_DRIVING_SRC_TRAJECTORY_H_

#include <vector>
#include <math.h>

using namespace std;

typedef struct _TrajPoint
{
  double p;
  double p_dot;
  double p_dot_dot;
  void InitTrajPoint(double p2, double p2_dot,double p2_dot_dot)
  {
    p = p2;
    p_dot = p2_dot;
    p_dot_dot = p2_dot_dot;
  }
}TrajPoint;

class Trajectory
{
 private:
  //Generates the JMT Coeffs for s as well as d data.
  vector<double> generate_JMT_coeffs(vector<double> &start, vector<double> &end, double T, int start_idx);

 public:
  Trajectory() { };
  ~Trajectory() { }
  /* Future TODO, the below members prev_path_s/d should ideally be private variables
   * accessible to only members. Needs future cleanup.
   */
  //Holds the previous path S points(S,s-dot,s-double-dot) for the Ego vehicle(50 points)
  vector<TrajPoint> prev_path_s;
  //Holds the previous path D points(D,D-dot,D-double-dot) for the Ego vehicle(50 points)
  vector<TrajPoint> prev_path_d;

  vector<double> getPrevTraj_params( int i );
  void InitTrajectory(double car_d, double car_s,double s_vel, double s_accl);
  double getPrevTrajS_dotdot( int i);
  double getPrevTrajS( int i);
  double getPrevTrajS_dot( int i);
  vector<vector<double>> GetCurrentSDTrajectory();

  //Generates the Traj Points for the S and D trajectories for the entire time-horizon
  vector<vector<double>> generate_Horizon_Trajectory(vector<double> traj_init_state, vector<double> traj_final_state, double T, vector<double>&s_coeffs_jmt,
                                                                 vector<double>&d_coeffs_jmt);
  void SetPrevTrajectory(vector<vector<double>> gen_traj, double T, vector<double> s_coeffs_list, vector<double> d_coeffs_list);
};




#endif /* HIGHWAY_DRIVING_SRC_TRAJECTORY_H_ */
