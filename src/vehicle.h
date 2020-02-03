/*
 * vehicle.h
 *
 *  Created on: Feb 2, 2020
 *      Author: sourav
 */

#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_

#include <map>
#include <string>
#include <vector>
#include "trajectory.h"
#include "helpers.h"
#include <iostream>
#include <fstream>

using namespace std;
using std::map;
using std::string;
using std::vector;

class Vehicle {
 private:

  //Stores the Ego vehicle data
  VehData ego_vdata;

  //Stores the non-Ego Vehicle prediction data
  vector<VehData> predictions;

  //Maps the d value to the corresponding lane
  int Map_d_coord_to_lane_val(double d_val);

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1},
                                     {"LCR", 1}, {"PLCR", 1}};

  vector<string> successor_states(Trajectory &traj_current);

  vector<double> generate_trajectory(string state);

  bool get_vehicle_behind( int lane,VehData &rVehicle);

  bool get_vehicle_ahead(int lane,VehData &rVehicle);

  vector<double> get_kinematics(int lane);

  vector<double> constant_speed_trajectory();

  vector<double> keep_lane_trajectory(void );

  vector<double> lane_change_trajectory(string state);

  vector<double> prep_lane_change_trajectory(string state );

  //Used for generating Predictions from Sensor Fusion Data
  void generate_predictions(int horizon, vector<vector<double>> sensor_fusion_data,int prev_size);

 public:
  /* Holds the Size of the path reported back by simulator from the previous trajectory */
  int previous_path_sz;
  /* Holds the Final point index of the Latest generated trajectory that was sent to the simulator
   * after adding all the previously reported back trajectory points.
   */
  int final_pt_sent = 0;
  //Current state of the Ego vehicle
  string ego_state;
  /* Indicates if a LCL or LCR is in progress i.e. its marked as false
   * only when the target_lane and the current-lane match. Used for ensuring
   * that KL substate is only considered after a lane change finishes to the
   * desired target-lane
   */
  bool is_lane_change_in_progress;
  int target_lane;
  // Constructors
  Vehicle();
  // Destructor
  ~Vehicle() {}

  void SetEgoVehData(double d_coord,double s_coord,double vel, double accl)
  {
    ego_vdata.SetData(d_coord,s_coord,vel,accl);
    if(LOGGING_ENABLED)
    {
      ofstream outfile;
      outfile.open("logs.txt", ios::out| ios::app);
      outfile<<"Speed at start is"<<vel<<endl;
      outfile<<"S-Distance at start is"<<s_coord<<endl;
      outfile<<"D-Distance at is"<<d_coord<<endl;
      outfile.close();
    }
  }

  vector<VehData> get_other_vehicle_predictions() { return predictions; }

  // Vehicle functions
  void Exec_Behaviour_Planner_Actions(Trajectory& traj_current, vector<vector<double>> sensor_fusion_data);

};



#endif /* SRC_VEHICLE_H_ */
