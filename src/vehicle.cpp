/*
 * vehicle.cpp
 *
 *  Created on: Feb 2, 2020
 *      Author: sourav
 */


#include "vehicle.h"
#include "helpers.h"
#include <iostream>
#include <fstream>
#include "trajectory.h"
#include "cost_functions.h"
#include <algorithm>
#include <cmath>

using namespace std;

Vehicle::Vehicle(void)
{
  //Since Ego vehicle starts in the middle lane(i.e. lane 1)
  ego_vdata.SetData(6.0,0.0,0.0,0.0);
  //Current state of the Ego vehicle
  ego_state = "KL";
  is_lane_change_in_progress = false;
}

void Vehicle::generate_predictions(int horizon, vector<vector<double>> sensor_fusion_data,int prev_size)
{
  /*
   * A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates,
   * car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s,
   * car's s position in frenet coordinates, car's d position in frenet coordinates.
   */
  double car_s = 0.0;
  double car_d = 0.0;
  double car_vx = 0.0;
  double car_vy = 0.0;
  double car_speed = 0.0;
  double s_dist_front_fov = 0.0;
  double s_dist_back_fov = 0.0;
  bool front_wrap = false;
  bool back_wrap = false;
  bool target_vehicle = false;
  bool vehicle_in_front = false;
  ofstream outfile;

  print_to_console("Entered generate_predictions function");
  /* Logic to determine if the current s of the ego vehicle
   * added to the max field of view from it(front or back),
   * causes a wrap-around
   */
  /* Note that only Front Wrap or Back wrap is possible and not both
   * since the FOV is significantly smaller than the MAX_S
   */
  if( ego_vdata.dist_s + LANE_FIELD_OF_VIEW > MAX_S )
  {
    front_wrap = true;
    s_dist_front_fov = (double)(MAX_S - (ego_vdata.dist_s + LANE_FIELD_OF_VIEW )) ;
    print_to_console("Front wrap is true");
  }
  else
  {
    s_dist_front_fov = (double)(ego_vdata.dist_s + LANE_FIELD_OF_VIEW) ;
    print_to_console("Front wrap is false");
  }

  if( ego_vdata.dist_s - LANE_FIELD_OF_VIEW < 0 )
  {
    back_wrap = true;
    s_dist_back_fov = findMod(fabs(ego_vdata.dist_s - LANE_FIELD_OF_VIEW),MAX_S);
    print_to_console("Back wrap is true");
  }
  else
  {
    s_dist_back_fov = (double)(ego_vdata.dist_s - LANE_FIELD_OF_VIEW);
    print_to_console("Back wrap is false");
  }

  /* Add Vehicles and its candidate predictions for the given time-horizon
   * assuming constant velocity for only those vehicles which are in
   * the Field of View of the Ego vehicle.
   */

  if(LOGGING_ENABLED)
  {
    outfile.open("logs.txt", ios::out| ios::app);
    outfile<<"Size of Sensor fusion data received is"<<sensor_fusion_data.size()<<endl;
    outfile<<"Prev point size received is"<<prev_size<<endl;
    outfile.close();
  }

  for(int i=0; i < sensor_fusion_data.size(); i++)
  {

    target_vehicle = false;
    vehicle_in_front = false;
    car_vx = sensor_fusion_data[i][3];
    car_vy = sensor_fusion_data[i][4];
    car_speed = sqrt(car_vx*car_vx + car_vy*car_vy);
    car_s = sensor_fusion_data[i][5]+(prev_size*0.02*car_speed);
    car_d = sensor_fusion_data[i][6];
    /* Sanity checks of lane before processing this data-point
     * to ensure that only lanes 0 or 1 or 2 based cars are processed.
     * Also cars which are in any of the lanes of interest and are
     * within the Field of View of the current s position of the Ego Vehicle
     * need to be considered(S-Wrap-around also needs to be taken into account)
     */
    if( getLane(car_d) != -1 )
    {
       print_to_console("Not junk data");
       if( front_wrap == true )
       {
         /* For Front Field of view wrap-around, any target vehicle in front of ego will be
          * 1) Either having S greater than Ego Vehicle's S and lesser than MAX_S
          * 2) Having S lesser than Ego due to wraparound and also lesser than s_dist_front_fov
          */
         if( ( ( car_s >= ego_vdata.dist_s ) && ( car_s <= MAX_S ) ) ||
             ( ( car_s < ego_vdata.dist_s ) && ( car_s <= s_dist_front_fov ) ) )
         {
           target_vehicle = true;
           vehicle_in_front = true;
           print_to_console("Target vehicle found and veh is in front with front wrap");
         }
         /* For Front field of view wrap-around, any target vehicle behind ego will be having:
          * S lesser than EGO's and within LANE_FIELD_OF_VIEW
          */
         else if( ( ego_vdata.dist_s > car_s ) && ( fabs(ego_vdata.dist_s-car_s) <= LANE_FIELD_OF_VIEW) )
         {
           target_vehicle = true;
           vehicle_in_front = false;
           print_to_console("Target vehicle found and veh is not in front with front wrap");
         }
       }
       else if( back_wrap == true )
       {
         /* For Back Field of view wrap-around any target vehicle behind EGO will be
          * 1) Either having S greater than Ego vehicle's and S greater/equal than MAX_S - s_dist_back_fov
          * 2) Having S lesser than Ego.
          */
         if( ( ( car_s >= ego_vdata.dist_s ) && ( car_s >= MAX_S - s_dist_back_fov) ) ||
             (car_s <= ego_vdata.dist_s ) )
         {
           target_vehicle = true;
           vehicle_in_front = false;
           print_to_console("Target vehicle found and veh is in back with back wrap");
         }
         /* For Back field of view wrap-around any target vehicle in front of ego will be having
          * S as greater than Ego's S within LANE_FIELD_OF_VIEW
          */
         else if ( ( car_s > ego_vdata.dist_s ) && ( fabs(car_s - ego_vdata.dist_s) <= LANE_FIELD_OF_VIEW) )
         {
           target_vehicle = true;
           vehicle_in_front = true;
           print_to_console("Target vehicle found and veh is in front with back-wrap");
         }
       }
       else
       {
         /* For no wrap case, the vehicle is either within Field of View distance back or front of Ego's S*/
         if( fabs(car_s-ego_vdata.dist_s) <= (double)LANE_FIELD_OF_VIEW )
         {
           target_vehicle = true;
           if( car_s >= ego_vdata.dist_s )
             vehicle_in_front = true;
           else
             vehicle_in_front = false;
           print_to_console("Target vehicle found without front or back wrap");
         }
       }
    }
    /* Generate predictions for a 1 second time-horizon i.e. 50 points each 0.02 seconds apart and inserts into
     * the predictions vector*/
    if(target_vehicle == true)
    {
      if(LOGGING_ENABLED)
      {
        outfile.open("logs.txt", ios::out| ios::app);
        outfile<<"D initial of the predicted car is"<<car_d<<endl;
        outfile<<"S initial of the predicted car is"<<car_s<<endl;
        outfile.close();
      }
      VehData vdata;
      vdata.accl_s = 0.0;
      vdata.dist_d = car_d;
      vdata.dist_s = car_s;
      vdata.vel_s = car_speed;
      if( vehicle_in_front == true)
      {
        vdata.is_veh_ahead = true;
      }
      else
      {
        vdata.is_veh_ahead = false;
      }
      predictions.push_back(vdata);
      if(LOGGING_ENABLED)
      {
        outfile.open("logs.txt", ios::out| ios::app);
        outfile<<"One vehicle Added Checkpoint to Predictions           "<<endl;
        outfile.close();
      }
    }
  }
}

vector<string> Vehicle::successor_states(Trajectory &traj_current) {
  vector<string> states;

  /* If a lane change is in progress due to LCL or LCR, allow a successor KL state
   * only if the lane change if finished i.e. EGO ends up being the target lane
   */
  if ( !( (this->is_lane_change_in_progress == true ) &&
          ( getLane(this->ego_vdata.dist_d) != this->target_lane ) ) )
  {
    states.push_back("KL");
    this->is_lane_change_in_progress = false;
  }

  string state = this->ego_state;

  if(state.compare("KL") == 0) {
    if( getLane(ego_vdata.dist_d) != FIRST_LANE_ID )
    {
      states.push_back("PLCL");
    }
    if( getLane(ego_vdata.dist_d) != LAST_LANE_ID )
    {
      states.push_back("PLCR");
    }
    //print_to_console("PLCL and PLCR are possible states");
  } else if (state.compare("PLCL") == 0) {
    //Only state transition possible is to go back to KL if its the left most lane.
    if (getLane(ego_vdata.dist_d) != FIRST_LANE_ID) {
      states.push_back("PLCL");
      states.push_back("LCL");
      print_to_console("PLCL and LCL are possible states");
    }
  } else if (state.compare("PLCR") == 0) {
    //Only state transition possible is to go back to KL if its the right most lane
    if (getLane(ego_vdata.dist_d) != LAST_LANE_ID) {
      states.push_back("PLCR");
      states.push_back("LCR");
      print_to_console("PLCR and LCR are possible states");
    }
  }
  else if( ( state.compare("LCL") == 0) && this->is_lane_change_in_progress)
  {
    /* This ensures that a LCL can result in a transition back to itself to enable
     * EGO to finish the transition to the target lane.
     */
    states.push_back("LCL");
  }
  else if(( state.compare("LCR") == 0) && this->is_lane_change_in_progress)
  {
    /* This ensures that a LCL can result in a transition back to itself to enable
     * EGO to finish the transition to the target lane.
     */
    states.push_back("LCR");
  }

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<double> Vehicle::generate_trajectory(string state ) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<double> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
    print_to_console("Constant Speed Traj generated");
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory();
    print_to_console("Keep-Lane Traj generated");
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state);
    print_to_console("LCL or LCR traj generated");
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state);
    print_to_console("PLCR or PLCL traj generated");
  }

  return trajectory;
}


bool Vehicle::get_vehicle_behind(int lane, VehData &rVehicle)
{
  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  double dist = MAX_S;
  bool found_vehicle = false;
  VehData temp_vehicle;
  ofstream outfile;
  for (vector<VehData>::iterator it = predictions.begin();it != predictions.end(); ++it) {
    temp_vehicle = (*it);

    /* Find the Vehicle that is in the same lane as the lane of interest, is behind EGO
     * and also the closest to EGO in terms of S-distance.
     */
    if( ( getLane(temp_vehicle.dist_d) == lane) && (temp_vehicle.is_veh_ahead == false) &&
        ( getSdistance(ego_vdata.dist_s, temp_vehicle.dist_s, true) < dist) )
    {
      dist = getSdistance(ego_vdata.dist_s, temp_vehicle.dist_s, true);
      rVehicle = temp_vehicle;
      found_vehicle = true;
      if(LOGGING_ENABLED)
      {
        outfile.open("logs.txt", ios::out| ios::app);
        outfile<<"Vehicle found behind with distance"<<dist<<"in lane"<<lane<<endl;
        outfile.close();
      }
    }
  }

  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(int lane, VehData &rVehicle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  double dist = MAX_S;
  bool found_vehicle = false;
  VehData temp_vehicle;
  ofstream outfile;
  for (vector<VehData>::iterator it = predictions.begin();it != predictions.end(); ++it) {
    temp_vehicle = (*it);

    /* Find the Vehicle that is in the same lane as the lane of interest. is in front of EGO
     * and also the closest to EGO in terms of S-Distance
     */
    if( ( getLane(temp_vehicle.dist_d) == lane) && (temp_vehicle.is_veh_ahead == true) &&
        ( getSdistance(ego_vdata.dist_s, temp_vehicle.dist_s, false) < dist) )
    {
      dist = getSdistance(ego_vdata.dist_s, temp_vehicle.dist_s, false);
      rVehicle = temp_vehicle;
      found_vehicle = true;
      if(LOGGING_ENABLED)
      {
        outfile.open("logs.txt", ios::out| ios::app);
        outfile<<"Vehicle found ahead with distance"<<dist<<"in lane"<<lane<<endl;
        outfile.close();
      }
    }
  }

  return found_vehicle;
}

vector<double> Vehicle::get_kinematics(int lane) {
  // Gets next timestep kinematics (position, velocity, acceleration)
  //   for a given lane. Tries to choose the maximum velocity and acceleration,
  //   given other vehicle positions and accel/velocity constraints.
  double max_velocity_accel_limit = MAX_ACCELERATION + ego_vdata.vel_s;
  double new_position;
  double new_velocity;
  double new_accel = 0.0;
  VehData vehicle_ahead;
  VehData vehicle_behind;
  ofstream outfile;

  if (get_vehicle_ahead(lane, vehicle_ahead)) {
    if (get_vehicle_behind(lane, vehicle_behind)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      new_velocity = vehicle_ahead.vel_s;
      if(LOGGING_ENABLED)
      {
        outfile.open("logs.txt", ios::out| ios::app);
        outfile<<"Vehicle found ahead and behind, so new velocity is that of veh ahead"<<new_velocity<<endl;
        outfile.close();
      }
    }
    else
    {
      double dist_veh_ego = getSdistance( ego_vdata.dist_s,vehicle_ahead.dist_s,false);
      double max_velocity_in_front = ( dist_veh_ego - PREFERRED_BUFFER_S )/TIME_DURATION_FOR_TRAJ + vehicle_ahead.vel_s - 0.5 * ego_vdata.accl_s*TIME_DURATION_FOR_TRAJ;
      if(LOGGING_ENABLED)
      {
        outfile.open("logs.txt", ios::out| ios::app);
        outfile<<"Vehicle found ahead but not behind, so max velocity in front is "<<max_velocity_in_front<<endl;
        outfile<<"Vehicle is ahead of ego with distance"<<dist_veh_ego<<" "<<"with velocity"<<vehicle_ahead.vel_s<<endl;
        outfile.close();
      }
      new_velocity = std::max(std::min(std::min(max_velocity_in_front,
                                       max_velocity_accel_limit),
                                       TARGET_SPEED),0.0);
      if(LOGGING_ENABLED)
      {
        outfile.open("logs.txt", ios::out| ios::app);
        outfile<<"Max velocity in front was"<<max_velocity_in_front<<endl;
        outfile<<"MAx accl limit was"<<max_velocity_accel_limit<<endl;
        outfile<<"Vehicle found ahead but not behind, so new velocity"<<new_velocity<<endl;
        outfile.close();
      }
    }
  }
  else
  {
    new_velocity = std::min(max_velocity_accel_limit, TARGET_SPEED);
    if(LOGGING_ENABLED)
    {
      outfile.open("logs.txt", ios::out| ios::app);
      outfile<<"MAx accl limit was"<<max_velocity_accel_limit<<endl;
      outfile<<"Vehicle not ahead or behind so vel is "<<new_velocity<<endl;
      outfile.close();
    }
  }

  new_position = findMod((ego_vdata.dist_s + new_velocity*TIME_DURATION_FOR_TRAJ + new_accel*TIME_DURATION_FOR_TRAJ*TIME_DURATION_FOR_TRAJ / 2.0),MAX_S);

  //The data returned from this function is the final goal for S-coordinate so the acceleration returned is 0.0 {s,s_dot,s_double_dot, 0.0}
  return{new_position, new_velocity, 0.0};
}


vector<double> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  double next_pos = ego_vdata.dist_s + ego_vdata.vel_s*TIME_DURATION_FOR_TRAJ;
  //Return the final_d, final_d_dot, final_d_double_dot, final s, final s_dot, final s_double_dot endpoint
  return {ego_vdata.dist_d,0.0,0.0,next_pos,ego_vdata.vel_s,0.0};
}

vector<double> Vehicle::keep_lane_trajectory() {
  ofstream outfile;
  // Generate a keep lane trajectory.
  vector<double> kinematics = get_kinematics(getLane(ego_vdata.dist_d));
  if(LOGGING_ENABLED)
  {
      outfile.open("logs.txt", ios::out| ios::app);
      outfile<<"KL Traj generated"<<endl;
      outfile<<"S final chosen for KL Trajectory is "<<kinematics[0]<<endl;
      outfile<<"S velocity final chosen for KL Trajectory is "<<kinematics[1]<<endl;
      outfile<<"S acceleration final chosen for KL Trajectory is"<<kinematics[2]<<endl;
      outfile<<"Lane used for KL was"<<getLane(ego_vdata.dist_d)<<endl;
      outfile.close();
   }
  //Return the final_d, final_d_dot, final_d_double_dot, final s, final s_dot, final s_double_dot endpoint
  return {gecenterD(getLane(ego_vdata.dist_d)),0.0,0.0,kinematics[0],kinematics[1],kinematics[2]};
}

vector<double> Vehicle::prep_lane_change_trajectory(string state) {
  // Generate a trajectory preparing for a lane change.
  double new_s;
  double new_v;
  double new_a;
  VehData vehicle_behind;
  ofstream outfile;
  int new_lane = getLane(ego_vdata.dist_d) + lane_direction[state];
  vector<double> curr_lane_new_kinematics = get_kinematics(getLane(ego_vdata.dist_d));

  bool is_veh_behind = get_vehicle_behind(getLane(ego_vdata.dist_d), vehicle_behind);

  if (is_veh_behind && getSdistance(ego_vdata.dist_s,vehicle_behind.dist_s,true) <= LANE_FIELD_OF_VIEW_BACK)
  //if(is_veh_behind == true)
  {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];
  } else {
    vector<double> best_kinematics;
    vector<double> next_lane_new_kinematics = get_kinematics(new_lane);
    // Choose kinematics with best velocity.
    if (next_lane_new_kinematics[1] > curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  if(LOGGING_ENABLED)
  {
      outfile.open("logs.txt", ios::out| ios::app);
      if(state.compare("PLCL") == 0)
      {
        outfile<<"Prepare lane change left traj generated"<<endl;
      }
      else
      {
        outfile<<"Prepare lane change right traj generated"<<endl;
      }
      outfile<<"S final chosen for Prep traj is"<<new_s<<endl;
      outfile<<"S velocity final chosen for Prep traj is "<<new_v<<endl;
      outfile<<"S acceleration final chosen for Prep traj is"<<new_a<<endl;
      outfile.close();
   }
  //Return the final_d, final_d_dot, final_d_double_dot, final s, final s_dot, final s_double_dot endpoint
  return {ego_vdata.dist_d,0.0,0.0,new_s,new_v,new_a};
}

vector<double> Vehicle::lane_change_trajectory(string state) {
  // Generate a lane change trajectory.
  int new_lane = getLane(ego_vdata.dist_d) + lane_direction[state];
  VehData next_lane_vehicle;
  ofstream outfile;
  // Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  for (vector<VehData>::iterator it = predictions.begin();it != predictions.end(); ++it) {
    next_lane_vehicle = (*it);
    if (next_lane_vehicle.dist_s == ego_vdata.dist_s && getLane(next_lane_vehicle.dist_d) == new_lane) {
      // If lane change is not possible, return empty trajectory.
      return {};
    }
  }
  vector<double> kinematics = get_kinematics(new_lane);
  if(LOGGING_ENABLED)
  {
    outfile.open("logs.txt", ios::out| ios::app);
    if(state.compare("LCL") == 0)
    {
      outfile<<"lane change left traj generated"<<endl;
    }
    else
    {
      outfile<<"lane change right traj generated"<<endl;
    }
    outfile<<"S final chosen for lane change traj is"<<kinematics[0]<<endl;
    outfile<<"S velocity final chosen for lane change traj is "<<kinematics[1]<<endl;
    outfile<<"S acceleration final chosen for lane change traj is"<<kinematics[2]<<endl;
    outfile.close();
  }
  return {gecenterD(new_lane),0.0,0.0,kinematics[0], kinematics[1],kinematics[2]};

}

void Vehicle::Exec_Behaviour_Planner_Actions(Trajectory &traj_current, vector<vector<double>> sensor_fusion_data)
{
  vector<vector<vector<double>>> potential_trajs;
  vector<double> potential_traj_cost;
  vector<string> potential_states;
  vector<vector<double>> potential_s_coeffs_jmt;
  vector<vector<double>> potential_d_coeffs_jmt;
  ofstream outfile;

  //Generate predictions from Sensor Fusion data
  this->generate_predictions(TIME_DURATION_FOR_TRAJ,sensor_fusion_data,this->previous_path_sz);
  print_to_console("Predictions generated");

  //Generate Successor States based on egos current state
  vector<string> states = successor_states(traj_current);
  print_to_console("Successor states identified");

  //Obtain {d,d_dot, d_double_dot, s., s_dot, s_double_dot} for each successor state
  for( vector<string>::iterator it = states.begin(); it != states.end(); ++it)
  {
     vector<double> traj_final_state = generate_trajectory(*it);

     //Only generate a JMT if the final state of the Trajectory for this potential successor state is even a possibility
     if(traj_final_state.size() > 0 )
     {
       print_to_console("Candidate Trajectory found");
       //Calculate the JMT for the Final trajectory state
       vector<vector<double>> gen_traj;
       vector<double> s_coeffs_jmt;
       vector<double> d_coeffs_jmt;
       //Get the Initial Traj points for S and D Trajs
       vector<double> traj_init_state = traj_current.getPrevTraj_params(this->final_pt_sent);

       /* Account for wrap-around to ensure JMT traj generation is not impacted
        * when obtaining the S and D coefficients.
        */
       if( traj_init_state[3] > traj_final_state[3])
       {
         traj_final_state[3] = MAX_S + traj_final_state[3];
       }

       //Generate 75 points for the S and D trajectories
       gen_traj = traj_current.generate_Horizon_Trajectory(traj_init_state,traj_final_state,TIME_DURATION_FOR_TRAJ,s_coeffs_jmt, d_coeffs_jmt );

       //Calculate the cost of this JMT with predictions and the S & D of this trajectory
       double traj_cost = total_trajectory_cost(TIME_DURATION_FOR_TRAJ,gen_traj,predictions,s_coeffs_jmt, d_coeffs_jmt );
       if(LOGGING_ENABLED)
       {
         outfile.open("logs.txt", ios::out| ios::app);
         outfile<<"Total cost for candidate trajectory state: "<<(*it)<<" is"<<traj_cost<<endl;
         outfile.close();
       }
       potential_traj_cost.push_back(traj_cost);
       potential_trajs.push_back(gen_traj);
       potential_states.push_back(*it);
       potential_s_coeffs_jmt.push_back(s_coeffs_jmt);
       potential_d_coeffs_jmt.push_back(d_coeffs_jmt);
     }

  }

  /* Choose the state with the min cost and set the state of EGO for the corresponding successor state
   * and also save the generated S and D points to the traj_current variable.
   */
  int minElementIndex = std::min_element(potential_traj_cost.begin(),potential_traj_cost.end()) - potential_traj_cost.begin();
  traj_current.SetPrevTrajectory(potential_trajs[minElementIndex],1.0, potential_s_coeffs_jmt[minElementIndex],potential_d_coeffs_jmt[minElementIndex]);

  /* Mark a lane change in progress to ensure that LCL and LCR states only result in going back
   * to KL state once the lane change actually finishes.
   */
  if( ( potential_states[minElementIndex] != this->ego_state) &&
      ( this->is_lane_change_in_progress == false ) &&
      ( ( !potential_states[minElementIndex].compare("LCL") || !potential_states[minElementIndex].compare("LCR") ) ) )
  {
    this->is_lane_change_in_progress = true;
    //this->target_lane = getLane(traj_current.prev_path_d[MAX_POINTS_TO_SIM-1].p) + lane_direction[potential_states[minElementIndex]];
    this->target_lane = getLane(traj_current.prev_path_d[NUM_POINTS_FOR_TRAJ-1].p);
  }

  /* Set the current state and clear the predictions to make it ready for next iteration */
  this->ego_state = potential_states[minElementIndex];
  this->predictions.clear();
  cout<<"Trajectory selected is: "<<this->ego_state<<endl;

  if(LOGGING_ENABLED)
  {
    outfile.open("logs.txt", ios::out| ios::app);
    outfile<<"Trajectory chosen is "<<this->ego_state<<endl;
    outfile<<"Speed at end of chosen trajectory is"<<traj_current.prev_path_s[NUM_POINTS_FOR_TRAJ-1].p_dot<<endl;
    outfile<<"S-Distance at end of chosen trajectory is"<<traj_current.prev_path_s[NUM_POINTS_FOR_TRAJ-1].p<<endl;
    outfile<<"D-Distance at end of chosen trajectory is"<<traj_current.prev_path_d[NUM_POINTS_FOR_TRAJ-1].p<<endl;
    outfile.close();
  }


}
