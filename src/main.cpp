#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "vehicle.h"
#include "map.h"
#include "trajectory.h"
#include <cmath>
#include <algorithm>

using namespace std;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> init_map_waypoints_x;
  vector<double> init_map_waypoints_y;
  vector<double> init_map_waypoints_s;
  vector<double> init_map_waypoints_dx;
  vector<double> init_map_waypoints_dy;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  Vehicle ego_v;
  bool is_first_data_point = true;
  Trajectory traj;
  ofstream outfile;
  double prev_s;
  double inst_accl = 0.0;
  double inst_vel  = 0.0;
  double ref_vel = 0.0;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    init_map_waypoints_x.push_back(x);
    init_map_waypoints_y.push_back(y);
    init_map_waypoints_s.push_back(s);
    init_map_waypoints_dx.push_back(d_x);
    init_map_waypoints_dy.push_back(d_y);
  }

  /* Generate a smoother map by using spline which leads to smoother trajectories since more way-points are available */
  Map_data mp;
  mp.buildAccurateMap(init_map_waypoints_s,init_map_waypoints_x,init_map_waypoints_y,init_map_waypoints_dx,init_map_waypoints_dy,(double)MAX_S);

  map_waypoints_x = mp.map_waypoints_x;
  map_waypoints_y = mp.map_waypoints_y;
  map_waypoints_s = mp.map_waypoints_s;
  map_waypoints_dx = mp.map_waypoints_dx;
  map_waypoints_dy = mp.map_waypoints_dy;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_v, &is_first_data_point, &traj, &mp,&outfile,&prev_s,&inst_accl,&inst_vel,&ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          car_speed = mphtoms(car_speed);

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;

          /* Use the spline apporach if the car-speed is lesser than the initial cutoff velocity of 10 m/s since JMT approach only works
           * well at higher speeds.
           */
          if(car_speed < CUTOFF_INITIAL_VELOCITY)
          {
            vector<double> ptsx;
            vector<double> ptsy;
            int lane = getLane(car_d);

            /* Clean this up in-case of switching between Spline and JMT */
            traj.prev_path_s.clear();
            traj.prev_path_d.clear();
            ego_v.final_pt_sent = 0;
            ego_v.ego_state = "KL";

            if(LOGGING_ENABLED)
            {
              outfile.open("logs.txt", ios::out| ios::app);
              outfile<<"Inside splien generator"<<endl;
              outfile<<"Size of prev path reported back is"<<previous_path_x.size()<<endl;
              outfile<<"End path S and S coordinates of prev trajectory are"<<end_path_s<<" "<<end_path_d<<endl;
              outfile<<"Car speed is"<<car_speed<<endl;
            }

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            int prev_size = previous_path_x.size();

            if ( prev_size < 2 ) {

              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            } else {
              // Use the last two points.
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            // Setting up target points in the future.
            vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // Making coordinates to local car coordinates.
            for ( int i = 0; i < ptsx.size(); i++ ) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // Create the spline.
            tk::spline s;
            s.set_points(ptsx, ptsy);

            // Output path points from previous path for continuity.
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            for ( int i = 0; i < prev_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
              if(LOGGING_ENABLED)
                outfile<<"kth X,Y val of prev path in Spline generator"<<previous_path_x[i]<<" "<<previous_path_y[i]<<endl;
            }

            // Calculate distance y position on 30 m ahead.
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            for( int i = 1; i < 50 - prev_size; i++ ) {
              if ( ref_vel > TARGET_SPEED ) {
                inst_accl = -0.10;
                ref_vel -= 0.10;
              }
              else
              {
                ref_vel += 0.10;
                inst_accl = 0.10;
              }
              double N = target_dist/(0.02*ref_vel);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
              if(LOGGING_ENABLED)
                outfile<<"kth X,Y val of prev path in Spline generator"<<x_point<<" "<<y_point<<endl;
            }

            if(LOGGING_ENABLED)
              outfile.close();

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            is_first_data_point = true;
            //Calculate the instantaenous acceleration using the last 4 points
            inst_vel = ref_vel;
          }
          else
          {
            double tmp_s;
            double tmp_d;
            if( previous_path_x.size()>0)
            {
              double delta_x = previous_path_x[previous_path_x.size()-1] - previous_path_x[previous_path_x.size()-2];
              double delta_y = previous_path_y[previous_path_y.size()-1] - previous_path_y[previous_path_y.size()-2];
              double theta = atan2(delta_y,delta_x);
              vector<double> s_d = getFrenet(previous_path_x[previous_path_x.size()-1],previous_path_y[previous_path_y.size()-1] ,theta,map_waypoints_x, map_waypoints_y);

              /* Since there is a previous path reported back by the Simulator, use the last two points of the previous path to obtain
               * the current S and D coordinates if the previous trajectory stored in the JMT Trajectory database is empty.
               */
              if( traj.prev_path_s.size() == 0)
              {
                tmp_s = s_d[0];
                tmp_d = s_d[1];
              }
              else
              {
                /* Since previous points were actually fed from the JMT Trajectory database to the simulator, use the starting S and D values
                 * from the last point of the trajectory stored in the JMT Database that was sent to the simulator.
                 */
                tmp_s = traj.getPrevTrajS(ego_v.final_pt_sent);
                tmp_d = traj.prev_path_d[ego_v.final_pt_sent].p;
              }

            }
            else
            {
              /* This is dead code and will not be hit, too scared to remove it though */
              tmp_s = car_s;
              tmp_d = car_d;
            }

            if( is_first_data_point == true )
            {
              traj.InitTrajectory(tmp_d,tmp_s,inst_vel,inst_accl);
              is_first_data_point = false;
              cout<<"First data point Initialized, transitioned from Spline to JMT approach"<<endl;
              inst_vel = 0.0;
              inst_accl = 0.0;
            }

            ref_vel = car_speed;

            if(LOGGING_ENABLED)
            {
              outfile.open("logs.txt", ios::out| ios::app);
              outfile<<"                                "<<endl;
              outfile<<"Inside JMT code path"<<endl;
              outfile<<"Size of prev path reported back is"<<previous_path_x.size()<<endl;
              outfile<<"Current S and D coordinates are"<<car_s<<" "<<car_d<<endl;
              if( traj.prev_path_s.size() >0 )
                outfile<<"Last point S and D value of previous JMT traj sent to Sim"<<traj.prev_path_s[ego_v.final_pt_sent].p<<" "<<traj.prev_path_d[ego_v.final_pt_sent].p<<endl;
            }

            ego_v.previous_path_sz = previous_path_x.size();
            ego_v.SetEgoVehData(tmp_d,tmp_s,traj.getPrevTrajS_dot(ego_v.final_pt_sent),traj.getPrevTrajS_dotdot(ego_v.final_pt_sent));

            //Generate Predictions, Potential Trajectories and select the Best Rated Trajectory using Sensor Fusion Data
            ego_v.Exec_Behaviour_Planner_Actions(traj, sensor_fusion);

            //Convert the Best Rated Trajectory from S and D trajectories to X And Y vals
            vector<vector<double>> sd_traj = traj.GetCurrentSDTrajectory();
            vector<vector<double>> xy_traj = mp.getXYfromSD_withSpline(sd_traj);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            //First insert the left over points from the Previous Trajectory reported by the Simulator
            for(int k=0; k < previous_path_x.size() ; k++)
            {
              next_x_vals.push_back(previous_path_x[k]);
              next_y_vals.push_back(previous_path_y[k]);
              if(LOGGING_ENABLED)
              {
                outfile<<"kth X,Y val of prev path from prev traj"<<previous_path_x[k]<<" "<<previous_path_y[k]<<endl;
              }
            }

            //Now insert the remaining 50 - current_size points of the currently calculated trajectory to the lists.
            for(int k=1; k < MAX_POINTS_TO_SIM-previous_path_x.size(); k++)
            {
              next_x_vals.push_back(xy_traj[0][k]);
              next_y_vals.push_back(xy_traj[1][k]);
              if(LOGGING_ENABLED)
              {
                outfile<<"kth X,Y val of prev path from cur traj"<<xy_traj[0][k]<<" "<<xy_traj[1][k]<<endl;
              }
            }

            //Save the final point of the generated trajectory that was sent to the smulator. This will act as a starting point for the next iteration.
            ego_v.final_pt_sent = MAX_POINTS_TO_SIM-previous_path_x.size() -1;

            if(LOGGING_ENABLED)
              outfile.close();

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
          }

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
