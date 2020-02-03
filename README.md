# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Description
The objective of the Path-Planning project is to make a car drive on a highway populated with other cars in the background under a top-speed of 50 MPH by performing lane changes as necessary, slowing down and speeding up whenever needed to determine the most optimal path around the circuit loop. The Simulator has been provided by Udacity and the source code of the Path-Planning project is responsible for generating the X and Y coordinates of the path that the Simulator's car should execute.

   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


## Model Description and Reflection
---------------------------------------
### Code organization:
#### map.cpp, map.h : 
Contains functions for Map waypoint extrapolation using splines as well as helper functions to perform XY -> SD coordinate conversions and vice-versa.

#### helpers.cpp, helpers.h: 
Defines all the utility functions and macros that are used across all the source files.

#### trajectory.cpp trajectory.h: 
Defines all the functions and classes/structures that are needed for Trajectory Initialization, generation of JMT Trajectories, calculation of S and D coefficients for a Jerk-Minimizing Trajectory(JMT).

#### vehicle.cpp, vehicle.h: 
Defines all the classes/structures necessary to store the Ego vehicle's parameters and its member functions for generating predictions of other vehicles using sensor fusion, generating various types of trajectories, obtaining the S and D endpoints of any specific trajectory using kinematics and lane dynamics. It also implements the Behaviour Planner which is responsible foe generating the Final Trajectory for the EGO vehicle to follow.

#### cost_functions.h:
Defines all the cost-functions needed for computing the various costs of Any trajectory generate and the associated weights of each of these cost functions.

#### main.cpp: 
Defines the logic for either generating the X-Y trajectory for the next iteration using Splines or using Jerk-Minimizing trajectory approach.


### Map Extrapolation
The HighWay map provided only contains around 181 waypoints which wasn't sufficient to generate smooth trajectories around the whole track. In order to have smoother trajectories, the function buildAccurateMap first fits the X,Y, dx, dy to the S waypoints using a spline and then generates, X,Y,dx,dy coordinates for waypoints, each of which are 1m spaced apart which results in around 6000+ way-points and helps in generating smoother trajectories.

### Trajectory Generation
The Trajectory generation for generating the set of X-Y points to be fed into the simulator has been divided into 2 segments:

1) For a Car-Speed of less than the Initial Cutoff velocity of 10 m/s, the Trajectory points are generated using the Spline approach, that was suggested in the Project Q&A. This helps in gradual speed-up on the starting lane(lane 1) without unnecessary extra jerk/acceleration and solves the Cold-start problem. The Spline approach does the below:

  i) Takes the previous X-Ypoints and 3 more X-Y points spaced 30m,60,90m in Frenet S-Coordinate to git a spline in the vehicle's local coordinate system.
  
  ii) Uses this fitted spline to generate X-Y pairs upto a target distance and convert them back to global coordinate system to generate new X-Y points for the trajectory.
 
  iii) Feeds all points from the previously reported trajectory from the Simulator and then the remaining points(total upto 50) to the Simulator next_x and next_y values.

2) For a Car-Speed greater than 10 m/s, the JMT approach has been utilized which operates towards trajectory generation in S-D coordinates as follows:

  i) The traj class maintains the generated S and D trajectory coordinates of 75 points(for a time-span of 1.5 seconds) in the variables prev_path_s and prev_path_d.
 
  ii) The traj class is initialized for all the 75 trajectory points using the initial S and D coordinates of the last previous trajectory points sent from the Spline generator. This initialization happens for every switch been Spline to JMT trajectory generator.
 
  iii) The Ego Vehicle specific data is set via the function SetEgoVehData.
 
  iv) The function Exec_Behaviour_Planner_Actions generates predictions, calculates all possible trajectories based on the current state of EGO, evalutes the cost of all these trajectories and generates the S and D coordinates of the best trajectory in terms of cost.
 
  v) The function GetCurrentSDTrajectory gets the generated S-D trajectory in the step above and then these S-D coordinates are converted to the X-Y coordinate system using getXYfromSD_withSpline with the splines generated at the time of Map Extrapolation.
 
  vi) Finally all the points from the previous trjaectory are first fed into the next_x_vals, next_y_vals vectors, followed by the remaining points(total 50) from the currently generated X-Y trajectory in step v).
 
  vii) The last point of the currently generated trajectory is stored and recorded in the variable: final_pt_sent as this S-D trajectory point would be the starting point of the next iteration of S-D trajectory generation.


### Behaviour Planner and Trajectory Generator

The entire behaviour planning and trajectory generation and selection logic has been implemented in the function Exec_Behaviour_Planner_Actions which does the following:

   i) Generates predictions of of vehicles within the field-of-view(40 metres) of the current S of the EGO vehicle and stores them for all the lanes(0,1 and 2) via the function generate_predictions.
  
   ii) Generates successor states: KL/PLCL/PLCR/LCL/LCR based on the current state of the EGO vehcile via the function successor_states.
  
   iii) For each possible successor state, generates a start and end point of its corresponding trajectory and D and S trajectory points for a time-duration of 1.5 seconds(75 points spaced 0.02s apart) using Jerk Mimizing trajectory equations in the function generate_Horizon_Trajectory. The function generate_Horizon_Trajectory, first generates the S and D coefficients corresponding to the start and final positions for a Jerk Minimizing trajectory and then derives the S and D points for the entire duration of 1.5 seconds using these generated coefficients.
 
   iv) Derives the cost for each generated trajectory via the function total_trajectory_cost.
 
   v) Selects the trajectory with the lowest cost and saves the generated trajectory in the database via SetPrevTrajectory and sets the EGO class variables.

### Successor States state transitions:

The possible states of the EGO vehicle are:

i) KL -> Keep Lane.

ii) PLCL -> Prepare Lane Change Left.

iii) PLCR -> Prepare Lane Change Right.

iv) LCL -> Lane Change Left.

v) LCR -> Lane Change Right.

The vehicle initially starts off in KL state and then decides which state to transition to, depending upon which trajectory is ranked best from the Behaviour Planner + Trajecttory generator logic.
The possible successor states identified based on the current state of the Ego vehicle are structured as follows:

1) In KL state, PLCL is a possible state if the vehicle is not already in the left-most lane and PLCR is a possible state if the vehicle is not already in the right-most lane. Also KL state to KL state transition is a possibility.

2) In PLCL state, KL, PLCL and LCL are the possible state transitions.

3) In PLCR state, KL, PLCR and LCR are the possible state transitions.

4) In LCR state, LCR is a possible state transition and KL is only allowed as a feasible state transition when the associated lane change has been completed.

5) In LCL state, LCL is a possible state transition and KL is only allowed as a feasible state transition when the associated lane change has been completed.

### Cost Functions:

The below cost functions have been utilized to rank the possible candidate trajectories:

1) collision_cost: Penalizes a trajectory which generates a point that is within the 2*vehcile-radius of other vehciles in the prediction database.

2) total_acceleration_cost: Rewards a trajectory with a lower average acceleration within the entire time-horizon of 1.5 seconds.

3) max_accleration_cost: Penalizes a trajectory whose instantaneous acceleation exceds the Max-Acceleration for any of the points in the time-horizon of 1.5 seconds.

4) total_jerk_cost: Rewards a trajectory with a lower average jerk within the entire time-horizon of 1.5 seconds.

5) max_jerk_cost: Penalizes a trajectory whose instantaneous jerk exceds the Max-jerk for any of the points in the time-horizon of 1.5 seconds.

6) total_velocity_cost: Penalizes a trajectory whose average velocity is much lesser than the target-speed.

7) free_lane_cost: Rewards a trajectory that leads to a lane-change when there are no vehicles within the Field-of-View of the EGO vehicle.

### Link for Vehicle navigation on Highway using the above algorithm
[![](http://img.youtube.com/vi/W_7A0PqAYhk/0.jpg)](http://www.youtube.com/watch?v=W_7A0PqAYhk "Highway Driving Path Planner")

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

