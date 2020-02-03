#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

/* Macro definitions */
#define LOGGING_ENABLED 0
#define LANE_FIELD_OF_VIEW 40.0 //in metres
// The max s value before wrapping around the track back to 0
#define MAX_S 6945.554 //in metres
#define MAX_ACCELERATION 10.0 //in m/s^2
#define MAX_JERK 10.0 //in m/s^3
#define PREFERRED_BUFFER_S 30 //in metres
#define MAX_POINTS_TO_SIM 50
#define FIRST_LANE_ID 0
#define LAST_LANE_ID 2
//#define TARGET_SPEED 20.0 //in m/s
//#define TARGET_SPEED 21.4579 //48 MPH
#define TARGET_SPEED 20.5638 //46 MPH
#define VEHICLE_RADIUS 1.5
#define EXPECTED_JERK_IN_ONE_SEC 2.0 //m/s/s
#define EXPECTED_ACC_IN_ONE_SEC 1.0 //m/s

#define CUTOFF_INITIAL_VELOCITY 10
/* Defines the point within the previous Path to be used as a starting point
 * for the current JMT
 */
#define INIT_POINT_JMT 49
#define MAX_ACCELERATION_PER_POINT 0.20

#define LANE_FIELD_OF_VIEW_BACK 20.0
#define LANE_FIELD_OF_VIEW_BOOST 30.0
#define S_THRESH 5.0
#define D_THRESH 0.2

#define TIME_DURATION_FOR_TRAJ 1.50
#define NUM_POINTS_FOR_TRAJ 75


/* Represents the s and d-data of each Vehicle
 * used for maintaining Predictions data of all non-ego
 * vehicles as well as the type used for a Trajectory for
 * the EGO vehicle.
 */
typedef struct _VehData
{
   //Distance in D direction
   double dist_d;
   //Distance in s direction
   double dist_s;
   //Velocity in S Direction
   double vel_s;
   //Acceleration in S Direction
   double accl_s;
   /* Indicates whether the vehicle is ahead or behind the Ego Vehicle obtained from the Sensor Fusion data.
    * Not applicable for
    */
   bool is_veh_ahead;

   void SetData(double d_coord,double s_coord,double vel, double accl );
} VehData;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
double pi();
double deg2rad(double x);
double rad2deg(double x);

//Convert miles per hour to metres per second
double mphtoms(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

/* Convert Car's d frenet coordinate to lane numbers such that:
 * 0 to 4 is lane 0
 * 4 to 8 is lane 1
 * 8 to 12 is lane 2
 */
int getLane(double d);

double gecenterD(int lane);

/* Returns the absolute distance between two points
 * whose S-Coordinates could have potentially wrapped around
 */
double getSdistance(double ego_s, double car_s, bool is_car_behind);

/* Reference: geeksforgeeks*/
double findMod(double a, double b);

double logistic(double x);

void print_to_console(string s);

#endif  // HELPERS_H
