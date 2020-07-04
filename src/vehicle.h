#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <map>
#include <math.h>
#include <string>

using std::vector;
using std::string;
using std::map;

class Vehicle {
 public:
  Vehicle() = default;
  
  // For an ego vehicle.
  Vehicle(double x, double y, double s, double d, double yaw, double vel,
          const vector<double> &previous_path_x,
          const vector<double> &previous_path_y,
          const vector<double> &map_waypoints_x,
          const vector<double> &map_waypoints_y,
          const vector<double> &map_waypoints_s);

  // For an ado vehicle.
  Vehicle(double x, double y, double s, double d, double vx, double vy,
          const vector<double> &map_waypoints_x,
          const vector<double> &map_waypoints_y,
          const vector<double> &map_waypoints_s);

  ~Vehicle() = default;

  double getX(void);
  double getY(void);
  double getD(void);
  double getS(void);
  double getYaw(void);
  double getVel(void);
  double getVelMph(void);

  void generateTrajectory(
      const int &target_lane, const double &target_velocity,
      vector<double> &trajectory_x, vector<double> &trajectory_y);

 private:
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double vel;
  int prev_size;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
};

#endif // VEHICLE_H
