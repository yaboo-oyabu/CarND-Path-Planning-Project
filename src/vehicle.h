#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <map>
#include <math.h>
#include <string>

using std::vector;
using std::string;
using std::map;


const double kSimInterval = 0.02;

class Vehicle {
 public:
  Vehicle() = default;
  
  // For an ego vehicle.
  Vehicle(double x, double y, double s, double d, double yaw, double speed,
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
  double getSpeed(void);

  void generateTrajectoryXY(const int &target_lane, const double &target_velocity,
                            vector<double> &trajectory_x, vector<double> &trajectory_y);
  void generateTrajectorySD(const int &target_lane, const double &target_velocity,
                            vector<double> &trajectory_s, vector<double> &trajectory_d);
  void convertTrajectory(const vector<double> &trajectory_x,
                         const vector<double> &trajectory_y,
                         vector<double> &trajectory_s, vector<double> &trajectory_d);
 private:
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  int prev_size;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;

  double calculateSpeed(const double &vx, const double &vy);
  double calculateYaw(const double &vx, const double &vy);
};

#endif // VEHICLE_H
