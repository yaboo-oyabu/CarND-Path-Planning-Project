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
const double kDistanceThreshold = 30.0;
const int kNumAvailableLanes = 3;

class Ego {
 public:
  Ego(double x, double y, double s, double d, double yaw, double speed,
      const vector<double> &previous_path_x, const vector<double> &previous_path_y,
      const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
      const vector<double> &map_waypoints_s);
  Ego(){};
  ~Ego(){};

  void generateTrajectory(int target_lane, double target_velocity,
                          vector<double> &next_x_vals, vector<double> &next_y_vals);
  int prev_size;
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
};

class Ado {
 public:
  Ado(double id, double x, double y, double s, double d, double vx, double vy);
  Ado() {};
  ~Ado() {};

  double id;
  double x;
  double y;
  double s;
  double d;
  double vx;
  double vy;
  double v;
};

#endif // VEHICLE_H
