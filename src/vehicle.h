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


class Ego {
 public:
  Ego(double x, double y, double s, double d, double yaw, double speed,
      const vector<double> &previous_path_x,
      const vector<double> &previous_path_y,
      const vector<double> &map_waypoints_x,
      const vector<double> &map_waypoints_y,
      const vector<double> &map_waypoints_s);
  Ego(){};
  ~Ego(){};

  void generateTrajectory(const int &target_lane,
                          const double &target_velocity,
                          vector<double> &trajectory_x,
                          vector<double> &trajectory_y);

  void generateTrajectoryForState(const string &state,
                                  const int &current_lane,
                                  const double &current_velocity,
                                  vector<double> &next_px,
                                  vector<double> &next_py,
                                  int &next_lane,
                                  double &next_velocity);

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
  Ado(double id, double x, double y, double s, double d, double vx, double vy,
      const vector<double> &map_waypoints_x,
      const vector<double> &map_waypoints_y,
      const vector<double> &map_waypoints_s);
  Ado() {};
  ~Ado() {};

  void predictPosition(int num_steps);
  
  double id;
  double x;
  double y;
  double s;
  double d;
  double vx;
  double vy;
  double v;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  double next_s;
  double next_d;
  double next_x;
  double next_y;
};



#endif // VEHICLE_H
