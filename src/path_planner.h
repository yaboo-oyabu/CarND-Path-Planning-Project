#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <vector>
#include <map>
#include <math.h>
#include <string>

#include "vehicle.h"


using std::vector;
using std::string;
using std::map;

class PathPlanner {
 public:
  PathPlanner(
      double x, double y, double s, double d, double yaw, double speed,
      const vector<double> &previous_path_x, const vector<double> &previous_path_y, 
      const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
      const vector<double> &map_waypoints_s,
      const vector< vector<double> > &sensor_fusion);

  ~PathPlanner() {};

  bool isLeadVehicleTooClose(int current_lane);
  vector<string> getAvailableStates(int current_lane, string state);
  void getTrajectory(int &current_lane, string &state, double &current_velocity,
                     vector<double> &next_x_vals, vector<double> &next_y_vals);

 private:
  Ego ego;
  map<int, Ado> ados;
};

#endif // PATH_PLANNING_H
