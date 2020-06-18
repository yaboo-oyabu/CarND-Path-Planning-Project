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


const double kDistanceThreshold = 30.0;
const int kNumAvailableLanes = 3;
const float kCollisionWeight = 1.0;
const float kInefficiencyWeight = 0.5;

class PathPlanner {
 public:
  PathPlanner(
      double x, double y, double s, double d, double yaw, double speed,
      const vector<double> &previous_path_x, const vector<double> &previous_path_y, 
      const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
      const vector<double> &map_waypoints_s,
      const vector< vector<double> > &sensor_fusion);

  ~PathPlanner() {};

  void getTrajectory(string &state, int &current_lane, double &current_velocity,
                     vector<double> &next_x_vals, vector<double> &next_y_vals);


 private:
  bool isLeadVehicleTooClose(int current_lane);
  vector<string> getPossibleStates(int current_lane, string state);
  float calculateCost(
      const int &target_lane, const double &target_velocity,
      const vector<double> &points_x, const vector<double> &points_y);
  float calculateCollisionCost(
      const vector<double> &points_x, const vector<double> &points_y);
  float calculateInefficiencyCost(
      const vector<double> &points_x, const vector<double> &points_y);

  Ego ego;
  map<int, Ado> ados;
};

#endif // PATH_PLANNING_H
