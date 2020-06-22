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

  ~PathPlanner() = default;

  void getTrajectory(string &state, int &current_lane, double &current_velocity,
                     vector<double> &next_x_vals, vector<double> &next_y_vals);

 private:
  // review relationship between these values.
  const double kSpeedLimit = 50.0;
  const double kMaxVelocity = 48.0;
  const double kMaxAcceleration = 0.224;
  const double kMaxDeceleration = 0.224;
  const double kDistanceThreshold = 40.0;
  const double kMinLaneChangeVelocity = 30.0;

  const double kBufferVelocity = 2.0;
  const int kNumAvailableLanes = 3;

  const float kStopCost = 0.8;
  const float kSpeedWeight = 4.0;
  const float kCollisionWeight = 5.0;
  const float kStateWeight = 1.0;
  const float kTotalWeight = kSpeedWeight + kCollisionWeight + kStateWeight;
  
  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1},
                                     {"PLCR",  1}, {"LCR",  1}, {"KL", 0}};

  vector<string> getPossibleStates(string state, int &curr_lane, double curr_velocity);
  int getCurrentLane(int curr_lane);
  void execLaneKeep(string curr_state, int curr_lane, double curr_velocity,
                      int &next_lane, double &next_velocity);
  void prepLaneChange(string curr_state, int curr_lane, double curr_velocity,
                      int &next_lane, double &next_velocity);
  void execLaneChange(string curr_state, int curr_lane, double curr_velocity,
                      int &next_lane, double &next_velocity);
  void getKinematics(int &next_lane, double &curr_velocity, double &next_velocity);
  bool getVehicleAhead(int curr_lane, Ado &ado_ahead);
  bool getVehicleBehind(int curr_lane, Ado &ado_behind);

  float calculateCost(
      const string &curr_state, const int &next_lane, const double &next_velocity,
      const vector<double> &traj_x_vals, const vector<double> &traj_y_vals);
  float calculateSpeedCost(const double &next_velocity);
  float calculateCollisionCost(const vector<double> &points_x,
                               const vector<double> &points_y);
  float calculateInefficiencyCost(const vector<double> &points_x,
                                  const vector<double> &points_y);
  float calculateStateCost(const string &state);

  Ego ego;
  map<int, Ado> ados;
};

#endif // PATH_PLANNING_H
