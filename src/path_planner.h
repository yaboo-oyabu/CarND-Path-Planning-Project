#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <vector>
#include <map>
#include <math.h>
#include <string>

#include "helpers.h"
#include "vehicle.h"


using std::vector;
using std::string;
using std::map;

enum State {KL, PLCL, PLCR, LCL, LCR};

class PathPlanner {
 public:
  PathPlanner(
      double x, double y, double s, double d, double yaw, double vel,
      const vector<double> &previous_path_x, const vector<double> &previous_path_y, 
      const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
      const vector<double> &map_waypoints_s,
      const vector< vector<double> > &sensor_fusion);

  ~PathPlanner() = default;

  void generateTrajectory(
      State &curr_state, int &curr_lane, double &curr_vel,
      vector<double> &next_x_vals, vector<double> &next_y_vals);

 private:
  const double kDelayCoef = 0.8;
  const int kNumAvailableLanes = 3;
  const double kMaxVelMph = 46.0;
  const double kMaxVelMps = kMaxVelMph * kMphToMps;
  const double kMaxAccMph = 0.244;
  const double kMaxAccMps = kMaxAccMph * kMphToMps;
  const double kMaxDccMph = 0.164;
  const double kMaxDccMps = kMaxDccMph * kMphToMps;
  const double kMinVelMphLaneChange = 30.0;
  const double kMinVelMpsLaneChange = kMinVelMphLaneChange * kMphToMps;

  Vehicle ego;
  map<int, Vehicle> vehicles;
  
  map<State, int> lane_direction = {
      {PLCL, -1}, {LCL, -1}, {PLCR,  1}, {LCR,  1}, {KL, 0}};

  // Get traffic information around ego vehicle.
  int getVehicleInRange(int lane, double min_range, double max_range, Vehicle &vehicle);
  vector< vector<int> > getSurroundingVehicleInformation(int curr_lane);

  // Determine next state.
  void changeState(State &curr_state, int &curr_lane);
  State getNextStateKL(const vector< vector<int> > &spaces);
  State getNextStatePLCL(const vector< vector<int> > &spaces);
  State getNextStatePLCR(const vector< vector<int> > &spaces);
  State getNextStateLCL(const vector< vector<int> > &spaces, const int next_lane, int &curr_lane);
  State getNextStateLCR(const vector< vector<int> > &spaces, const int next_lane, int &curr_lane);
  bool isLaneChangeCompleted(int next_lane);

  // Determine next velocity.
  void execLaneKeep(State curr_state, int curr_lane, double curr_velocity,
                    int &next_lane, double &next_velocity);
  void prepLaneChange(State curr_state, int curr_lane, double curr_velocity,
                      int &next_lane, double &next_velocity);
  void execLaneChange(State curr_state, int curr_lane, double curr_velocity,
                      int &next_lane, double &next_velocity);
  void getKinematics(int &next_lane, double &curr_velocity, double &next_velocity);
};

#endif // PATH_PLANNING_H
