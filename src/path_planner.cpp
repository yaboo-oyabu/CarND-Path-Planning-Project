#include <functional>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include "helpers.h"
#include "spline.h"

#include "vehicle.h"
#include "path_planner.h"

using std::string;
using std::vector;
using std::min;
using std::max;


PathPlanner::PathPlanner(
    double x, double y, double s, double d, double yaw, double vel,
    const vector<double> &previous_path_x, const vector<double> &previous_path_y, 
    const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
    const vector<double> &map_waypoints_s,
    const vector< vector<double> > &sensor_fusion) {

  // Initialize an ego vehicle.
  this->ego = Vehicle(x, y, s, d, yaw, vel, previous_path_x, previous_path_y,
                      map_waypoints_x, map_waypoints_y, map_waypoints_s);

  // Initialize actors.
  for (int i = 0; i < sensor_fusion.size(); i++) {
    int vehicle_id = (int) sensor_fusion[i][0] + 1;
    Vehicle vehicle = Vehicle(sensor_fusion[i][1], sensor_fusion[i][2], //  x,  y
                              sensor_fusion[i][5], sensor_fusion[i][6], //  s,  d
                              sensor_fusion[i][3], sensor_fusion[i][4], // vx, vy
                              map_waypoints_x, map_waypoints_y, map_waypoints_s);
    this->vehicles[vehicle_id] = vehicle;
  }
  return;
}

void PathPlanner::generateTrajectory(
    State &curr_state, int &curr_lane, double &curr_vel,
    vector<double> &next_x_vals, vector<double> &next_y_vals) {

  State next_state = this->getNextState(curr_state, curr_lane);

  int next_lane;
  double next_vel;

  switch(next_state) {
    case KL:
      execLaneKeep(next_state, curr_lane, curr_vel, next_lane, next_vel);
      break;
    case PLCL:
      prepLaneChange(next_state, curr_lane, curr_vel, next_lane, next_vel);
      break;
    case LCL:
      execLaneChange(next_state, curr_lane, curr_vel, next_lane, next_vel);
      break;
    case PLCR:
      prepLaneChange(next_state, curr_lane, curr_vel, next_lane, next_vel);
      break;
    case LCR:
      execLaneChange(next_state, curr_lane, curr_vel, next_lane, next_vel);
      break;
  }

  this->ego.generateTrajectory(next_lane, next_vel, next_x_vals, next_y_vals);
  curr_state = next_state;
  curr_lane = next_lane;
  curr_vel = next_vel;
}

State PathPlanner::getNextState(
    const State curr_state, const int curr_lane) {
  // Returns possible states based on current lane and state.
  State next_state;
  vector< vector<int> > spaces = getSurroundingVehicleInformation(curr_lane);

  switch(curr_state) {
    case KL:
      next_state = getNextStateKL(spaces);
      break;
    case PLCL:
      next_state = getNextStatePLCL(spaces);
      break;
    case LCL:
      next_state = getNextStateLCL(spaces, curr_lane);
      break;
    case PLCR:
      next_state = getNextStatePLCR(spaces);
      break;
    case LCR:
      next_state = getNextStateLCR(spaces, curr_lane);
      break;
  }

  return next_state;
}

vector< vector<int> > PathPlanner::getSurroundingVehicleInformation(int curr_lane) {
  Vehicle vehicle;
  double interval = 30.0;
  vector<vector<int>> spaces(3, vector<int>(3, 0));
  for (int i = 0; i < 3; i++) {
    double min_range = interval * (1 - i);
    double max_range = min_range + interval;
    for (int j = 0; j < 3; j++) {
      int lane = curr_lane + (j - 1);
      if (lane < 0 || lane >= kNumAvailableLanes) {
        spaces[i][j] = -1;
      } else {
        spaces[i][j] = getVehicleInRange(lane, min_range, max_range, vehicle);
      }
    }
  }

  // For Debug information;
  // for (int i = 0; i < spaces.size(); i++){
  //   for (int j = 0; j < spaces[0].size(); j++) {
  //     std::cout << std::setw(3) << spaces[i][j];
  //   }
  //   std::cout << std::endl;
  // }
  return spaces;
} 

State PathPlanner::getNextStateKL(const vector< vector<int> > &spaces) {
  State next_state;
  if (this->ego.getVelMph() < kMinVelMphLaneChange) {
    next_state = KL;
    return next_state;
  }

  // No vehicles on the first and the second row of the current lane.
  if (spaces[0][1] == 0 && spaces[1][1] == 0) {
    next_state = KL;
  } else {
    // No vehicles on the first and the second row of the left lane.
    if (spaces[0][0] == 0 && spaces[1][0] == 0) {
      next_state = PLCL;
    // No vehicles on the first and the second row of the right lane.
    } else if (spaces[0][2] == 0 && spaces[1][2] == 0) {
      next_state = PLCR;
    // Falls into the default KL state.
    } else {
      next_state = KL;
    }
  }
  return next_state;
}

State PathPlanner::getNextStatePLCL(const vector< vector<int> > &spaces) {
  State next_state;
  // No vehicles on the first and the second row of the current lane.
  if (spaces[0][1] == 0 && spaces[1][1] == 0) {
    next_state = KL;
  } else {
    // Vehicles on the first or the second row of the left lane.
    if (!(spaces[0][0] == 0 && spaces[1][0] == 0)) {
      next_state = KL;
    } else {
      // No Vehicle from the first to the third row of the left lane.
      if (spaces[2][0] == 0) {
        next_state = LCL;
      } else {
        next_state = KL;
      }
    }
  }
  return next_state;
}

State PathPlanner::getNextStatePLCR(const vector< vector<int> > &spaces) {
  State next_state;
  // No vehicles on the first and the second row of the current lane.
  if (spaces[0][1] == 0 && spaces[1][1] == 0) {
    next_state = KL;
  } else {
    // Vehicles on the first or the second row of the left lane.
    if (!(spaces[0][2] == 0 && spaces[1][2] == 0)) {
      next_state = KL;
    } else {
      // No Vehicle from the first to the third row of the left lane.
      if (spaces[2][2] == 0) {
        next_state = LCR;
      } else {
        next_state = KL;
      }
    }
  }
  return next_state;
}

State PathPlanner::getNextStateLCL(const vector< vector<int> > &spaces, const int curr_lane) {
  State next_state;
  // Vehicles on the first or the second row of the left lane.
  if (!(spaces[0][0] == 0 && spaces[1][0] == 0)) {
    next_state = PLCL;
  } else {
    int next_lane = curr_lane - 1;
    if (isLaneChangeCompleted(next_lane)) {
      next_state = KL;
    } else {
      next_state = LCL;
    }
  }
  return next_state;
}

State PathPlanner::getNextStateLCR(const vector< vector<int> > &spaces, const int curr_lane) {
  State next_state;
  // Vehicles on the first or the second row of the left lane.
  if (!(spaces[0][2] == 0 && spaces[1][2] == 0)) {
    next_state = PLCR;
  } else {
    int next_lane = curr_lane + 1;
    if (isLaneChangeCompleted(next_lane)) {
      next_state = KL;
    } else {
      next_state = LCR;
    }
  }
  return next_state;
}

bool PathPlanner::isLaneChangeCompleted(int next_lane){
  bool completed = false;
  double next_lane_center = 2 + 4 * (next_lane);
  double lower_bound = next_lane_center - 0.5;
  double upper_bound = next_lane_center + 0.5;

  if ((lower_bound < this->ego.getD()) && (this->ego.getD() < upper_bound)) {
    completed = true;
  }
  return completed;
}

void PathPlanner::execLaneKeep(
    State curr_state, int curr_lane, double curr_velocity,
    int &next_lane, double &next_velocity) {
  // lane_detection["KL"] = 0
  next_lane = curr_lane + lane_direction[curr_state];
  getKinematics(curr_lane, curr_velocity, next_velocity);
}

void PathPlanner::prepLaneChange(
    State curr_state, int curr_lane, double curr_velocity,
    int &next_lane, double &next_velocity) {
  // lane_detection["PLCR"] = 1
  next_lane = curr_lane;

  // calculate new velocity.
  double curr_lane_new_velocity;
  double next_lane_new_velocity;
  getKinematics(curr_lane, curr_velocity, curr_lane_new_velocity);
  getKinematics(next_lane, curr_velocity, next_lane_new_velocity);
  
  // choose faster lane
  if (next_lane_new_velocity > curr_lane_new_velocity) {
    next_velocity = next_lane_new_velocity;
  } else {
    next_velocity = curr_lane_new_velocity;
  }
}

void PathPlanner::execLaneChange(
    State curr_state, int curr_lane, double curr_velocity,
    int &next_lane, double &next_velocity) {
  // lane_detection["LCR"] = 1
  next_lane = curr_lane + lane_direction[curr_state];
  getKinematics(next_lane, curr_velocity, next_velocity);
}

void PathPlanner::getKinematics(
    int &next_lane, double &curr_velocity, double &next_velocity) {
  // Gets next timestep kinematics (position, velocity, acceleration) 
  //   for a given lane. Tries to choose the maximum velocity and acceleration, 
  //   given other vehicle positions and accel/velocity constraints.
  
  double max_velocity = curr_velocity + kMaxAccMps;
  double min_velocity = curr_velocity - kMaxDccMps;

  Vehicle ado;
  double min_range = 0;
  double max_range = 50;
  int ado_id = getVehicleInRange(next_lane, min_range, max_range, ado);

  if (ado_id == 0) {
    next_velocity = min(max_velocity, kMaxVelMps);
  } else {
    double ado_velocity = vehicles[ado_id].getVel();
    double relative_velocity = ado_velocity - curr_velocity;
    if (relative_velocity > 0) {
      next_velocity = min(max_velocity, kMaxVelMps);
    } else {
      next_velocity = max(ado_velocity, min_velocity);
    }
  }
}

int PathPlanner::getVehicleInRange(
    int lane, double min_range, double max_range, Vehicle &vehicle) {

  int vehicle_id = 0;
  int d_lane = 2 + 4 * lane;
  
  double s_dist;
  double offset;
  double min_s_dist = max(abs(min_range), abs(max_range));

  for (auto it = this->vehicles.begin(); it != vehicles.end(); it++) {
    Vehicle v = it->second;

    if (v.getD() < (d_lane + 2) && v.getD() > (d_lane - 2)) {
      s_dist = v.getS() - this->ego.getS();
      offset = kDelayCoef * v.getVel();

      if ((min_range - offset < s_dist) && (s_dist < max_range - offset)) {
        if (abs(s_dist) < min_s_dist) {
          min_s_dist = abs(s_dist);
          vehicle_id = it->first;
          vehicle = v;
        }
      }
    }
  }
  return vehicle_id;
}
