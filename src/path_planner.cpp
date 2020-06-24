#include <functional>
#include <string>
#include <vector>
#include <algorithm>
#include "helpers.h"
#include "spline.h"

#include "vehicle.h"
#include "path_planner.h"

using std::string;
using std::vector;
using std::min;
using std::max;

PathPlanner::PathPlanner(
    double x, double y, double s, double d, double yaw, double speed,
    const vector<double> &previous_path_x, const vector<double> &previous_path_y, 
    const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
    const vector<double> &map_waypoints_s,
    const vector< vector<double> > &sensor_fusion) {

  // Initialize an ego vehicle.
  this->ego = Vehicle(x, y, s, d, yaw, speed, previous_path_x, previous_path_y,
                      map_waypoints_x, map_waypoints_y, map_waypoints_s);

  // Initialize actors.
  for (int i = 0; i < sensor_fusion.size(); i++) {
    int vehicle_id = (int) sensor_fusion[i][0];
    Vehicle vehicle = Vehicle(sensor_fusion[i][1], sensor_fusion[i][2], //  x,  y
                              sensor_fusion[i][5], sensor_fusion[i][6], //  s,  d
                              sensor_fusion[i][3], sensor_fusion[i][4], // vx, vy
                              map_waypoints_x, map_waypoints_y, map_waypoints_s);
    this->vehicles[vehicle_id] = vehicle;
  }
  return;
}

void PathPlanner::getTrajectory(
    string &curr_state, int &curr_lane, double &curr_velocity,
    vector<double> &next_x_vals, vector<double> &next_y_vals) {

  // possible states
  vector<string> next_states = this->getPossibleStates(
      curr_state, curr_lane, curr_velocity);
  
  // cost per state
  map<string, float> costs = {};
  
  // minimum cost
  float min_cost = 1.0;

  // next state
  float cost;
  string next_state;
  int next_lane;
  double next_velocity;

  for (unsigned int i = 0; i < next_states.size(); i++) {
    string next_state = next_states[i];
    vector<double> traj_x_vals;
    vector<double> traj_y_vals;
    
    // get next_velocity and next_lane for each state.
    if (next_state.compare("KL") == 0) {
      execLaneKeep(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
    } else if (next_state.compare("PLCL") == 0 || next_state.compare("PLCR") == 0) {
      prepLaneChange(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
    } else if (next_state.compare("LCL") == 0 || next_state.compare("LCR") == 0) {
      execLaneChange(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
    }

    // generate trajectory for possible states
    this->ego.generateTrajectoryXY(
        next_lane, next_velocity, traj_x_vals, traj_y_vals);

    // remember states' costs
    costs[next_state] = this->calculateCost(
        next_state, next_lane, next_velocity, traj_x_vals, traj_y_vals);
  }


  // pick up min cost state.
  string t_next_state;
  for (auto it = costs.begin(); it != costs.end(); it++) {
    if (min_cost > it->second) {
      min_cost = it->second;
      next_state = it->first;
    }
  }

  // get next_velocity and next_lane for each state.
  if (next_state.compare("KL") == 0) {
    execLaneKeep(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
  } else if (next_state.compare("PLCL") == 0 || next_state.compare("PLCR") == 0) {
    prepLaneChange(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
  } else if (next_state.compare("LCL") == 0 || next_state.compare("LCR") == 0) {
    execLaneChange(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
  }

  //realize next state
  curr_state = next_state;
  curr_velocity = next_velocity;

  // std::cout << " next_state: " << next_state
  //           << " lane: " << curr_lane << "->" << next_lane
  //           << " velo: " << curr_velocity << "->" << next_velocity
  //           << " speed: " << this->ego.getSpeed() << std::endl;
  // for (auto it = costs.begin(); it != costs.end(); it++) {
  //   std::cout << " state: " << it->first << ", cost: " << it->second;
  // }
  // std::cout << std::endl;

  this->ego.generateTrajectoryXY(next_lane, next_velocity, next_x_vals, next_y_vals);
}

/*
 * Private methods
 */ 

vector<string> PathPlanner::getPossibleStates(
    string curr_state, int &curr_lane, double curr_velocity) {
  // Returns possible states based on current lane and state.
  vector<string> states;

  states.push_back("KL");

  if (curr_state.compare("KL") == 0) {
    if (curr_lane != 0 && curr_velocity > kMinLaneChangeVelocity) {
      states.push_back("PLCL");
    }
    if (curr_lane != kNumAvailableLanes -1) {
      states.push_back("PLCR");
    }
  }
  
  if (curr_state.compare("PLCL") == 0) {
    if (curr_lane != 0 && curr_velocity > kMinLaneChangeVelocity) {
      states.push_back("PLCL");
      states.push_back("LCL");
    } 
  }
  
  if (curr_state.compare("PLCR") == 0) {
    if (curr_lane != kNumAvailableLanes - 1 && curr_velocity > kMinLaneChangeVelocity) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  
  int temp_lane = getCurrentLane(curr_lane);
  
  if (curr_state.compare("LCL") == 0) {
    if (curr_lane == temp_lane) {
      states.pop_back();
      states.push_back("LCL");
    } else if (temp_lane < curr_lane) {
      curr_lane = temp_lane;
    } else {
      std::cout << "shouldn't happen." << std::endl;
    }
  }
  
  if (curr_state.compare("LCR") == 0) {
    if (curr_lane == temp_lane) {
      states.pop_back();
      states.push_back("LCR");
    } else if (curr_lane < temp_lane) {
      curr_lane = temp_lane;
    } else {
      std::cout << "shouldn't happen." << std::endl;
    }
  }

  return states;
}

int PathPlanner::getCurrentLane(int curr_lane){
  int temp_lane;

  if ((this->ego.getD() >= 1.5) && (this->ego.getD() <= 2.5)) {
    temp_lane = 0;
  } else if ((this->ego.getD() >= 5.5) && (this->ego.getD() <= 6.5)) {
    temp_lane = 1;
  } else if ((this->ego.getD() >= 9.5) && (this->ego.getD() <= 10.5)) {
    temp_lane = 2;
  } else {
    temp_lane = curr_lane;
  }
  return temp_lane;
}

void PathPlanner::execLaneKeep(
    string curr_state, int curr_lane, double curr_velocity,
    int &next_lane, double &next_velocity) {
  // lane_detection["KL"] = 0
  next_lane = curr_lane + lane_direction[curr_state];
  getKinematics(curr_lane, curr_velocity, next_velocity);
}

void PathPlanner::prepLaneChange(
    string curr_state, int curr_lane, double curr_velocity,
    int &next_lane, double &next_velocity) {
  // lane_detection["PLCR"] = 1
  next_lane = curr_lane + lane_direction[curr_state];

  // calculate new velocity.
  double curr_lane_new_velocity;
  double next_lane_new_velocity;
  getKinematics(curr_lane, curr_velocity, curr_lane_new_velocity);
  getKinematics(next_lane, curr_velocity, next_lane_new_velocity);
  
  // choose faster lane
  if (next_lane_new_velocity > curr_lane_new_velocity) {
    next_velocity = next_lane_new_velocity;
  } else {
    next_lane = curr_lane;
    next_velocity = curr_lane_new_velocity;
  }
}

void PathPlanner::execLaneChange(
    string curr_state, int curr_lane, double curr_velocity,
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
  
  double max_velocity = curr_velocity + kMaxAcceleration;
  double min_velocity = curr_velocity - kMaxDeceleration;
  double target_velocity;

  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (getVehicleAhead(next_lane, vehicle_ahead)) {
    target_velocity = vehicle_ahead.getSpeed();
    if (getVehicleBehind(next_lane, vehicle_behind)) {
      target_velocity = max(target_velocity, vehicle_behind.getSpeed());
    }
    target_velocity = max(min_velocity, target_velocity);
  } else {
    target_velocity = kMaxVelocity;
  }
  next_velocity = min(target_velocity, max_velocity);
  // avoid jerk.
  // next_velocity = calculateNextVelocity(curr_velocity, target_velocity);
}

double PathPlanner::calculateNextVelocity(double curr_velocity, double target_velocity) {
  double next_velocity;

  if (curr_velocity == target_velocity) {
    next_velocity = target_velocity;
  } else {
    double max_t = 10.0;
    double a = 1.0;
    double sign = 1.0;
    double temp_t;
    double temp_v;

    if (curr_velocity > target_velocity) {
      sign = -1.0;
    }

    temp_t = inverse_sigmoid(curr_velocity, max_t, a);
    temp_v = sigmoid(temp_t + (0.06 * sign), max_t, a);

    if ((sign * temp_v) < (sign * target_velocity)) {
      next_velocity = temp_v;
    } else {
      next_velocity = target_velocity;
    }
  }
  std::cout << " curr_v: " << curr_velocity
            << " targ_v: " << target_velocity
            << " next_v: " << next_velocity << std::endl;

  return next_velocity;
}

double PathPlanner::sigmoid(double x, double t, double a) {
  if (x == 0.0) {
    return sigmoid(x+0.0001, t, a);
  } 
  
  if (x == 1.0) {
    return sigmoid(x-0.0001, t, a);
  }

  x = x - (t / 2.0);
  return kMaxVelocity * (1 / (1 + exp(-a * x)));
}

double PathPlanner::inverse_sigmoid(double y, double t, double a) {
  if (y == 0) {
    return inverse_sigmoid(y+0.01, t, a);
  }

  if (y == kMaxVelocity) {
    return inverse_sigmoid(y-0.01, t, a);
  }

  y = y / kMaxVelocity;
  return (1/a) * (log(y) - log(1-y)) + (t/2.0);
}

bool PathPlanner::getVehicleAhead(int target_lane, Vehicle &vehicle_ahead) {
  bool ret = false;
  int d_target_lane = 2 + 4 * target_lane;
  double s_distance;

  for (auto it = this->vehicles.begin(); it != vehicles.end(); it++) {
    Vehicle vehicle = it->second;

    // Check if a vehicle is running on a target lane.
    if (vehicle.getD() < (d_target_lane + 2) && vehicle.getD() > (d_target_lane - 2)) {
      s_distance = vehicle.getS() - this->ego.getS();
      if ((s_distance > 0) && (s_distance < kVehicleSearchRange)) {
        ret = true;
        vehicle_ahead = vehicle;
      }
    }
  }
  return ret;
}

bool PathPlanner::getVehicleBehind(int target_lane, Vehicle &vehicle_behind) {
  // Returns true if another vehicle is behind.
  bool ret = false;
  int d_target_lane = 2 + 4 * target_lane;
  double s_distance;

  for (auto it = this->vehicles.begin(); it != vehicles.end(); it++) {
    Vehicle vehicle = it->second;
    // Check if an ADO vehicle is running on the same lane.
    if (vehicle.getD() < (d_target_lane + 2) && vehicle.getD() > (d_target_lane - 2)) {
      s_distance = this->ego.getS() - vehicle.getS();
      if ((s_distance > 0) && (s_distance < kVehicleSearchRange)) {
        ret = true;
        vehicle_behind = vehicle;
      }
    }
  }
  return ret;
}

float PathPlanner::calculateCost(
    const string &next_state, const int &next_lane, const double &next_velocity,
    const vector<double> &trajectory_x, const vector<double> &trajectory_y) {
  // This method calculate weighted sum of results from multiple cost functions.

  vector<double> trajectory_s;
  vector<double> trajectory_d;
  this->ego.convertTrajectory(trajectory_x, trajectory_y, trajectory_s, trajectory_d);
  float collision_cost = kCollisionWeight * calculateCollisionCost(trajectory_s,
                                                                   trajectory_d);
  float total_cost;
  if (collision_cost > 0) {
    std::cout << "[Collision] "
              << " next_state: " << next_state
              << " next_lane: " << next_lane
              << " next_velocity: " << next_velocity
              << std::endl;
    total_cost = 1.0;
  } else {
    float speed_cost = kSpeedWeight * calculateSpeedCost(next_velocity);
    float state_cost = kStateWeight * calculateStateCost(next_state);
    total_cost = (speed_cost + state_cost) / (kSpeedWeight + kStateWeight);
  }

  return total_cost;
}

float PathPlanner::calculateSpeedCost(const double &velocity) {
  float cost;
  double target_speed = kSpeedLimit - kBufferVelocity;

  if (velocity < target_speed) {
    cost = kStopCost * ((target_speed - velocity) / target_speed);
  } else {
    if (velocity < kSpeedLimit) {
      cost = (velocity - target_speed) / kBufferVelocity;
    } else {
      cost = 1.0;
    }
  }
  return cost;
}

float PathPlanner::calculateCollisionCost(const vector<double> &ego_traj_s,
                                          const vector<double> &ego_traj_d) {
  // Cost becomes 1.0 when ego collides with ado vehicles.                                          
  // TODO(Yaboo) check distance per each point. 
  int trajectory_length = ego_traj_s.size();

  for (auto it = this->vehicles.begin(); it != vehicles.end(); it++) {
    Vehicle vehicle = it->second;
    double s_distance = abs(this->ego.getS() - vehicle.getS());
    if (s_distance < kCollisionSearchRange) {
      double point_distance;
      vector<double> ado_traj_s;
      vector<double> ado_traj_d;

      int target_lane = floor(vehicle.getD() / 4);
      double target_velocity = vehicle.getSpeed();

      vehicle.generateTrajectorySD(target_lane, target_velocity, ado_traj_s, ado_traj_d);

      for (int i = 0; i < trajectory_length; i++) {
        if (abs(ado_traj_d[i] - ego_traj_d[i]) < 3.0) {
          if (abs(ado_traj_s[i] - ego_traj_s[i]) < 20.0) {
            return 1.0;
          }
        }
      }
    }
  }
  return 0.0;
}

float PathPlanner::calculateStateCost(const string &state) {
  float cost;
  if (state.compare("KL") == 0) {
    cost = 0.9;
  } else if ((state.compare("PLCL") == 0) || (state.compare("PLCR") == 0)) {
    cost = 0.9;
  } else if ((state.compare("LCL") == 0) || (state.compare("LCR") == 0)) {
    cost = 0.8;
  }
  return cost;
}
