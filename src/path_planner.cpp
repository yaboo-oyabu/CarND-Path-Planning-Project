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

  this->ego = Ego(x, y, s, d, yaw, speed, previous_path_x, previous_path_y,
                  map_waypoints_x, map_waypoints_y, map_waypoints_s);

  int num_steps = 50; //previous_path_x.size();

  for (int i = 0; i < sensor_fusion.size(); i++) {
    // The vx, vy values can be useful for predicting where the cars will be in the future.
    // For instance, if you were to assume that the tracked car kept moving along the road,
    // then its future predicted Frenet s value will be its current s value 
    // plus its (transformed) total velocity (m/s) multiplied by the time elapsed into the
    // future (s).
    int ado_id = (int) sensor_fusion[i][0];
    Ado ado = Ado(ado_id, // id
                  sensor_fusion[i][1], sensor_fusion[i][2], //  x,  y
                  sensor_fusion[i][3], sensor_fusion[i][4], // vx, vy
                  sensor_fusion[i][5], sensor_fusion[i][6], //  s,  d
                  map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ado.predictPosition(num_steps);
    this->ados[ado_id] = ado;
  }
  return;
}


////////
// Main method
////////

void PathPlanner::getTrajectory(
    string &curr_state, int &curr_lane, double &curr_velocity,
    vector<double> &next_x_vals, vector<double> &next_y_vals) {

  // possible states
  vector<string> states = this->getPossibleStates(
      curr_state, curr_lane, curr_velocity);
  
  // cost per state
  map<string, float> costs = {};
  
  // minimum cost
  float min_cost = 100.0;

  // next state
  float cost;
  string next_state;
  int next_lane;
  double next_velocity;
  vector<double> traj_x_vals;
  vector<double> traj_y_vals;

  for (unsigned int i = 0; i < states.size(); i++) {
    string state = states[i];

    // get next_velocity and next_lane for each state.
    if (state.compare("KL") == 0) {
      execLaneKeep(state, curr_lane, curr_velocity, next_lane, next_velocity);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
      prepLaneChange(state, curr_lane, curr_velocity, next_lane, next_velocity);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
      execLaneChange(state, curr_lane, curr_velocity, next_lane, next_velocity);
    }

    // generate trajectory for possible states
    this->ego.generateTrajectory(
        next_lane, next_velocity, traj_x_vals, traj_y_vals);

    // remember states' costs
    costs[state] = this->calculateCost(
        state, next_lane, next_velocity, traj_x_vals, traj_y_vals);
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

  std::cout << " next_state: " << next_state
            << " lane: " << curr_lane << "->" << next_lane
            << " velo: " << curr_velocity << "->" << next_velocity
            << std::endl;
  for (auto it = costs.begin(); it != costs.end(); it++) {
    std::cout << " state: " << it->first << ", cost: " << it->second;
  }
  std::cout << std::endl;

  this->ego.generateTrajectory(next_lane, next_velocity, next_x_vals, next_y_vals);
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
    if (curr_lane != kNumAvailableLanes - 1) {
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

  if ((this->ego.d >= 1.5) && (this->ego.d <= 2.5)) {
    temp_lane = 0;
  } else if ((this->ego.d >= 5.5) && (this->ego.d <= 6.5)) {
    temp_lane = 1;
  } else if ((this->ego.d >= 9.5) && (this->ego.d <= 10.5)) {
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

  Ado ado_ahead;
  Ado ado_behind;

  if (getVehicleAhead(next_lane, ado_ahead)) {
    if (getVehicleBehind(next_lane, ado_behind)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      next_velocity = (ado_ahead.v + ado_behind.v) / 2.0;
      next_velocity = min(next_velocity, max_velocity);
    } else {
      next_velocity = ado_ahead.v;
      next_velocity = max(ado_ahead.v, min_velocity);
    }
  } else {
    next_velocity = min(kMaxVelocity, max_velocity);
  }
}

bool PathPlanner::getVehicleAhead(int target_lane, Ado &ado_ahead) {
  bool ret = false;
  int d_target_lane = 2 + 4 * target_lane;
  double min_s_distance = kDistanceThreshold;
  double s_distance = kDistanceThreshold;

  for (auto it = this->ados.begin(); it != ados.end(); it++) {
    Ado ado = it->second;
    // Check if an ADO vehicle is running on the same lane
    if (ado.next_d < (d_target_lane + 2) && ado.next_d > (d_target_lane - 2)) {
      // Distance between ado and ego is less than kDistanceThreshold
      s_distance = ado.next_s - this->ego.s;
      if ((s_distance > 0) && (s_distance < min_s_distance)) {
        ret = true;
        ado_ahead = ado;
      }
    }
  }
  return ret;
}

bool PathPlanner::getVehicleBehind(int target_lane, Ado &ado_behind) {
  // Returns true if another vehicle is behind.
  bool ret = false;
  int d_target_lane = 2 + 4 * target_lane;
  double min_s_distance = kDistanceThreshold;
  double s_distance = kDistanceThreshold;

  for (auto it = this->ados.begin(); it != ados.end(); it++) {
    Ado ado = it->second;
    // Check if an ADO vehicle is running on the same lane.
    if (ado.next_d < (d_target_lane + 2) && ado.next_d > (d_target_lane - 2)) {
      // Distance between an ado and an ego is less than kDistanceThreshold
      s_distance = this->ego.s - ado.next_s;
      if ((s_distance > 0) && (s_distance < min_s_distance)) {
        ret = true;
        ado_behind = ado;
      }
    }
  }
  return ret;
}

float PathPlanner::calculateCost(
    const string &curr_state, const int &next_lane, const double &next_velocity,
    const vector<double> &px, const vector<double> &py) {
  // This method calculate weighted sum of results from multiple cost functions.                                 
  float speed_cost = kSpeedWeight * calculateSpeedCost(next_velocity);
  float collision_cost = kCollisionWeight * calculateCollisionCost(px, py);
  float state_cost = kStateWeight * calculateStateCost(curr_state);
  float total_cost = (speed_cost + collision_cost + state_cost) / kTotalWeight;
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

float PathPlanner::calculateCollisionCost(const vector<double> &points_x,
                                          const vector<double> &points_y) {
  // Cost becomes 1.0 when ego collides with ado vehicles.                                          
  // TODO(Yaboo) check distance per each point. 
  float cost = 0.0;
  int trajectory_length = points_x.size();
  double x_s = points_x[0];
  double y_s = points_x[0];
  double x_e = points_x[trajectory_length-1];
  double y_e = points_y[trajectory_length-1];

  for (auto it = this->ados.begin(); it != ados.end(); it++) {
    Ado ado = it->second;
    if ((distance(x_s, y_s, ado.x, ado.y) < kDistanceThreshold) ||
        (distance(x_e, y_e, ado.next_x, ado.next_y) < kDistanceThreshold)) {
      cost = 1.0;
    }
  }
  return cost;
}

float PathPlanner::calculateStateCost(const string &state) {
  float cost;
  if (state.compare("KL") == 0) {
    cost = 1.0;
  } else if ((state.compare("PLCL") == 0) || (state.compare("PLCR") == 0)) {
    cost = 1.0;
  } else if ((state.compare("LCL") == 0) || (state.compare("LCR") == 0)) {
    cost = 0.9;
  }
  return cost;
}
