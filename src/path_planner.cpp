#include <functional>
#include <string>
#include <vector>
#include "helpers.h"
#include "spline.h"

#include "vehicle.h"
#include "path_planner.h"

using std::string;
using std::vector;

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
    string &current_state, int &current_lane, double &current_velocity,
    vector<double> &next_x_vals, vector<double> &next_y_vals) {

  // possible states
  vector<string> states = this->getPossibleStates(current_lane, current_state);

  printVector(states);

  // cost per state
  map<string, float> costs = {};

  float min_cost = 100.0;
  string next_state = "";
  int next_lane;
  double next_velocity;

  bool too_close = false;
  too_close = this->isLeadVehicleTooClose(current_lane);
  std::cout << "too_close: " << too_close << std::endl;

  for (unsigned int i = 0; i < states.size(); i++) {
    float cost;
    int target_lane;
    double target_velocity;
    vector<double> next_px;
    vector<double> next_py;

    // rule.
    string state = states[i];

    if (state.compare("KL") == 0) {
      target_lane = current_lane;
      target_velocity = current_velocity;
      // set the same velocity as the lead vehicle.
      if (too_close) {
        target_velocity = current_velocity - .224;
      } else if (current_velocity < 49.5) {
        target_velocity = current_velocity + .224;
      }
    } else if (state.compare("LCL") == 0) {
      target_lane = current_lane - 1;
      target_velocity = current_velocity; // keep current velocity.
    } else if (state.compare("LCR") == 0) {
      target_lane = current_lane + 1;
      target_velocity = current_velocity; // keep current velocity.
    } else if (state.compare("PLCL") == 0) {
      target_lane = current_lane;
      target_velocity = current_velocity; // set the same velocity as the left lane vehicle.
    } else if (state.compare("PLCR") == 0) {
      target_lane = current_lane;
      target_velocity = current_velocity; // set the same velocity as the right lane vehicle.
    }

    // generate trajectory for possible states
    this->ego.generateTrajectory(target_lane, target_velocity, next_px, next_py);

    // remember states' costs
    cost = this->calculateCost(target_lane, target_velocity, next_px, next_py);

    std::cout << "# " << state << ", " << target_lane << ", " << target_velocity << ", " << cost << std::endl;

    // find minimum cost and it's state.
    if (cost < min_cost) {
      min_cost = cost;
      next_state = states[i];
      next_lane = target_lane;
      next_velocity = target_velocity;
    }
  }
  std::cout << "## " << next_state << std::endl;

  current_lane = next_lane;
  current_velocity = next_velocity;
  this->ego.generateTrajectory(current_lane, current_velocity, next_x_vals, next_y_vals);
}


/*
 * Private methods
 */ 

bool PathPlanner::isLeadVehicleTooClose(int current_lane) {
  bool ret = false;
  int d_current_lane = 2 + 4 * current_lane;

  for (auto it = this->ados.begin(); it != ados.end(); it++) {
    Ado ado = it->second;
    // Check if an ADO vehicle is running on the same lane
    if (ado.next_d < (d_current_lane + 2) && ado.next_d > (d_current_lane - 2)) {
      // Distance between ado and ego is less than kDistanceThreshold
      if ((ado.next_s > this->ego.s) && ((ado.next_s - this->ego.s) < kDistanceThreshold)) {
        ret = true;
        break;
      }
    }
  }
  return ret;
}

vector<string> PathPlanner::getPossibleStates(int current_lane, string state) {
  // Returns possible states based on current lane and state.
  vector<string> states;

  states.push_back("KL");

  if (state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (current_lane != kNumAvailableLanes - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    } 
  } else if (state.compare("PLCR") == 0) {
    if (current_lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  return states;
}


float PathPlanner::calculateCollisionCost(const vector<double> &points_x,
                                          const vector<double> &points_y) {
  // Cost becomes 1.0 when ego collides with ado vehicles.                                          
  float cost = 0.0;
  int trajectory_length = points_x.size();
  double x = points_x[trajectory_length-1];
  double y = points_y[trajectory_length-1];
  for (auto it = this->ados.begin(); it != ados.end(); it++) {
    Ado ado = it->second;
    if (distance(x, y, ado.next_x, ado.next_y) < kDistanceThreshold) {
      cost = 1.0;
    }
  }
  return cost;
}

float PathPlanner::calculateInefficiencyCost(const vector<double> &points_x,
                                             const vector<double> &points_y) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.                                       
  float cost = 0.0;
  return cost;
}

float PathPlanner::calculateCost(const int &target_lane,
                                 const double &target_velocity,
                                 const vector<double> &px,
                                 const vector<double> &py) {
  // This method calculate weighted sum of results from multiple cost functions.                                 
  float total_cost = 0.0;
  total_cost += kCollisionWeight * calculateCollisionCost(px, py);
  total_cost += kInefficiencyWeight * calculateInefficiencyCost(px, py);
  return total_cost;
}