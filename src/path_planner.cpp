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

  for (int i = 0; i < sensor_fusion.size(); i++) {
    // The vx, vy values can be useful for predicting where the cars will be in the future.
    // For instance, if you were to assume that the tracked car kept moving along the road,
    // then its future predicted Frenet s value will be its current s value 
    // plus its (transformed) total velocity (m/s) multiplied by the time elapsed into the
    // future (s).
    int id_ = (int) sensor_fusion[i][0];
    double x_ = sensor_fusion[i][1];
    double y_ = sensor_fusion[i][2];
    double vx_ = sensor_fusion[i][3];
    double vy_ = sensor_fusion[i][4];
    double s_ = sensor_fusion[i][5];
    double d_ = sensor_fusion[i][6];
    Ado ado = Ado(id_, x_, y_, s_, d_, vx_, vy_);
    this->ados[id_] = ado;
  }
  return;
}

bool PathPlanner::isLeadVehicleTooClose(int current_lane) {
  bool ret = false;
  int d_current_lane = 2 + 4 * current_lane;

  map<int, Ado>::iterator it = this->ados.begin();

  while (it != ados.end()) {
    Ado ado = it->second;
    if (ado.d < (d_current_lane + 2) && ado.d > (d_current_lane - 2)) {
      ado.s += ((double) this->ego.prev_size * kSimInterval * ado.v);
      if ((ado.s > this->ego.s) && ((ado.s - this->ego.s) < kDistanceThreshold)) {
        ret = true;
      }
    }
    it++;
  }
  return ret;
}

vector<string> PathPlanner::getAvailableStates(int current_lane, string state) {
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

void PathPlanner::getTrajectory(
    int &current_lane, string &current_state, double &current_velocity,
    vector<double> &next_x_vals, vector<double> &next_y_vals) {

  bool too_close = false;
  too_close = this->isLeadVehicleTooClose(current_lane);


  if (too_close) {
    vector<string> states = this->getAvailableStates(current_lane, current_state);
    if (current_lane > 0) {
      current_lane = 0;
    }
  }

  if (too_close) {
    current_velocity -= .224;
  } else if (current_velocity < 49.5) {
    current_velocity += .224;
  }

  this->ego.generateTrajectory(current_lane, current_velocity, next_x_vals, next_y_vals);
}