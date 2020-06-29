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
    State &curr_state, int &curr_lane, double &curr_velocity,
    vector<double> &next_x_vals, vector<double> &next_y_vals) {

  // possible states
  State next_state = this->getNextState(curr_state, curr_lane);
  
  // cost per state
  map<string, float> costs = {};
  
  // minimum cost
  float min_cost = 1.0;

  // next state
  float cost;
  int next_lane = curr_lane;
  double next_velocity = curr_velocity;

  // for (unsigned int i = 0; i < next_states.size(); i++) {
  //   string next_state = next_states[i];
  //   vector<double> traj_x_vals;
  //   vector<double> traj_y_vals;
    
  //   // get next_velocity and next_lane for each state.
  //   if (next_state.compare("KL") == 0) {
  //     execLaneKeep(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
  //   } else if (next_state.compare("PLCL") == 0 || next_state.compare("PLCR") == 0) {
  //     prepLaneChange(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
  //   } else if (next_state.compare("LCL") == 0 || next_state.compare("LCR") == 0) {
  //     execLaneChange(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
  //   }

  //   // generate trajectory for possible states
  //   this->ego.generateTrajectoryXY(
  //       next_lane, next_velocity, traj_x_vals, traj_y_vals);

  //   // remember states' costs
  //   costs[next_state] = this->calculateCost(
  //       next_state, next_lane, next_velocity, traj_x_vals, traj_y_vals);
  // }


  // // pick up min cost state.
  // string t_next_state;
  // for (auto it = costs.begin(); it != costs.end(); it++) {
  //   if (min_cost > it->second) {
  //     min_cost = it->second;
  //     next_state = it->first;
  //   }
  // }

  // // get next_velocity and next_lane for each state.
  // if (next_state.compare("KL") == 0) {
  //   execLaneKeep(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
  // } else if (next_state.compare("PLCL") == 0 || next_state.compare("PLCR") == 0) {
  //   prepLaneChange(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
  // } else if (next_state.compare("LCL") == 0 || next_state.compare("LCR") == 0) {
  //   execLaneChange(next_state, curr_lane, curr_velocity, next_lane, next_velocity);
  // }

  switch(next_state) {
    case KL:
      generateBasePlanKL(next_state, curr_lane);
      break;
    case PLCL:
      generateBasePlanKL(next_state, curr_lane);
      break;
    case LCL:
      generateBasePlanKL(next_state, curr_lane);
      break;
    case PLCR:
      generateBasePlanKL(next_state, curr_lane);
      break;
    case LCR:
      generateBasePlanKL(next_state, curr_lane);
      break;
  }

  //realize next state
  // curr_state = next_state;
  // if (curr_velocity < 50) {
  //   next_velocity += 0.224;
  //   curr_velocity = next_velocity;
  // }

  // std::cout << " next_state: " << next_state
  //           << " lane: " << curr_lane << "->" << next_lane
  //           << " velo: " << curr_velocity << "->" << next_velocity
  //           << " speed: " << this->ego.getSpeed() << std::endl;
  // for (auto it = costs.begin(); it != costs.end(); it++) {
  //   std::cout << " state: " << it->first << ", cost: " << it->second;
  // }
  // std::cout << std::endl;

  // this->ego.generateTrajectoryXY(next_lane, next_velocity, next_x_vals, next_y_vals);
  std::cout << "-------" << std::endl;
  printVector(this->s_start);
  printVector(this->s_end);
  bool overwrite = false;
  this->ego.generateTrajectory(this->s_start, this->d_start, this->s_end, this->d_end,
                               this->T, next_x_vals, next_y_vals, overwrite);
}

/*
 * Private methods
 */ 


///////////////////////////////////////////////////////////////////////////
// Determine a next state with Finite State Machine.
///////////////////////////////////////////////////////////////////////////

State PathPlanner::getNextState(
    const State curr_state, const int curr_lane) {
  // Returns possible states based on current lane and state.
  State next_state;
  vector< vector<int> > spaces = getSurroundingVehicleInformation(curr_lane);

  std::cout << "curr_state: " << curr_state << std::endl;

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

  // Debug information
  std::cout << "next_state: " << next_state << std::endl;

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
  // Debug output.
  // std::cout << "----------------" << std::endl;
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
  if (this->ego.getSpeedMph() < kMinLaneChangeVelocity) {
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
        double ado_speed = this->vehicles[spaces[2][0]].getSpeedMph();
        double ego_speed = this->ego.getSpeedMph();
        if (ado_speed < ego_speed) {
          next_state = LCL;
        } else {
          next_state = KL;
        }
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
        double ado_speed = this->vehicles[spaces[2][2]].getSpeedMph();
        double ego_speed = this->ego.getSpeedMph();
        if (ado_speed < ego_speed) {
          next_state = LCR;
        } else {
          next_state = KL;
        }
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

///////////////////////////////////////////////////////////////////////////
// Set a goal for next trajectories.
///////////////////////////////////////////////////////////////////////////

void PathPlanner::generateBasePlanKL(State next_state, int curr_lane) {
  int next_lane = curr_lane;
  
  Vehicle vehicle;
  double min_range = 0;
  double max_range = 50;
  int vehicle_id = getVehicleInRange(next_lane, min_range, max_range, vehicle);
  
  this->T = 1;
  this->s_start = {this->ego.getS(), this->ego.getSpeedMps(), 0.0};
  this->d_start = {(double) 2 + (4 * curr_lane), 0.0, 0.0};
  this->d_end = {(double) 2 + (4 * next_lane), 0.0, 0.0};

  double target_mps;
  double target_acc;
  double target_dist;
  
  if (vehicle_id == 0) {
    // When no vehicle in front of the ego.
    // assume linear acceleration.
    double diff_to_max_velocity = kMaxVelMps - this->ego.getSpeedMps();
    double time_of_acceleration = min(this->T, diff_to_max_velocity / kMaxAccMps);
    double time_of_max_velocity = this->T - time_of_acceleration;
    double dist_of_acceleration = 0;
    double dist_of_max_velocity = 0;

    // calculate s_distance to max velocity;
    if (time_of_acceleration > 0) {
      target_acc = kMaxAccMps; 
      target_mps = this->ego.getSpeedMps() + (time_of_acceleration * target_acc);
      dist_of_acceleration = 
          ((this->ego.getSpeedMps() + target_mps) * time_of_acceleration) / 2.0;
      // Debug
      // std::cout << "dist_of_acceleration: " << dist_of_acceleration << std::endl;
    }
    // calculate s_distance with max velocity; 
    if (time_of_max_velocity > 0) {
      target_acc = 0.0;
      target_mps = kMaxVelMps;
      dist_of_max_velocity = target_mps * time_of_max_velocity;
      // Debug
      // std::cout << "dist_of_max_velocity: " << dist_of_max_velocity << std::endl;
    }
    target_dist = this->ego.getS() + dist_of_acceleration + dist_of_max_velocity;
    this->s_end = {target_dist, target_mps, target_acc};
  
  } else {
    // When a vehicle in front of the ego.
    double time_to_collision = 2.0;
    double buffer_dist_to_ado = vehicle.getSpeedMps() * time_to_collision;
    double predicted_s_of_ado = vehicle.getS() + (vehicle.getSpeedMps() * this->T);
    target_dist = predicted_s_of_ado - buffer_dist_to_ado;
    target_mps = vehicle.getSpeedMps();
    target_acc = 0.0;
    this->s_end = {target_dist, target_mps, target_acc}; 
  }
}

// void PathPlanner::execKL(State next_state, int curr_lane, double curr_velocity,
//                          int &next_lane, double &next_velocity) {
//   // lane_detection["KL"] = 0
//   next_lane = curr_lane;
//   double min_range = 0;
//   double max_range = 50;
//   double acceleration = 2.0; // mph
//   double t;
//   double s_goal;
//   double d_goal = 2 + (4 * curr_lane);

//   Vehicle vehicle;
//   int vehicle_id = getVehicleInRange(next_lane, min_range, max_range, vehicle);
//   if (vehicle_id == 0) {
//     t = (kMaxVelocity - curr_velocity) / acceleration;
//     s_goal = this->ego.getS() + ((curr_velocity + kMaxVelocity) * t / 2.0);
//   } else {
//     t = 6;
//     double s_buffer_to_ado = vehicle.getSpeedMps() * 1.5;
//     double s_goal_ado = vehicle.getS() + (t * vehicle.getSpeedMps());
//     s_goal = s_goal_ado - s_buffer_to_ado;
//   }
//   // getKinematics(curr_lane, curr_velocity, next_velocity);
// }

// void PathPlanner::prepLaneChange(
//     string curr_state, int curr_lane, double curr_velocity,
//     int &next_lane, double &next_velocity) {
//   // lane_detection["PLCR"] = 1
//   next_lane = curr_lane + lane_direction[curr_state];

//   // calculate new velocity.
//   double curr_lane_new_velocity;
//   double next_lane_new_velocity;
//   getKinematics(curr_lane, curr_velocity, curr_lane_new_velocity);
//   getKinematics(next_lane, curr_velocity, next_lane_new_velocity);
  
//   // choose faster lane
//   if (next_lane_new_velocity > curr_lane_new_velocity) {
//     next_velocity = next_lane_new_velocity;
//   } else {
//     next_lane = curr_lane;
//     next_velocity = curr_lane_new_velocity;
//   }
// }

// void PathPlanner::execLaneChange(
//     string curr_state, int curr_lane, double curr_velocity,
//     int &next_lane, double &next_velocity) {
//   // lane_detection["LCR"] = 1
//   next_lane = curr_lane + lane_direction[curr_state];
//   getKinematics(next_lane, curr_velocity, next_velocity);
// }

// void PathPlanner::getKinematics(
//     int &next_lane, double &curr_velocity, double &next_velocity) {
//   // Gets next timestep kinematics (position, velocity, acceleration) 
//   //   for a given lane. Tries to choose the maximum velocity and acceleration, 
//   //   given other vehicle positions and accel/velocity constraints.
  
//   double max_velocity = curr_velocity + kMaxAcceleration;
//   double min_velocity = curr_velocity - kMaxDeceleration;
//   double target_velocity;

//   Vehicle vehicle_ahead;
//   Vehicle vehicle_behind;

//   if (getVehicleAhead(next_lane, vehicle_ahead)) {
//     target_velocity = vehicle_ahead.getSpeedMph();
//     if (getVehicleBehind(next_lane, vehicle_behind)) {
//       target_velocity = max(target_velocity, vehicle_behind.getSpeedMph());
//     }
//     target_velocity = max(min_velocity, target_velocity);
//   } else {
//     target_velocity = kMaxVelocity;
//   }
//   next_velocity = min(target_velocity, max_velocity);
//   // avoid jerk.
//   // next_velocity = calculateNextVelocity(curr_velocity, target_velocity);
// }

// double PathPlanner::calculateNextVelocity(double curr_velocity, double target_velocity) {
//   double next_velocity;

//   if (curr_velocity == target_velocity) {
//     next_velocity = target_velocity;
//   } else {
//     double max_t = 10.0;
//     double a = 1.0;
//     double sign = 1.0;
//     double temp_t;
//     double temp_v;

//     if (curr_velocity > target_velocity) {
//       sign = -1.0;
//     }

//     temp_t = inverse_sigmoid(curr_velocity, max_t, a);
//     temp_v = sigmoid(temp_t + (0.06 * sign), max_t, a);

//     if ((sign * temp_v) < (sign * target_velocity)) {
//       next_velocity = temp_v;
//     } else {
//       next_velocity = target_velocity;
//     }
//   }
//   std::cout << " curr_v: " << curr_velocity
//             << " targ_v: " << target_velocity
//             << " next_v: " << next_velocity << std::endl;

//   return next_velocity;
// }

// double PathPlanner::sigmoid(double x, double t, double a) {
//   if (x == 0.0) {
//     return sigmoid(x+0.0001, t, a);
//   } 
  
//   if (x == 1.0) {
//     return sigmoid(x-0.0001, t, a);
//   }

//   x = x - (t / 2.0);
//   return kMaxVelocity * (1 / (1 + exp(-a * x)));
// }

// double PathPlanner::inverse_sigmoid(double y, double t, double a) {
//   if (y == 0) {
//     return inverse_sigmoid(y+0.01, t, a);
//   }

//   if (y == kMaxVelocity) {
//     return inverse_sigmoid(y-0.01, t, a);
//   }

//   y = y / kMaxVelocity;
//   return (1/a) * (log(y) - log(1-y)) + (t/2.0);
// }

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
      offset = kDelayCoef * v.getSpeedMps();

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


// int PathPlanner::getVehicleAhead(int target_lane, Vehicle &vehicle_ahead) {
//   int vehicle_id = 0;
//   int d_target_lane = 2 + 4 * target_lane;
//   double s_distance;
//   double min_s_distance = 100.0;
//   double offset;
//   for (auto it = this->vehicles.begin(); it != vehicles.end(); it++) {
//     Vehicle vehicle = it->second;

//     // Check if a vehicle is running on a target lane.
//     if (vehicle.getD() < (d_target_lane + 2) && vehicle.getD() > (d_target_lane - 2)) {
//       s_distance = vehicle.getS() - this->ego.getS();

//       // Needs to take care of sensor delay.
//       offset = 0.8 * vehicle.getSpeedMps();

//       if ((s_distance > -offset) && (s_distance < kVehicleSearchRange - offset)) {
//         if (s_distance < min_s_distance) {
//           min_s_distance = s_distance;
//           vehicle_id = it->first;
//           vehicle_ahead = vehicle;
//         } 
//       }
//     }
//   }
//   return vehicle_id;
// }

// bool PathPlanner::getVehicleBehind(int target_lane, Vehicle &vehicle_behind) {
//   // Returns true if another vehicle is behind.
//   bool ret = false;
//   int d_target_lane = 2 + 4 * target_lane;
//   double s_distance;

//   for (auto it = this->vehicles.begin(); it != vehicles.end(); it++) {
//     Vehicle vehicle = it->second;
//     // Check if an ADO vehicle is running on the same lane.
//     if (vehicle.getD() < (d_target_lane + 2) && vehicle.getD() > (d_target_lane - 2)) {
//       s_distance = this->ego.getS() - vehicle.getS();
//       if ((s_distance > 0) && (s_distance < kVehicleSearchRange)) {
//         ret = true;
//         vehicle_behind = vehicle;
//       }
//     }
//   }
//   return ret;
// }

// float PathPlanner::calculateCost(
//     const string &next_state, const int &next_lane, const double &next_velocity,
//     const vector<double> &trajectory_x, const vector<double> &trajectory_y) {
//   // This method calculate weighted sum of results from multiple cost functions.

//   vector<double> trajectory_s;
//   vector<double> trajectory_d;
//   this->ego.convertTrajectory(trajectory_x, trajectory_y, trajectory_s, trajectory_d);
//   float collision_cost = kCollisionWeight * calculateCollisionCost(trajectory_s,
//                                                                    trajectory_d);
//   float total_cost;
//   if (collision_cost > 0) {
//     // std::cout << "[Collision] "
//     //           << " next_state: " << next_state
//     //           << " next_lane: " << next_lane
//     //           << " next_velocity: " << next_velocity
//     //           << std::endl;
//     total_cost = 1.0;
//   } else {
//     float speed_cost = kSpeedWeight * calculateSpeedCost(next_velocity);
//     float state_cost = kStateWeight * calculateStateCost(next_state);
//     total_cost = (speed_cost + state_cost) / (kSpeedWeight + kStateWeight);
//   }

//   return total_cost;
// }

// float PathPlanner::calculateSpeedCost(const double &velocity) {
//   float cost;
//   double target_speed = kSpeedLimit - kBufferVelocity;

//   if (velocity < target_speed) {
//     cost = kStopCost * ((target_speed - velocity) / target_speed);
//   } else {
//     if (velocity < kSpeedLimit) {
//       cost = (velocity - target_speed) / kBufferVelocity;
//     } else {
//       cost = 1.0;
//     }
//   }
//   return cost;
// }

// float PathPlanner::calculateCollisionCost(const vector<double> &ego_traj_s,
//                                           const vector<double> &ego_traj_d) {
//   // Cost becomes 1.0 when ego collides with ado vehicles.                                          
//   // TODO(Yaboo) check distance per each point. 
//   int trajectory_length = ego_traj_s.size();

//   for (auto it = this->vehicles.begin(); it != vehicles.end(); it++) {
//     Vehicle vehicle = it->second;
//     double s_distance = abs(this->ego.getS() - vehicle.getS());
//     if (s_distance < kCollisionSearchRange) {
//       double point_distance;
//       vector<double> ado_traj_s;
//       vector<double> ado_traj_d;

//       int target_lane = floor(vehicle.getD() / 4);
//       double target_velocity = vehicle.getSpeedMph();

//       vehicle.generateTrajectorySD(target_lane, target_velocity, ado_traj_s, ado_traj_d);

//       for (int i = 0; i < trajectory_length; i++) {
//         if (abs(ado_traj_d[i] - ego_traj_d[i]) < 3.0) {
//           if (abs(ado_traj_s[i] - ego_traj_s[i]) < 20.0) {
//             return 1.0;
//           }
//         }
//       }
//     }
//   }
//   return 0.0;
// }

// float PathPlanner::calculateStateCost(const string &state) {
//   float cost;
//   if (state.compare("KL") == 0) {
//     cost = 0.9;
//   } else if ((state.compare("PLCL") == 0) || (state.compare("PLCR") == 0)) {
//     cost = 0.9;
//   } else if ((state.compare("LCL") == 0) || (state.compare("LCR") == 0)) {
//     cost = 0.8;
//   }
//   return cost;
// }

//
// Debug functions.
//

void PathPlanner::showInfo(int vehicle_id, Vehicle &vehicle){
  std::cout << "[" << vehicle_id << "]" << " "
            << "ado.s: " << vehicle.getS() << " "
            << "ado.d: " << vehicle.getD() << " "
            << "ado.v: " << vehicle.getSpeedMph() << " "
            << "ego.s: " << this->ego.getS() << " "
            << "ego.d: " << this->ego.getD() << " "
            << "ego.v: " << this->ego.getSpeedMph() << " "
            << "rv: " << vehicle.getSpeedMph() - this->ego.getSpeedMph() << " "
            << std::endl;
}