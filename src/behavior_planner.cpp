#include <string>
#include <vector>
#include "behavior_planner.h"

using std::string;
using std::vector;

BehaviorPlanner::BehaviorPlanner(
    const vector< vector<double> > &sensor_fusion) {
  for (int i = 0; i < sensor_fusion.size(); i++) {
    actors.push_back(Actor(sensor_fusion[i]));
  }
  return;
}

bool BehaviorPlanner::isLeadVehicleTooClose(
    double ego_s, double ego_d, int ego_prev_path_size) {
  bool ret = false;

  for (int i = 0; i < actors.size(); i++) {
    Actor actor = actors[i];
    double actor_d = actor.getD();
    double actor_s = actor.getS();
    double actor_vel = actor.getVel();
    if (actor_d < (ego_d + 2) && actor_d > (ego_d - 2)) {
      actor_s += ((double)ego_prev_path_size*kSimInterval*actor_vel);
      if ((actor_s > ego_s) && ((actor_s-ego_s) < kDistanceThreshold)){
        ret = true;
        break;
      }
    }
  }
  return ret;
}

vector<string> BehaviorPlanner::getAvailableStates(string state, int lane) {
  vector<string> states;
  states.push_back("KL");

  if (state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (lane != kNumAvailableLanes - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    } 
  } else if (state.compare("PLCR") == 0) {
    if (lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  return states;
}

vector<Ego> 




Actor::Actor(const vector<double> &actor){
  lon_vel = actor[3];
  lat_vel = actor[4];
  vel = sqrt(pow(lon_vel, 2.) + pow(lat_vel, 2.));
  s = actor[5];
  d = actor[6];
}

double Actor::getS(void) {
  return s;
}

double Actor::getD(void) {
  return d;
}

double Actor::getVel(void) {
  return vel;
}