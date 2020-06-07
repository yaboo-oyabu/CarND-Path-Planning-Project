#ifndef BEHAVIOR_PLANNING_H
#define BEHAVIOR_PLANNING_H

#include <vector>
#include <math.h>
#include <string>

using std::vector;
using std::string;

static const double kSimInterval = 0.02;
static const double kDistanceThreshold = 30.0;

class Actor {
 public:
  Actor(const vector<double> &actor);
  ~Actor() {};
  double getS(void);
  double getD(void);
  double getVel(void);

 private:
  double s;
  double d;
  double lon_vel;
  double lat_vel;
  double vel;  
};

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

class BehaviorPlanner {
 public:
  BehaviorPlanner(const vector< vector<double> > &sensor_fusion);
  ~BehaviorPlanner() {};

  bool isLeadVehicleTooClose(
      double ego_s, double ego_d, int ego_prev_path_size);

  vector<string> getAvailableStates(string state);

 private:
  vector<Actor> actors;
};

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

vector<string> BehaviorPlanner::getAvailableStates(string state) {
  vector<string> states;
  states.push_back(state);
  return states;
}
#endif // BEHAVIOR_PLANNING_H
