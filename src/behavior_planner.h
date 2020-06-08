#ifndef BEHAVIOR_PLANNING_H
#define BEHAVIOR_PLANNING_H

#include <vector>
#include <math.h>
#include <string>

using std::vector;
using std::string;

const double kSimInterval = 0.02;
const double kDistanceThreshold = 30.0;
const int kNumAvailableLanes = 3;

class Ego {
 public:
  Ego() {};
  ~Ego() {};
 private:
  double s;
};

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

class BehaviorPlanner {
 public:
  BehaviorPlanner(const vector< vector<double> > &sensor_fusion);
  ~BehaviorPlanner() {};

  bool isLeadVehicleTooClose(
      double ego_s, double ego_d, int ego_prev_path_size);

  vector<string> getAvailableStates(string state, int lane);

 private:
  vector<Actor> actors;
};

#endif // BEHAVIOR_PLANNING_H
