#include <string>
#include <vector>
#include "helpers.h"
#include "spline.h"
#include "vehicle.h"

using std::string;
using std::vector;

/*
 * Ego Vehicle
 */
Ego::Ego(double x, double y, double s, double d, double yaw, double speed,
         const vector<double> &previous_path_x, const vector<double> &previous_path_y, 
         const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
         const vector<double> &map_waypoints_s) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->prev_size = previous_path_x.size();
}

void Ego::generateTrajectory(int target_lane, double target_velocity,
                             vector<double> &next_x_vals, vector<double> &next_y_vals) {
  // int target_lane, double target_distance, ...

  vector<double> points_x;
  vector<double> points_y;
  double ref_x = this->x;
  double ref_y = this->y;
  double ref_yaw = deg2rad(this->yaw);

  if (this->prev_size < 2) {
    //Use two points that make the path tangent to the car
    double prev_x = this->x - cos(this->yaw);
    double prev_y = this->y - sin(this->yaw);
    points_x.push_back(prev_x);
    points_x.push_back(this->x);
    points_y.push_back(prev_y);
    points_y.push_back(this->y);
  } else {
    ref_x = previous_path_x[this->prev_size-1];
    ref_y = previous_path_y[this->prev_size-1];
    double ref_x_prev = previous_path_x[this->prev_size-2];
    double ref_y_prev = previous_path_y[this->prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    
    //Use two points that make the path tangent to the previous path's end point
    points_x.push_back(ref_x_prev);
    points_x.push_back(ref_x);
    points_y.push_back(ref_y_prev);
    points_y.push_back(ref_y);
  }

  ////////////////////////////////////////////////////////////
  // lane change behavior is detemined by these 3 points.
  ////////////////////////////////////////////////////////////
  vector<double> next_wp0 = getXY(this->s+30, (2+4*target_lane), this->map_waypoints_s,
                                  this->map_waypoints_x, this->map_waypoints_y);
  vector<double> next_wp1 = getXY(this->s+60, (2+4*target_lane), this->map_waypoints_s,
                                  this->map_waypoints_x, this->map_waypoints_y);
  vector<double> next_wp2 = getXY(this->s+90, (2+4*target_lane), this->map_waypoints_s,
                                  this->map_waypoints_x, this->map_waypoints_y);

  points_x.push_back(next_wp0[0]);
  points_x.push_back(next_wp1[0]);
  points_x.push_back(next_wp2[0]);
  points_y.push_back(next_wp0[1]);
  points_y.push_back(next_wp1[1]);
  points_y.push_back(next_wp2[1]);

  // Global coordinate TO Reference coordinate
  for (int i = 0; i < points_x.size(); i++) {
    double shift_x = points_x[i] - ref_x;
    double shift_y = points_y[i] - ref_y;
    points_x[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    points_y[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
  }

  // create a spline.
  tk::spline s;

  // set (x, y) points to the spline.
  s.set_points(points_x, points_y);

  // Start with all of the previous path points from last time.
  for (int i = 0; i < this->prev_size; i++) {
    next_x_vals.push_back(this->previous_path_x[i]);
    next_y_vals.push_back(this->previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
  double x_add_on = 0.0;
  for (int i = 0; i < (50 - this->prev_size); i++) {
    double N = (target_dist/(.02*target_velocity/2.24));
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    // Note that ref_x and x_ref are different!
    x_point += ref_x;
    y_point += ref_y;

    // std::cout << i << ": " << x_point << ", " << y_point << std::endl;
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

/*
 * Ado Vehicle
 */
Ado::Ado(double id, double x, double y, double s, double d, double vx, double vy){
  this->id = id;
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->vx = vx;
  this->vy = vy;
  this->v = sqrt(pow(vx, 2) + pow(vy, 2));
}