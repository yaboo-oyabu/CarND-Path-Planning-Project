#include <string>
#include <vector>
#include <functional>
#include "helpers.h"
#include "spline.h"
#include "vehicle.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::string;
using std::vector;

const int kTrajectoryLength = 50;

///////////////////
// Public methods.
///////////////////

// For an ego vehicle.
Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed,
         const vector<double> &previous_path_x, const vector<double> &previous_path_y, 
         const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
         const vector<double> &map_waypoints_s) {

  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed_mph = speed;
  this->speed_mps = this->speed_mph * kMphToMps;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->prev_size = previous_path_x.size();
}

// For an ado vehicle.
Vehicle::Vehicle(double x, double y, double s, double d, double vx, double vy,
                 const vector<double> &map_waypoints_x,
                 const vector<double> &map_waypoints_y,
                 const vector<double> &map_waypoints_s) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = calculateYaw(vx, vy);
  this->speed_mps = calculateSpeed(vx, vy);
  this->speed_mph = this->speed_mps * kMpsToMph;
  this->previous_path_x = {};
  this->previous_path_y = {};
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->prev_size = previous_path_x.size();
}

double Vehicle::getX(void) { return this->x; }
double Vehicle::getY(void) { return this->y; }
double Vehicle::getD(void) { return this->d; }
double Vehicle::getS(void) { return this->s; }
double Vehicle::getYaw(void) { return this->yaw; }
double Vehicle::getSpeedMph(void) { return this->speed_mph; }
double Vehicle::getSpeedMps(void) { return this->speed_mps; }


void Vehicle::generateTrajectory(
    const vector<double> &s_start, const vector<double> &d_start,
    const vector<double> &s_end, const vector<double> &d_end, const double T,
    vector<double> &traj_x, vector<double> &traj_y, bool overwrite) {

  double ref_x = this->x;
  double ref_y = this->y;
  double ref_yaw = deg2rad(this->yaw);

  
  double s;
  double d;
  double t = 0.0;


  vector<double> s_;
  vector<double> d_;
  if (this->prev_size >= 4) {
    for (int i = 1; i < 4; i++) {
      double theta = atan2(
          this->previous_path_x[i] - this->previous_path_x[i-1],
          this->previous_path_y[i] - this->previous_path_y[i-1]);
      vector<double> sd = getFrenet(this->previous_path_x[i],
                                    this->previous_path_y[i],
                                    theta, map_waypoints_x, map_waypoints_y);
      s_.push_back(sd[0]);
      d_.push_back(sd[1]);
    }
    vector<double> s_start_ = {
    s_[2],
    (s_[2]-s_[1]) * 50,
    ((s_[2]-s_[1]) - (s_[1]-s_[0])) * 50};

    std::cout << "s_start_: ";
    printVector(s_start_);
  }


  /////////////////////////
  // spline
  // vector<double> s_start = {};
  // vector<double> d_start = {};
  // 後半の軌跡を更新するようにしようか。
  vector<double> s_coefs = JMT(s_start, s_end, T);
  vector<double> d_coefs = JMT(d_start, d_end, T);

  if (this->prev_size <= 4) {
    vector<double> points_x;
    vector<double> points_y;
    t = 0.0;
    while (t <= T) {
      s = generateS(t, s_coefs);
      d = generateD(t, d_coefs);
      vector<double> xy = getXY(
          s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      points_x.push_back(xy[0]);
      points_y.push_back(xy[1]);
      t += 0.5;
    }

    tk::spline sp;
    sp.set_points(points_x, points_y);


    t = 0.0;
    // t = T - (kSimInterval * prev_size);
    std::cout << "t: " << t << std::endl;
    while (t <= T) {
      s = generateS(t, s_coefs);
      d = generateD(t, d_coefs);
      vector<double> xy = getXY(
          s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);    
      traj_x.push_back(xy[0]);
      traj_y.push_back(sp(xy[0]));
      // std::cout << " t: " << t
      //           << " s: " << s
      //           << " d: " << d
      //           << " x: " << xy[0]
      //           << " y: " << xy[1] << std::endl;
      t += kSimInterval;
    }
  } else {
    for (int i = 0; i < this->prev_size; i++) {
      traj_x.push_back(this->previous_path_x[i]);
      traj_y.push_back(this->previous_path_y[i]);
    }
  }
}

vector<double> Vehicle::JMT(
    const vector<double> &start, const vector<double> &end, const double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A(3, 3);
  A <<   pow(T, 3),    pow(T, 4),    pow(T, 5),
       3*pow(T, 2),  4*pow(T, 3),  5*pow(T, 4),
               6*T, 12*pow(T, 2), 20*pow(T, 3);
               
  MatrixXd X(3, 1);
  X << end[0] - (start[0] + start[1]*T + 0.5*start[2]*pow(T,2)),
       end[1] - (start[1] + start[2]*T),
       end[2] - start[2];
       
  MatrixXd Y = A.inverse() * X;
  
  return {start[0], start[1], 0.5*start[2], Y(0), Y(1), Y(2)};
}

double Vehicle::generateX(double t, const vector<double> &x_coefs) {
  double total = 0.0;
  for (int i = 0; i < x_coefs.size(); i++) {
    total += x_coefs[i] * pow(t, i);
  }
  return total;
}


// void Vehicle::generateTrajectoryXY(const int &target_lane,
//                                    const double &target_velocity,
//                                    vector<double> &trajectory_x,
//                                    vector<double> &trajectory_y) {

//   vector<double> points_x;
//   vector<double> points_y;
//   double ref_x = this->x;
//   double ref_y = this->y;
//   double ref_yaw = deg2rad(this->yaw);

//   if (this->prev_size < 2) {
//     //Use two points that make the path tangent to the car
//     double prev_x = this->x - cos(this->yaw);
//     double prev_y = this->y - sin(this->yaw);
//     points_x.push_back(prev_x);
//     points_x.push_back(this->x);
//     points_y.push_back(prev_y);
//     points_y.push_back(this->y);
//   } else {
//     ref_x = previous_path_x[this->prev_size-1];
//     ref_y = previous_path_y[this->prev_size-1];
//     double ref_x_prev = previous_path_x[this->prev_size-2];
//     double ref_y_prev = previous_path_y[this->prev_size-2];
//     ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    
//     //Use two points that make the path tangent to the previous path's end point
//     points_x.push_back(ref_x_prev);
//     points_x.push_back(ref_x);
//     points_y.push_back(ref_y_prev);
//     points_y.push_back(ref_y);
//   }

//   ////////////////////////////////////////////////////////////
//   // lane change behavior is detemined by these 3 points.
//   ////////////////////////////////////////////////////////////
//   vector<double> next_wp0 = getXY(this->s+30, (2+4*target_lane), this->map_waypoints_s,
//                                   this->map_waypoints_x, this->map_waypoints_y);
//   vector<double> next_wp1 = getXY(this->s+60, (2+4*target_lane), this->map_waypoints_s,
//                                   this->map_waypoints_x, this->map_waypoints_y);
//   vector<double> next_wp2 = getXY(this->s+90, (2+4*target_lane), this->map_waypoints_s,
//                                   this->map_waypoints_x, this->map_waypoints_y);

//   points_x.push_back(next_wp0[0]);
//   points_x.push_back(next_wp1[0]);
//   points_x.push_back(next_wp2[0]);
//   points_y.push_back(next_wp0[1]);
//   points_y.push_back(next_wp1[1]);
//   points_y.push_back(next_wp2[1]);
  
//   // Global coordinate TO Reference coordinate
//   for (int i = 0; i < points_x.size(); i++) {
//     double shift_x = points_x[i] - ref_x;
//     double shift_y = points_y[i] - ref_y;
//     points_x[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
//     points_y[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
//   }

//   // create a spline.
//   tk::spline s;

//   // set (x, y) points to the spline.
//   s.set_points(points_x, points_y);

//   // Start with all of the previous path points from last time.
//   for (int i = 0; i < this->prev_size; i++) {
//     trajectory_x.push_back(this->previous_path_x[i]);
//     trajectory_y.push_back(this->previous_path_y[i]);
//   }

//   double target_x = 30.0;
//   double target_y = s(target_x);
//   double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
//   double x_add_on = 0.0;
//   for (int i = 0; i < (50 - this->prev_size); i++) {
//     double N = (target_dist/(.02*target_velocity/2.24));
//     double x_point = x_add_on + (target_x)/N;
//     double y_point = s(x_point);

//     x_add_on = x_point;

//     double x_ref = x_point;
//     double y_ref = y_point;

//     x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
//     y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

//     // Note that ref_x and x_ref are different!
//     x_point += ref_x;
//     y_point += ref_y;

//     trajectory_x.push_back(x_point);
//     trajectory_y.push_back(y_point);
//   }
// }

// void Vehicle::generateTrajectorySD(const int &target_lane,
//                                    const double &target_velocity,
//                                    vector<double> &trajectory_s,
//                                    vector<double> &trajectory_d) {
//   vector<double> trajectory_x;
//   vector<double> trajectory_y;
//   generateTrajectoryXY(target_lane, target_velocity, trajectory_x, trajectory_y);
//   convertTrajectory(trajectory_x, trajectory_y, trajectory_s, trajectory_d);
// }

// void Vehicle::convertTrajectory(const vector<double> &trajectory_x,
//                                 const vector<double> &trajectory_y,
//                                 vector<double> &trajectory_s,
//                                 vector<double> &trajectory_d) {
//   int trajectory_length = trajectory_x.size();
//   double theta;
//   vector<double> sd;

//   for (int i = 1; i < trajectory_length; i++) {
//     theta = atan2(trajectory_x[i] - trajectory_x[i-1],
//                   trajectory_y[i] - trajectory_y[i-1]);
//     sd = getFrenet(trajectory_x[i], trajectory_y[i], theta,
//                    map_waypoints_x, map_waypoints_y);
//     trajectory_s.push_back(sd[0]);
//     trajectory_d.push_back(sd[1]);
//   }
// }

///////////////////
// Private methods.
///////////////////
double Vehicle::calculateSpeed(const double &vx, const double &vy) {
  return sqrt(pow(vx, 2) + pow(vy, 2));
}

double Vehicle::calculateYaw(const double &vx, const double &vy) {
  return atan2(vy, vx);
}
