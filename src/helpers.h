#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>

// for convenience
using std::string;
using std::vector;

string hasData(string s);
constexpr double pi();

double deg2rad(double x);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(
    double x, double y,
    const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(
    double x, double y, double theta,
    const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(
    double x, double y, double theta, 
    const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(
    double s, double d, const vector<double> &maps_s,
    const vector<double> &maps_x, const vector<double> &maps_y);

template<typename T>
void printVector(const T& t) {
  std::copy(t.cbegin(), t.cend(), std::ostream_iterator<typename T::value_type>(std::cout, ", "));
  std::cout << std::endl;
}

#endif  // HELPERS_H