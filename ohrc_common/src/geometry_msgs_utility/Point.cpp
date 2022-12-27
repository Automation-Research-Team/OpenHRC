/**
 * @file Point.cpp
 * @author Shunki Itadera
 * @date Oct. 2018
 * @brief utility library for Point message
 **/

#include "ohrc_common/geometry_msgs_utility/Point.h"

namespace geometry_msgs_utility {
Point::Point(int order, double cutoff_freq, double sampling_freq) {
  initLPF(order, cutoff_freq, sampling_freq);
}

Point::~Point() {
  // deleted because this vector conteners std::unique_ptr
  //  for (int i = 0; i < 3; ++i)
  //    delete _filter[i];
}

void Point::initLPF(int order, double cutoff_freq, double sampling_freq) {
  for (int i = 0; i < 3; ++i)
    _filter.push_back(std::shared_ptr<butterworth>(new butterworth(order, cutoff_freq, sampling_freq)));

  delta_t = 1.0 / sampling_freq;
}

void Point::LPF(geometry_msgs::Point raw_point, geometry_msgs::Point &filtered_point) {
  filtered_point.x = _filter[0]->filter(raw_point.x);
  filtered_point.y = _filter[1]->filter(raw_point.y);
  filtered_point.z = _filter[2]->filter(raw_point.z);
}

void Point::diff(geometry_msgs::Point point, geometry_msgs::Point &diff_point) {
  static geometry_msgs::Point old_point = point;
  diff_point.x = (point.x - old_point.x) / delta_t;
  diff_point.y = (point.y - old_point.y) / delta_t;
  diff_point.z = (point.z - old_point.z) / delta_t;
  old_point = point;
}

void Point::diff_LPF(geometry_msgs::Point point, geometry_msgs::Point &filtered_point) {
  geometry_msgs::Point diff_point;
  diff(point, diff_point);
  LPF(diff_point, filtered_point);
}

double dist(geometry_msgs::Point point1, geometry_msgs::Point point2) {
  return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
}

void initPoint(geometry_msgs::Point &point, double x, double y, double z) {
  point.x = x;
  point.y = y;
  point.z = z;
}
void initPoint(geometry_msgs::Point &point, double val) {
  initPoint(point, val, val, val);
}

void clamp(geometry_msgs::Point &point, double x_max, double y_max, double z_max, double x_min, double y_min, double z_min) {
  math_utility::clamp(point.x, x_max, x_min);
  math_utility::clamp(point.y, z_max, y_min);
  math_utility::clamp(point.z, z_max, y_min);
}

void clamp(geometry_msgs::Point &point, double max, double min) {
  clamp(point, max, max, max, min, min, min);
}

void clamp_moreThan(geometry_msgs::Point &point, double x, double y, double z) {
  if (point.x < x)
    point.x = x;

  if (point.y < y)
    point.y = y;

  if (point.z < z)
    point.z = z;
}

void clamp_moreThan(geometry_msgs::Point &point, double val) {
  clamp_moreThan(point, val, val, val);
}

void clamp_lessThan(geometry_msgs::Point &point, double x, double y, double z) {
  if (point.x > x)
    point.x = x;

  if (point.y > y)
    point.y = y;

  if (point.z > z)
    point.z = z;
}

void clamp_lessThan(geometry_msgs::Point &point, double val) {
  clamp_lessThan(point, val, val, val);
}

geometry_msgs::Point error(geometry_msgs::Point point1, geometry_msgs::Point point2) {
  geometry_msgs::Point errorPoint;
  errorPoint.x = point1.x - point2.x;
  errorPoint.y = point1.y - point2.y;
  errorPoint.z = point1.z - point2.z;

  return errorPoint;
}

geometry_msgs::Point sum(std::vector<geometry_msgs::Point> points) {
  geometry_msgs::Point sumPoint;
  for (int i = 0; i < points.size(); i++) {
    sumPoint.x += points[i].x;
    sumPoint.y += points[i].y;
    sumPoint.z += points[i].z;
  }
  return sumPoint;
}

geometry_msgs::Point average(std::vector<geometry_msgs::Point> points) {
  geometry_msgs::Point averagePoint = sum(points);
  averagePoint.x /= points.size();
  averagePoint.y /= points.size();
  averagePoint.z /= points.size();

  return averagePoint;
}

}  // namespace geometry_msgs_utility
