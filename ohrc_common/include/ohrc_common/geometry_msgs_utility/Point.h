#ifndef POINT_H
#define POINT_H

#include <geometry_msgs/msg/point.hpp>
#include <ohrc_common/math_utility.h>
//#include "ohrc_common/filter_utility.h"
#include "ohrc_common/filter_utility/butterworth.h"

namespace geometry_msgs_utility {

double dist(geometry_msgs::msg::Point point1, geometry_msgs::msg::Point point2);

void initPoint(geometry_msgs::msg::Point &point, double x, double y, double z);
void initPoint(geometry_msgs::msg::Point &point, double val);

void clamp(geometry_msgs::msg::Point &point, double x_max, double y_max, double z_max, double x_min, double y_min, double z_min);
void clamp(geometry_msgs::msg::Point &point, double max, double min);

void clamp_moreThan(geometry_msgs::msg::Point &point, double x, double y, double z);
void clamp_moreThan(geometry_msgs::msg::Point &point, double val);

void clamp_lessThan(geometry_msgs::msg::Point &point, double x, double y, double z);
void clamp_lessThan(geometry_msgs::msg::Point &point, double val);

geometry_msgs::msg::Point error(geometry_msgs::msg::Point point1, geometry_msgs::msg::Point point2);
geometry_msgs::msg::Point sum(std::vector<geometry_msgs::msg::Point> points);
geometry_msgs::msg::Point average(std::vector<geometry_msgs::msg::Point> points);

class Point {
private:
  double delta_t;
  std::vector<std::shared_ptr<butterworth>> _filter;

public:
  Point(int order, double cutoff_freq, double sampling_freq);
  Point(){};
  ~Point();

  void LPF(geometry_msgs::msg::Point raw_point, geometry_msgs::msg::Point &filtered_point);
  void diff(geometry_msgs::msg::Point point, geometry_msgs::msg::Point &diff_point);
  void diff_LPF(geometry_msgs::msg::Point point, geometry_msgs::msg::Point &filtered_point);
  void initLPF(int order, double cutoff_freq, double sampling_freq);
};
}  // namespace geometry_msgs_utility

#endif  // POINT_H
