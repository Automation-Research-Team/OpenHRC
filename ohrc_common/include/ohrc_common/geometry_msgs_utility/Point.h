#ifndef POINT_H
#define POINT_H

#include <geometry_msgs/Point.h>
#include <ohrc_common/math_utility.h>
//#include "ohrc_common/filter_utility.h"
#include "ohrc_common/filter_utility/butterworth.h"

namespace geometry_msgs_utility {

double dist(geometry_msgs::Point point1, geometry_msgs::Point point2);

void initPoint(geometry_msgs::Point &point, double x, double y, double z);
void initPoint(geometry_msgs::Point &point, double val);

void clamp(geometry_msgs::Point &point, double x_max, double y_max, double z_max, double x_min, double y_min, double z_min);
void clamp(geometry_msgs::Point &point, double max, double min);

void clamp_moreThan(geometry_msgs::Point &point, double x, double y, double z);
void clamp_moreThan(geometry_msgs::Point &point, double val);

void clamp_lessThan(geometry_msgs::Point &point, double x, double y, double z);
void clamp_lessThan(geometry_msgs::Point &point, double val);

geometry_msgs::Point error(geometry_msgs::Point point1, geometry_msgs::Point point2);
geometry_msgs::Point sum(std::vector<geometry_msgs::Point> points);
geometry_msgs::Point average(std::vector<geometry_msgs::Point> points);

class Point {
private:
  double delta_t;
  std::vector<std::shared_ptr<butterworth>> _filter;

public:
  Point(int order, double cutoff_freq, double sampling_freq);
  Point(){};
  ~Point();

  void LPF(geometry_msgs::Point raw_point, geometry_msgs::Point &filtered_point);
  void diff(geometry_msgs::Point point, geometry_msgs::Point &diff_point);
  void diff_LPF(geometry_msgs::Point point, geometry_msgs::Point &filtered_point);
  void initLPF(int order, double cutoff_freq, double sampling_freq);
};
}  // namespace geometry_msgs_utility

#endif  // POINT_H
