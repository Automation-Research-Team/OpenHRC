#include "ohrc_common/std_utility.h"

bool std_utility::exist_in(const std::vector<std::string>& c, const std::string& v, int& index) {
  auto itr = std::find(c.begin(), c.end(), v);
  bool found = itr != c.end();

  if (found)
    index = std::distance(c.begin(), itr);
  else
    index = -1;

  return found;
}

bool std_utility::exist_in(const std::vector<std::string>& c, const std::string& v) {
  int index;
  return exist_in(c, v, index);
}

#include <iomanip>
#include <iostream>

std::string std_utility::getDatetimeStr() {
  time_t t = time(nullptr);
  const tm* localTime = localtime(&t);
  std::stringstream s;
  s << localTime->tm_year + 1900;
  s << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1;
  s << std::setw(2) << std::setfill('0') << localTime->tm_mday << "_";
  s << std::setw(2) << std::setfill('0') << localTime->tm_hour;
  s << std::setw(2) << std::setfill('0') << localTime->tm_min;
  // s << std::setw(2) << std::setfill('0') << localTime->tm_sec;

  return s.str();
}
