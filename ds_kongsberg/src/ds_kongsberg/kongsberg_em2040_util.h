//
// Created by jvaccaro on 7/17/19.
//

#ifndef PROJECT_KONGSBERG_EM2040_UTIL_H
#define PROJECT_KONGSBERG_EM2040_UTIL_H

#define M_PI_RAD 3.14159
#define M_PI_DEG 180.0

#include <string>

namespace ds_kongsberg {

std::string filename(std::string directory, int count, std::string base, std::string extension)
{
  std::stringstream filename_ss;
  auto facet = new boost::posix_time::time_facet("%Y%m%d_%H%M");
  filename_ss.imbue(std::locale(filename_ss.getloc(), facet));
  filename_ss << directory;
  filename_ss << std::setfill('0') << std::setw(4) << count;
  filename_ss << "_" << boost::posix_time::second_clock::universal_time();
  filename_ss << "_" << base;
  filename_ss << extension;
  return filename_ss.str();
}

double deg_to_rad(double deg){
  return deg*M_PI_RAD/M_PI_DEG;
}

double rad_to_deg(double rad){
  return rad*M_PI_DEG/M_PI_RAD;
}

}

#endif //PROJECT_KONGSBERG_EM2040_UTIL_H
