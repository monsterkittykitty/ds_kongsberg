//
// Created by jvaccaro on 8/20/19.
//
/**
* Copyright 2019 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ds_vfr_to_gga/ds_vfr_to_gga.h"
#include "ds_nmea_parsers/util.h"
#include "ds_util/int_to_hex.h"

namespace ds_nav {

VfrToGga::VfrToGga()
    : DsProcess()
{
}

VfrToGga::VfrToGga(int argc, char* argv[], const std::string& name)
    : DsProcess(argc, argv, name)
{
}

VfrToGga::~VfrToGga() = default;

// 0   1          2            3 4 5             6          7          8     9     10  11   12
// VFR 2018/03/23 00:00:00.821 9 0 SOLN_DEADRECK 179.035734 -34.905139 0.000 2.110 100 0.00 197.92

std::pair<bool, std::string>
VfrToGga::pack_vfr_message(const ds_core_msgs::RawData& msg, std::string soln, std::string beacon1, std::string beacon2)
{
  std::string vfr_string(msg.data.data(), msg.data.data()+msg.data.size());
  std::vector<std::string> vfr_list;
  boost::split(vfr_list, vfr_string, boost::is_any_of(" "));
  if (vfr_list.size() != 14){
    return {false, vfr_string};
  }
  if (vfr_list[0] != "VFR"){
    return {false, vfr_string};
  }
  if (vfr_list[3] != beacon1){
    return {false, vfr_string};
  }
  if (vfr_list[4] != beacon2){
    return {false, vfr_string};
  }
  if (vfr_list[5] != soln){
//    ROS_ERROR_STREAM("WANT SOLN "<<soln<<" GOT "<<vfr_list[5]);
    return {false, vfr_string};
  }
  auto lon = std::stod(vfr_list[6]);
  auto lat = std::stod(vfr_list[7]);

  std::stringstream ss;
  ss << "$GPGGA" << ",";
  ss << ds_nmea_msgs::to_nmea_utc_str(ros::Time::now()) << ","; // 1. Time (UTC)
  ss << ds_nmea_msgs::to_nmea_lat_string(lat); // 2. Lat  3. N/S
  ss << ds_nmea_msgs::to_nmea_lon_string(lon); // 4. Lon  5. E/W
  ss << "2" << ","; // 6. GPS Quality indicator (set to GPS, not DGPS)
  ss << "08" << ","; // 7. Number of satellites in view, pretend it's sufficient
  ss << "1.0" << ","; // 8. Horizontal Dilution of precision
  ss << vfr_list[8] << ","; // 9. Antenna altitude above/below mean-sea-level
  ss << "M" << ","; // 10. Units of antenna altitude, meters
  ss << "0.000" << ","; // 11) Geoidal separation, the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
  ss << "M" << ","; // 12) Units of geoidal separation, meters
  ss << "1012"; // 13) Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
  ss << "*"; // 14) Differential reference station ID, 0000-1023
  // Now calculate the checksum
  uint16_t checksum = ds_nmea_msgs::calculate_checksum(ss.str());

  // Append the checksum
  std::string checksum_str = ds_util::int_to_hex<uint16_t>(checksum);
  ss << checksum_str << "\r\n"; // 15) Checksum
  return {true, ss.str()};
}

void
VfrToGga::setupConnections()
{
  ds_base::DsProcess::setupConnections();
  m_vfr_conn = addConnection("vfr", boost::bind(&VfrToGga::_on_vfr_msg, this, _1));
  m_gga_conn = addConnection("gga", boost::bind(&VfrToGga::_on_gga_msg, this, _1));
  m_soln = ros::param::param<std::string>("~solution", "SOLN_DEADRECK");
  m_beacon1 = ros::param::param<std::string>("~beacon1", "9");
  m_beacon2 = ros::param::param<std::string>("~beacon2", "0");
  ROS_ERROR_STREAM("SOLUTION IS "<<m_beacon1 << " " << m_beacon2 << " " << m_soln);

}

void
VfrToGga::_on_vfr_msg(const ds_core_msgs::RawData& msg)
{
//  ROS_ERROR_STREAM(ros::Time::now());
  bool ok = false;
  std::string out_msg = "";
  std::tie(ok, out_msg) = pack_vfr_message(msg, m_soln, m_beacon1, m_beacon2);
  if (ok){
    m_gga_conn->send(out_msg);
  } else {
  }
}

void
VfrToGga::_on_gga_msg(const ds_core_msgs::RawData& msg)
{
}

//void
//NavstateToGga::_on_spitter_timer(const ros::TimerEvent&){
//  if (m_is_spitter){
//    ds_nav_msgs::NavState msg;
//    msg.ds_header.io_time = ros::Time::now();
//    msg.header.stamp = msg.ds_header.io_time;
//    msg.lat = -15;
//    msg.lon = 85;
//    msg.down = 20;
//    std::string out_msg = pack_message(msg);
//    m_gga_conn->send(out_msg);
//  }
//}
} //namespace
